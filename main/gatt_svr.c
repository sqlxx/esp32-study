#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bldc.h"
#include "bmi088.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "gatt_svr.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "servo.h"

/* 自定义 Service UUID: 0000ffe0-0000-1000-8000-00805f9b34fb */
static const ble_uuid16_t svc_uuid =
    BLE_UUID16_INIT(0xFFE0);

/* 自定义 Characteristic UUID: 0000ffe1-0000-1000-8000-00805f9b34fb */
static const ble_uuid16_t chr_uuid =
    BLE_UUID16_INIT(0xFFE1);

/* IMU 通知：0000ffe2-0000-1000-8000-00805f9b34fb */
static const ble_uuid16_t imu_chr_uuid =
    BLE_UUID16_INIT(0xFFE2);

/* 无刷：0000ffe3-0000-1000-8000-00805f9b34fb */
static const ble_uuid16_t bldc_chr_uuid =
    BLE_UUID16_INIT(0xFFE3);

#define IMU_PAYLOAD_LEN 12
#define BLDC_PAYLOAD_LEN 14

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_imu_val_handle;
static uint16_t s_bldc_val_handle;
static bool s_imu_notify;
static bool s_bldc_notify;
static uint8_t s_imu_last[IMU_PAYLOAD_LEN];
static uint16_t s_imu_last_len;
static uint8_t s_bldc_last[BLDC_PAYLOAD_LEN];
static uint16_t s_bldc_last_len;

/* 最近一次 Write 的内容（最多 20 字节） */
#define WRITE_BUF_SIZE 20
static uint8_t write_buf[WRITE_BUF_SIZE];
static uint16_t write_len;

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);
static int imu_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static int bldc_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &chr_uuid.u,
                .access_cb = chr_access_cb,
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &imu_chr_uuid.u,
                .access_cb = imu_chr_access_cb,
                .val_handle = &s_imu_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &bldc_chr_uuid.u,
                .access_cb = bldc_chr_access_cb,
                .val_handle = &s_bldc_val_handle,
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0,
            },
        },
    },
    {
        0,
    },
};

static int16_t clamp_i16(long v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return (int16_t)-32768;
    }
    return (int16_t)v;
}

static uint16_t clamp_u16_frac(float v, float scale)
{
    long x = lroundf(v * scale);
    if (x < 0) {
        return 0;
    }
    if (x > 65535) {
        return 65535;
    }
    return (uint16_t)x;
}

static void pack_bldc(uint8_t out[BLDC_PAYLOAD_LEN], const bldc_status_t *st)
{
    float deg = st->theta_rad * (180.0f / 3.14159265f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    int16_t rpm_x10 = clamp_i16(lroundf(st->rpm * 10.0f));
    uint16_t fields[5] = {
        clamp_u16_frac(st->u, 1000.0f),
        clamp_u16_frac(st->v, 1000.0f),
        clamp_u16_frac(st->w, 1000.0f),
        clamp_u16_frac(deg, 10.0f),
        clamp_u16_frac(st->modulation, 1000.0f),
    };
    memcpy(out, &rpm_x10, 2);
    out[2] = st->enabled ? 1 : 0;
    out[3] = 0;
    memcpy(out + 4, fields, 10);
}

static char *trim_cmd(char *cmd, uint16_t len)
{
    cmd[len] = '\0';
    char *p = cmd;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    char *end_trim = p + strlen(p);
    while (end_trim > p && isspace((unsigned char)end_trim[-1])) {
        *--end_trim = '\0';
    }
    for (char *q = p; *q; q++) {
        *q = (char)tolower((unsigned char)*q);
    }
    return p;
}

static int handle_bldc_cmd(const char *p)
{
    if (strcmp(p, "on") == 0 || strcmp(p, "start") == 0) {
        return bldc_enable() == ESP_OK ? 0 : BLE_ATT_ERR_UNLIKELY;
    }
    if (strcmp(p, "off") == 0 || strcmp(p, "stop") == 0) {
        bldc_disable();
        return 0;
    }

    const char *num = NULL;
    if (strncmp(p, "rpm:", 4) == 0) {
        num = p + 4;
    } else if (strncmp(p, "m:", 2) == 0) {
        char *end = NULL;
        float m = strtof(p + 2, &end);
        if (end == p + 2 || (end && *end != '\0')) {
            printf("BLDC: m 格式无效，示例 m:0.15\n");
            return 0;
        }
        return bldc_set_modulation(m) == ESP_OK ? 0 : BLE_ATT_ERR_UNLIKELY;
    } else if (*p == '\0' || (!isdigit((unsigned char)*p) && *p != '-' && *p != '+')) {
        printf("BLDC: 未知命令（on/off/rpm:30/m:0.15）\n");
        return 0;
    } else {
        num = p;
    }

    char *end = NULL;
    float rpm = strtof(num, &end);
    if (end == num || (end && *end != '\0')) {
        printf("BLDC: rpm 格式无效\n");
        return 0;
    }
    return bldc_set_openloop_rpm(rpm) == ESP_OK ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static void pack_imu(uint8_t out[IMU_PAYLOAD_LEN], const bmi088_vec3_t *acc, const bmi088_vec3_t *gyr)
{
    int16_t v[6] = {
        clamp_i16(lroundf(acc->x * 1000.0f)),
        clamp_i16(lroundf(acc->y * 1000.0f)),
        clamp_i16(lroundf(acc->z * 1000.0f)),
        clamp_i16(lroundf(gyr->x * 10.0f)),
        clamp_i16(lroundf(gyr->y * 10.0f)),
        clamp_i16(lroundf(gyr->z * 10.0f)),
    };
    memcpy(out, v, IMU_PAYLOAD_LEN);
}

static const char *reset_reason_str(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:  return "poweron";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:  return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    default:               return "other";
    }
}

/* 组装设备状态文本，供 GATT Read 返回 */
static int build_device_info(char *buf, size_t buflen)
{
    uint8_t mac[6];
    const char *name = ble_svc_gap_device_name();
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    uint32_t heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();

    esp_read_mac(mac, ESP_MAC_BT);

    bldc_status_t bldc;
    bldc_get_status(&bldc);

    return snprintf(buf, buflen,
                    "name=%s\n"
                    "mac=%02x:%02x:%02x:%02x:%02x:%02x\n"
                    "uptime=%lus\n"
                    "heap=%lu\n"
                    "min_heap=%lu\n"
                    "rst=%s\n"
                    "servo=%d\n"
                    "scan=%d\n"
                    "spd=%d\n"
                    "bldc=%d\n"
                    "rpm=%.0f",
                    name ? name : "?",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                    (unsigned long)uptime_s,
                    (unsigned long)heap,
                    (unsigned long)min_heap,
                    reset_reason_str(esp_reset_reason()),
                    servo_get_angle(),
                    servo_is_scanning() ? 1 : 0,
                    servo_get_scan_speed(),
                    bldc.enabled ? 1 : 0,
                    (double)bldc.rpm);
}

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        char info[256];
        int len = build_device_info(info, sizeof(info));
        if (len < 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        if ((size_t)len >= sizeof(info)) {
            len = sizeof(info) - 1;
        }

        printf("GATT Read:\n%s\n", info);
        rc = os_mbuf_append(ctxt->om, info, len);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0 || om_len > WRITE_BUF_SIZE) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        rc = ble_hs_mbuf_to_flat(ctxt->om, write_buf, WRITE_BUF_SIZE, &write_len);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        printf("GATT Write (%u bytes):", write_len);
        for (uint16_t i = 0; i < write_len; i++) {
            printf(" %02x", write_buf[i]);
        }
        printf("  |  ASCII: ");
        for (uint16_t i = 0; i < write_len; i++) {
            char c = (write_buf[i] >= 32 && write_buf[i] < 127) ? write_buf[i] : '.';
            putchar(c);
        }
        printf("\n");

        /* 命令: scan / stop / spd:N(1~10) / 角度数字 */
        char cmd[WRITE_BUF_SIZE + 1];
        memcpy(cmd, write_buf, write_len);
        cmd[write_len] = '\0';

        char *p = cmd;
        while (*p && isspace((unsigned char)*p)) {
            p++;
        }
        char *end_trim = p + strlen(p);
        while (end_trim > p && isspace((unsigned char)end_trim[-1])) {
            *--end_trim = '\0';
        }
        for (char *q = p; *q; q++) {
            *q = (char)tolower((unsigned char)*q);
        }

        if (strcmp(p, "scan") == 0) {
            if (servo_scan_start() != ESP_OK) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            return 0;
        }
        if (strcmp(p, "stop") == 0) {
            servo_scan_stop();
            return 0;
        }
        const char *num = NULL;
        if (strncmp(p, "spd:", 4) == 0) {
            num = p + 4;
        } else if (strncmp(p, "speed:", 6) == 0) {
            num = p + 6;
        }
        if (num != NULL) {
            char *end = NULL;
            long level = strtol(num, &end, 10);
            if (end == num || (end && *end != '\0')) {
                printf("舵机: spd 格式无效，示例 spd:5（1~10）\n");
                return 0;
            }
            if (servo_set_scan_speed((int)level) != ESP_OK) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            return 0;
        }

        if (*p == '\0' || (!isdigit((unsigned char)*p) && *p != '-' && *p != '+')) {
            printf("舵机: 未知命令（scan/stop/spd:1~10/角度）\n");
            return 0;
        }

        char *end = NULL;
        long angle = strtol(p, &end, 10);
        while (end && *end && isspace((unsigned char)*end)) {
            end++;
        }
        if (end == p || (end && *end != '\0')) {
            printf("舵机: 角度格式无效\n");
            return 0;
        }

        if (angle < 0 || angle > 180) {
            printf("舵机: 角度超出范围 0~180: %ld\n", angle);
            return 0;
        }

        if (servo_set_angle((int)angle) != ESP_OK) {
            printf("舵机: 设置失败\n");
            return BLE_ATT_ERR_UNLIKELY;
        }
        return 0;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int imu_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t zeros[IMU_PAYLOAD_LEN] = {0};
    const uint8_t *data = s_imu_last_len ? s_imu_last : zeros;
    int rc = os_mbuf_append(ctxt->om, data, IMU_PAYLOAD_LEN);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

void gatt_svr_on_connect(uint16_t conn_handle)
{
    s_conn_handle = conn_handle;
}

void gatt_svr_on_disconnect(void)
{
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_imu_notify = false;
    s_bldc_notify = false;
}

void gatt_svr_on_subscribe(uint16_t attr_handle, uint16_t conn_handle, bool notify_enabled)
{
    if (attr_handle == s_imu_val_handle) {
        s_conn_handle = conn_handle;
        s_imu_notify = notify_enabled;
        printf("IMU notify %s, conn=%u\n", notify_enabled ? "on" : "off", conn_handle);
        return;
    }
    if (attr_handle == s_bldc_val_handle) {
        s_conn_handle = conn_handle;
        s_bldc_notify = notify_enabled;
        printf("BLDC notify %s, conn=%u\n", notify_enabled ? "on" : "off", conn_handle);
    }
}

void gatt_svr_notify_bldc(const bldc_status_t *st)
{
    if (!s_bldc_notify || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || st == NULL) {
        return;
    }

    pack_bldc(s_bldc_last, st);
    s_bldc_last_len = BLDC_PAYLOAD_LEN;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_bldc_last, BLDC_PAYLOAD_LEN);
    if (om == NULL) {
        return;
    }

    (void)ble_gatts_notify_custom(s_conn_handle, s_bldc_val_handle, om);
}

static int bldc_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        uint8_t zeros[BLDC_PAYLOAD_LEN] = {0};
        if (s_bldc_last_len == 0) {
            bldc_status_t st;
            bldc_get_status(&st);
            pack_bldc(s_bldc_last, &st);
            s_bldc_last_len = BLDC_PAYLOAD_LEN;
        }
        const uint8_t *data = s_bldc_last_len ? s_bldc_last : zeros;
        int rc = os_mbuf_append(ctxt->om, data, BLDC_PAYLOAD_LEN);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0 || om_len > WRITE_BUF_SIZE) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        int rc = ble_hs_mbuf_to_flat(ctxt->om, write_buf, WRITE_BUF_SIZE, &write_len);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        char cmd[WRITE_BUF_SIZE + 1];
        memcpy(cmd, write_buf, write_len);
        return handle_bldc_cmd(trim_cmd(cmd, write_len));
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

void gatt_svr_notify_imu(const bmi088_vec3_t *acc, const bmi088_vec3_t *gyr)
{
    if (!s_imu_notify || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || acc == NULL || gyr == NULL) {
        return;
    }

    pack_imu(s_imu_last, acc, gyr);
    s_imu_last_len = IMU_PAYLOAD_LEN;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_imu_last, IMU_PAYLOAD_LEN);
    if (om == NULL) {
        return;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_imu_val_handle, om);
    if (rc != 0) {
        /* 连接繁忙时丢一帧，避免堵死采样任务 */
    }
}

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return ble_gatts_add_svcs(gatt_svr_svcs);
}
