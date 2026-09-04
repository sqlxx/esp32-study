#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
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

/* 最近一次 Write 的内容（最多 20 字节） */
#define WRITE_BUF_SIZE 20
static uint8_t write_buf[WRITE_BUF_SIZE];
static uint16_t write_len;

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
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
                0,
            },
        },
    },
    {
        0,
    },
};

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

    return snprintf(buf, buflen,
                    "name=%s\n"
                    "mac=%02x:%02x:%02x:%02x:%02x:%02x\n"
                    "uptime=%lus\n"
                    "heap=%lu\n"
                    "min_heap=%lu\n"
                    "rst=%s\n"
                    "servo=%d\n"
                    "scan=%d\n"
                    "spd=%d",
                    name ? name : "?",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                    (unsigned long)uptime_s,
                    (unsigned long)heap,
                    (unsigned long)min_heap,
                    reset_reason_str(esp_reset_reason()),
                    servo_get_angle(),
                    servo_is_scanning() ? 1 : 0,
                    servo_get_scan_speed());
}

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        char info[200];
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
