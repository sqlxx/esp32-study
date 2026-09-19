/*
 * ESP32 BLE 外设：广播连接后打印连接信息，并提供可读写 GATT 特征
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "as5600.h"
#include "bldc.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmi088.h"
#include "gatt_svr.h"
#include "servo.h"

static const char *TAG = "BLE";
static const char *IMU_TAG = "IMU";
static const char *ENC_TAG = "AS5600";
static const char *DEVICE_NAME = "ESP32-Hello";

static uint8_t own_addr_type;

static int gap_event_handler(struct ble_gap_event *event, void *arg);

static void print_mac(const char *label, const uint8_t *addr)
{
    printf("  %s: %02x:%02x:%02x:%02x:%02x:%02x\n",
           label, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

static void print_conn_desc(struct ble_gap_conn_desc *desc)
{
    printf("========== BLE 连接信息 ==========\n");
    printf("  连接句柄 (conn_handle): %d\n", desc->conn_handle);
    print_mac("本机地址 (our_id_addr)", desc->our_id_addr.val);
    print_mac("手机地址 (peer_id_addr)", desc->peer_id_addr.val);
    printf("  连接间隔 (conn_itvl): %d (单位: 1.25ms)\n", desc->conn_itvl);
    printf("  连接延迟 (conn_latency): %d\n", desc->conn_latency);
    printf("  超时时间 (supervision_timeout): %d (单位: 10ms)\n", desc->supervision_timeout);
    printf("  已加密: %s\n", desc->sec_state.encrypted ? "是" : "否");
    printf("  已认证: %s\n", desc->sec_state.authenticated ? "是" : "否");
    printf("  已绑定: %s\n", desc->sec_state.bonded ? "是" : "否");
    printf("==================================\n");
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    static const ble_uuid16_t adv_uuids16[] = {
        BLE_UUID16_INIT(0xFFE0),
    };
    const char *name;
    int rc;

    /* 广播里带上 Service UUID，方便 Web Bluetooth 按 UUID 过滤 */
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16 = adv_uuids16;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置广播数据失败, rc=%d", rc);
        return;
    }

    /* 设备名放扫描响应，避免广播包超长 */
    name = ble_svc_gap_device_name();
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置扫描响应失败, rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动广播失败, rc=%d", rc);
        return;
    }

    printf("正在广播，设备名: %s, Service: 0xFFE0\n", name);
    printf("可用 nRF Connect，或打开 web/index.html（Chrome + Web Bluetooth）\n");
    printf("0xFFE1: Write 角度 / scan / stop / spd:1~10\n");
    printf("0xFFE2: Notify IMU acc[g] + gyro[dps]\n");
    printf("0xFFE3: Write on/off/rpm:30 ，Notify 三相占空比\n");
    printf("0xFFE4: Notify AS5600 raw / deg / rpm / magnet\n");
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            gatt_svr_on_connect(event->connect.conn_handle);
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc == 0) {
                printf("\n手机已连接!\n");
                print_conn_desc(&desc);
            }
        } else {
            printf("连接失败, status=%d，重新广播\n", event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        gatt_svr_on_disconnect();
        printf("\n手机已断开, reason=%d\n", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        gatt_svr_on_subscribe(event->subscribe.attr_handle,
                              event->subscribe.conn_handle,
                              event->subscribe.cur_notify != 0);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc == 0) {
            printf("\n连接参数已更新:\n");
            print_conn_desc(&desc);
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        printf("MTU 已更新: conn_handle=%d, mtu=%d\n",
               event->mtu.conn_handle, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;

    default:
        return 0;
    }
}

static void on_sync(void)
{
    int rc;
    uint8_t addr[6];

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "获取地址类型失败, rc=%d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr, NULL);
    if (rc == 0) {
        print_mac("ESP32 蓝牙地址", addr);
    }

    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE 栈重置, reason=%d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void imu_log_task(void *arg)
{
    (void)arg;
    bmi088_vec3_t acc;
    bmi088_vec3_t gyr;

    int log_div = 0;

    while (true) {
        if (bmi088_read_accel(&acc) == ESP_OK && bmi088_read_gyro(&gyr) == ESP_OK) {
            gatt_svr_notify_imu(&acc, &gyr);
            if (++log_div >= 10) {
                log_div = 0;
                ESP_LOGI(IMU_TAG, "acc[g] %7.3f %7.3f %7.3f  gyro[dps] %8.2f %8.2f %8.2f",
                         acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
            }
        } else {
            ESP_LOGE(IMU_TAG, "read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void bldc_notify_task(void *arg)
{
    (void)arg;
    bldc_status_t st;

    while (true) {
        bldc_get_status(&st);
        gatt_svr_notify_bldc(&st);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void as5600_task(void *arg)
{
    (void)arg;
    as5600_sample_t s;
    as5600_sample_t last = {0};
    int log_div = 0;

    while (true) {
        if (as5600_read(&s) == ESP_OK) {
            last = s;
            gatt_svr_notify_as5600(&s);
            if (++log_div >= 10) {
                log_div = 0;
                ESP_LOGI(ENC_TAG, "raw=%4u  deg=%6.1f  rpm=%7.1f  mag=%s%s%s",
                         s.raw, (double)s.angle_deg, (double)s.rpm,
                         s.magnet_ok ? "OK" : "NO",
                         s.magnet_weak ? " weak" : "",
                         s.magnet_strong ? " strong" : "");
            }
        } else {
            gatt_svr_notify_as5600(&last);
            ESP_LOGE(ENC_TAG, "read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    int rc;

    printf("ESP32 BLE + SG90 舵机示例\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 舵机初始化，暂时不使用
    // ESP_ERROR_CHECK(servo_init());

    // IMU初始化 暂时不使用
    // ret = bmi088_init();
    // if (ret == ESP_OK) {
    //     BaseType_t ok = xTaskCreate(imu_log_task, "imu_log", 3072, NULL, 4, NULL);
    //     if (ok != pdPASS) {
    //         ESP_LOGE(IMU_TAG, "create imu_log task failed");
    //     }
    // } else {
    //     ESP_LOGE(IMU_TAG, "init failed: %s", esp_err_to_name(ret));
    // }

    ret = bldc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLDC 初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    if (xTaskCreate(bldc_notify_task, "bldc_n", 2048, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "create bldc notify task failed");
    }

    ret = as5600_init();
    if (ret == ESP_OK) {
        if (xTaskCreate(as5600_task, "as5600", 3072, NULL, 4, NULL) != pdPASS) {
            ESP_LOGE(ENC_TAG, "create as5600 task failed");
        }
    } else {
        ESP_LOGE(ENC_TAG, "init failed: %s", esp_err_to_name(ret));
    }

    // 初始化NimBLE（INFO 会每条 notify 打一遍 GATT procedure）
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE 初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    rc = gatt_svr_init();
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    assert(rc == 0);

    nimble_port_freertos_init(host_task);
}
