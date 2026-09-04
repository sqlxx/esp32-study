/*
 * ESP32 BLE 外设：广播连接后打印连接信息，并提供可读写 GATT 特征
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "servo.h"

static const char *TAG = "BLE";
static const char *DEVICE_NAME = "ESP32-Hello";

static uint8_t own_addr_type;

int gatt_svr_init(void);
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
    printf("可用 nRF Connect，或打开 web/index.html（Android Chrome）\n");
    printf("Characteristic 0xFFE1: Write 角度 / scan / stop / spd:1~10\n");
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
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
        printf("\n手机已断开, reason=%d\n", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        start_advertising();
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

    ESP_ERROR_CHECK(servo_init());

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
