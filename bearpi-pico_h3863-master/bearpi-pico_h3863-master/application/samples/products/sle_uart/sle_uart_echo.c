/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: UART Echo + SLE Auto Connect.
 *              Device acts as both SLE Server and Client simultaneously.
 *              Auto-scans, auto-connects, bidirectional data forwarding.
 *
 * History: 
 * 2024-07-04, Create file. 
 */
#include "common_def.h"
#include "soc_osal.h"
#include "securec.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"
#include "watchdog.h"
#include "gpio.h"
#include "sle_low_latency.h"
#include "sle_errcode.h"
#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_ssap_server.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"

#define UART_BAUDRATE                       115200
#define UART_TRANSFER_SIZE                  512
#define UART_TASK_STACK_SIZE                0x1800
#define UART_TASK_PRIO                      28
#define SLE_ADV_HANDLE_DEFAULT              1
#define SLE_SERVER_MSG_QUEUE_LEN            5
#define SLE_SERVER_MSG_QUEUE_MAX_SIZE       32
#define SLE_SEEK_INTERVAL                   100
#define SLE_SEEK_WINDOW                     100
#define SLE_MTU_SIZE                        520
#define SLE_UART_SERVER_NAME                "sle_uart_server"
#define BLUE_LED_PIN                        2

static uint8_t g_app_uart_rx_buff[UART_TRANSFER_SIZE] = { 0 };
static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = UART_TRANSFER_SIZE
};

static uint8_t g_rx_data[UART_TRANSFER_SIZE] = { 0 };
static volatile uint16_t g_rx_length = 0;
static volatile uint8_t g_rx_flag = 0;

// SLE Client state
static uint16_t g_sle_conn_id = 0;
static volatile bool g_sle_connected = false;
static ssapc_write_param_t g_sle_send_param = { 0 };
static sle_addr_t g_sle_remote_addr = { 0 };

// SLE Server message queue
static unsigned long g_sle_msgqueue_id;

static void sle_create_msgqueue(void)
{
    if (osal_msg_queue_create("sle_echo_mq", SLE_SERVER_MSG_QUEUE_LEN,
        (unsigned long *)&g_sle_msgqueue_id, 0, SLE_SERVER_MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        osal_printk("SLE msgqueue create failed\r\n");
    }
}

static void sle_write_msgqueue(uint8_t *buffer_addr, uint16_t buffer_size)
{
    osal_msg_queue_write_copy(g_sle_msgqueue_id, (void *)buffer_addr, (uint32_t)buffer_size, 0);
}

/* ==================== SLE Server Callbacks ==================== */

static void ssaps_read_request_cbk(uint8_t server_id, uint16_t conn_id,
    ssaps_req_read_cb_t *read_cb_para, errcode_t status)
{
    unused(server_id);
    unused(conn_id);
    unused(read_cb_para);
    unused(status);
}

static void ssaps_write_request_cbk(uint8_t server_id, uint16_t conn_id,
    ssaps_req_write_cb_t *write_cb_para, errcode_t status)
{
    unused(server_id);
    unused(conn_id);
    unused(status);
    if ((write_cb_para->length > 0) && write_cb_para->value) {
        uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)write_cb_para->value, write_cb_para->length, 0);
    }
}

/* ==================== SLE Client Callbacks ==================== */

static void sle_client_notification_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    if (data && data->data && data->data_len > 0) {
        uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)(data->data), data->data_len, 0);
    }
}

static void sle_client_indication_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_handle_value_t *data, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    // indication 回调不写 UART，避免与 notification 重复
}

static void sle_seek_result_cb(sle_seek_result_info_t *seek_result)
{
    if (seek_result == NULL) {
        return;
    }
    osal_printk("SLE found device: %s\r\n", seek_result->data);
    if (strstr((const char *)seek_result->data, SLE_UART_SERVER_NAME) != NULL) {
        osal_printk("Target found, stop scan and connect\r\n");
        memcpy_s(&g_sle_remote_addr, sizeof(sle_addr_t), &seek_result->addr, sizeof(sle_addr_t));
        sle_stop_seek();
    }
}

static void sle_seek_disable_cb(errcode_t status)
{
    if (status != 0) {
        return;
    }
    sle_remove_paired_remote_device(&g_sle_remote_addr);
    sle_connect_remote_device(&g_sle_remote_addr);
}

static void sle_connect_state_cb(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    unused(addr);
    unused(disc_reason);
    g_sle_conn_id = conn_id;
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        osal_printk("SLE client connected, conn_id: 0x%02x\r\n", conn_id);
        g_sle_connected = true;
        if (pair_state == SLE_PAIR_NONE) {
            sle_pair_remote_device(&g_sle_remote_addr);
        }
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        osal_printk("SLE client disconnected, rescan...\r\n");
        g_sle_conn_id = 0;
        g_sle_connected = false;
        uapi_gpio_set_val(BLUE_LED_PIN, GPIO_LEVEL_LOW);
        sle_remove_paired_remote_device(&g_sle_remote_addr);
        sle_start_seek();
    }
}

static void sle_pair_complete_cb(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    unused(addr);
    osal_printk("SLE pair complete, conn_id: 0x%02x, status: %d\r\n", conn_id, status);
    if (status == 0) {
        ssap_exchange_info_t info = {0};
        info.mtu_size = SLE_MTU_SIZE;
        info.version = 1;
        ssapc_exchange_info_req(0, g_sle_conn_id, &info);
    }
}

static void sle_exchange_info_cb(uint8_t client_id, uint16_t conn_id,
    ssap_exchange_info_t *param, errcode_t status)
{
    unused(client_id);
    unused(status);
    osal_printk("SLE exchange info, mtu: %d\r\n", param->mtu_size);
    ssapc_find_structure_param_t find_param = {0};
    find_param.type = SSAP_FIND_TYPE_PROPERTY;
    find_param.start_hdl = 1;
    find_param.end_hdl = 0xFFFF;
    ssapc_find_structure(0, conn_id, &find_param);
}

static void sle_find_structure_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_find_service_result_t *service, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("SLE find service, handle: 0x%02x-0x%02x\r\n", service->start_hdl, service->end_hdl);
}

static void sle_find_property_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_find_property_result_t *property, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("SLE find property, handle: %d\r\n", property->handle);
    g_sle_send_param.handle = property->handle;
    g_sle_send_param.type = SSAP_PROPERTY_TYPE_VALUE;
}

static void sle_write_cfm_cb(uint8_t client_id, uint16_t conn_id,
    ssapc_write_result_t *write_result, errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(write_result);
    unused(status);
}

/* ==================== UART ==================== */

static void uart_rx_callback(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (buffer == NULL || length == 0) {
        return;
    }
    if (memcpy_s(g_rx_data, UART_TRANSFER_SIZE, buffer, length) == EOK) {
        g_rx_length = length;
        g_rx_flag = 1;
    }
    // Send via SLE Server (if client connected to us)
    if (sle_uart_client_is_connected()) {
        sle_uart_server_send_report_by_handle((uint8_t *)buffer, length);
    }
    // Send via SLE Client (if we connected to a server)
    else if (g_sle_conn_id != 0) {
        g_sle_send_param.data_len = length;
        g_sle_send_param.data = (uint8_t *)buffer;
        ssapc_write_req(0, g_sle_conn_id, &g_sle_send_param);
    }
}

static void uart_init_pin(void)
{
    uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
    uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);
}

static void uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };
    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART_TXD_PIN,
        .rx_pin = CONFIG_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(CONFIG_SLE_UART_BUS);
    uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
}

/* ==================== Main Task ==================== */

static void *sle_echo_task(const char *arg)
{
    unused(arg);
    uint8_t rx_buf[SLE_SERVER_MSG_QUEUE_MAX_SIZE] = {0};
    uint32_t rx_length = SLE_SERVER_MSG_QUEUE_MAX_SIZE;
    uint8_t sle_dis[] = "sle_dis_connect";

    osal_printk("SLE echo task started\r\n");

    // 1. Init SLE Server (this calls enable_sle() internally)
    osal_printk("Init SLE server...\r\n");
    sle_create_msgqueue();
    sle_uart_server_register_msg(sle_write_msgqueue);
    sle_uart_server_init(ssaps_read_request_cbk, ssaps_write_request_cbk);
    osal_printk("SLE server init done\r\n");

    // 2. Register SLE Client callbacks manually (SLE already enabled by server)
    osal_printk("Init SLE client...\r\n");

    // Register seek callbacks
    sle_announce_seek_callbacks_t seek_cbk = {0};
    seek_cbk.seek_enable_cb = NULL;
    seek_cbk.seek_result_cb = sle_seek_result_cb;
    seek_cbk.seek_disable_cb = sle_seek_disable_cb;
    sle_announce_seek_register_callbacks(&seek_cbk);

    // Register connection callbacks
    sle_connection_callbacks_t connect_cbk = {0};
    connect_cbk.connect_state_changed_cb = sle_connect_state_cb;
    connect_cbk.pair_complete_cb = sle_pair_complete_cb;
    sle_connection_register_callbacks(&connect_cbk);

    // Register SSAP client callbacks
    ssapc_callbacks_t ssapc_cbk = {0};
    ssapc_cbk.exchange_info_cb = sle_exchange_info_cb;
    ssapc_cbk.find_structure_cb = sle_find_structure_cb;
    ssapc_cbk.ssapc_find_property_cbk = sle_find_property_cb;
    ssapc_cbk.write_cfm_cb = sle_write_cfm_cb;
    ssapc_cbk.notification_cb = sle_client_notification_cb;
    ssapc_cbk.indication_cb = sle_client_indication_cb;
    ssapc_register_callbacks(&ssapc_cbk);

    // Start scanning (SLE is already enabled)
    sle_seek_param_t seek_param = { 0 };
    seek_param.own_addr_type = 0;
    seek_param.filter_duplicates = 0;
    seek_param.seek_filter_policy = 0;
    seek_param.seek_phys = 1;
    seek_param.seek_type[0] = 1;
    seek_param.seek_interval[0] = SLE_SEEK_INTERVAL;
    seek_param.seek_window[0] = SLE_SEEK_WINDOW;
    sle_set_seek_param(&seek_param);
    sle_start_seek();
    osal_printk("SLE client scan started\r\n");

    // 3. Init UART
    osal_printk("Init UART...\r\n");
    uart_init_pin();
    uart_init_config();

    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
        UART_RX_CONDITION_MASK_IDLE, 0, uart_rx_callback);
    if (ret != ERRCODE_SUCC) {
        osal_printk("UART callback register failed: %x\r\n", ret);
        return NULL;
    }

    osal_printk("SLE auto-connect echo started, UART%d @ %d\r\n", CONFIG_SLE_UART_BUS, UART_BAUDRATE);

    // Init blue LED
    uapi_pin_set_mode(BLUE_LED_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(BLUE_LED_PIN, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(BLUE_LED_PIN, GPIO_LEVEL_LOW);

    while (1) {
        uapi_watchdog_kick();

        if (g_rx_flag) {
            g_rx_flag = 0;
            // uapi_uart_write(CONFIG_SLE_UART_BUS, g_rx_data, g_rx_length, 0);
        }

        rx_length = SLE_SERVER_MSG_QUEUE_MAX_SIZE;
        (void)memset_s(rx_buf, rx_length, 0, rx_length);
        (void)osal_msg_queue_read_copy(g_sle_msgqueue_id, (void *)rx_buf, &rx_length, 100);
        if (strncmp((const char *)rx_buf, (const char *)sle_dis, sizeof(sle_dis)) == 0) {
            sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
            sle_start_seek();
        }

        // LED control
        if (g_sle_connected || sle_uart_client_is_connected()) {
            // Connected: solid ON
            uapi_gpio_set_val(BLUE_LED_PIN, GPIO_LEVEL_HIGH);
            osal_msleep(10);
        } else {
            // Not connected: blink at 0.5s period
            uapi_gpio_toggle(BLUE_LED_PIN);
            osal_msleep(250);
        }
    }
    return NULL;
}

static void sle_echo_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)sle_echo_task, 0,
        "SleEchoTask", UART_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, UART_TASK_PRIO);
    }
    osal_kthread_unlock();
}

app_run(sle_echo_entry);
