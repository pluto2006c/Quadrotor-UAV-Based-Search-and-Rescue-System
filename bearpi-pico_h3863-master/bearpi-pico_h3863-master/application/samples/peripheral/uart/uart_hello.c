/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: UART Echo Sample. Receives data via UART0 and echoes back.
 *
 * History: 
 * 2024-07-04, Create file. 
 */
#include "pinctrl.h"
#include "soc_osal.h"
#include "uart.h"
#include "watchdog.h"
#include "osal_debug.h"
#include "cmsis_os2.h"
#include "app_init.h"
#include "string.h"

#define UART_BAUDRATE                      115200
#define UART_DATA_BITS                     UART_DATA_BIT_8
#define UART_STOP_BITS                     UART_STOP_BIT_1
#define UART_PARITY_BIT                    UART_PARITY_NONE
#define UART_TRANSFER_SIZE                 512
#define UART_BUS_ID                        UART_BUS_0
#define UART_TX_PIN                        47  // S_AGPIO15
#define UART_RX_PIN                        46  // S_AGPIO14

#define UART_TASK_STACK_SIZE               0x1000
#define UART_TASK_PRIO                     (osPriority_t)(17)

static volatile uint8_t g_uart_rx_flag = 0;
static uint8_t g_uart_rx_data[UART_TRANSFER_SIZE] = { 0 };
static uint16_t g_uart_rx_length = 0;

static uint8_t g_app_uart_rx_buff[UART_TRANSFER_SIZE] = { 0 };
static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = UART_TRANSFER_SIZE
};

static void uart_rx_callback(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (buffer == NULL || length == 0) {
        return;
    }

    if (memcpy_s(g_uart_rx_data, UART_TRANSFER_SIZE, buffer, length) == EOK) {
        g_uart_rx_length = length;
        g_uart_rx_flag = 1;
    }
}

static void app_uart_init_pin(void)
{
    uapi_pin_set_mode(UART_TX_PIN, PIN_MODE_1);
    uapi_pin_set_mode(UART_RX_PIN, PIN_MODE_1);
}

static void app_uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BITS,
        .stop_bits = UART_STOP_BITS,
        .parity = UART_PARITY_BIT
    };
    uart_pin_config_t pin_config = {
        .tx_pin = UART_TX_PIN,
        .rx_pin = UART_RX_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(UART_BUS_ID);
    uapi_uart_init(UART_BUS_ID, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
}

static void *uart_echo_task(const char *arg)
{
    unused(arg);
    app_uart_init_pin();
    app_uart_init_config();

    if (uapi_uart_register_rx_callback(UART_BUS_ID, UART_RX_CONDITION_FULL_OR_IDLE,
                                       1, uart_rx_callback) == ERRCODE_SUCC) {
        osal_printk("UART%d rx callback registered\r\n", UART_BUS_ID);
    }

    while (1) {
        uapi_watchdog_kick();

        if (g_uart_rx_flag) {
            g_uart_rx_flag = 0;
            uapi_uart_write(UART_BUS_ID, g_uart_rx_data, g_uart_rx_length, 0);
        }

        osal_msleep(10);
    }
    return NULL;
}

static void uart_echo_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)uart_echo_task, 0, "UartEchoTask", UART_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, UART_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

app_run(uart_echo_entry);
