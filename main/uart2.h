#ifndef uart2_H
#define uart2_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"

QueueHandle_t uart2_queue;
size_t RX2_BUFFER_SIZE = 1024;
size_t uart2_pattern_size = 0;

void (*uart2_data_cb)(char*, size_t) = NULL;
void (*uart2_fifo_overflow_cb)() = NULL;
void (*uart2_buffer_full_cb)() = NULL;
void (*uart2_break_cb)() = NULL;
void (*uart2_parity_err_cb)() = NULL;
void (*uart2_frame_err_cb)() = NULL;
void (*uart2_pattern_cb)(char*, size_t, size_t) = NULL;

void uart2_init();
void uart2_event_task(void *pvParameters);

inline void uart2_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RX2_BUFFER_SIZE);
    size_t pos;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart2_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RX2_BUFFER_SIZE);
            //ESP_LOGI(__func__, "uart[%d] event:", UART_NUM_2);
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                //ESP_LOGI(__func__, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
                if (uart2_data_cb)
                {
                    uart2_data_cb((char*)dtmp, event.size);
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                //ESP_LOGI(__func__, "hw fifo overflow");
                if (uart2_fifo_overflow_cb)
                {
                    uart2_fifo_overflow_cb();
                }

                uart_flush_input(UART_NUM_2);
                xQueueReset(uart2_queue);

                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                //ESP_LOGI(__func__, "ring buffer full");
                if (uart2_buffer_full_cb)
                {
                    uart2_buffer_full_cb();
                }

                uart_flush_input(UART_NUM_2);
                xQueueReset(uart2_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                //ESP_LOGI(__func__, "uart rx break");
                if (uart2_break_cb)
                {
                    uart2_break_cb();
                }
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                //ESP_LOGI(__func__, "uart parity error");
                if (uart2_parity_err_cb)
                {
                    uart2_parity_err_cb();
                }
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                //ESP_LOGI(__func__, "uart frame error");
                if (uart2_frame_err_cb)
                {
                    uart2_frame_err_cb();
                }
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(UART_NUM_2, &buffered_size);
                pos = uart_pattern_pop_pos(UART_NUM_2);
                //ESP_LOGI(__func__, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    //Pattern queue overflow
                    uart_flush_input(UART_NUM_2);
                } else {
                    uart_read_bytes(UART_NUM_2, dtmp, pos+uart2_pattern_size, 100 / portTICK_PERIOD_MS);
                    if (uart2_pattern_cb)
                    {
                        uart2_pattern_cb((char*)dtmp, pos+uart2_pattern_size, pos);
                    }
                }
                break;
            //Others
            default:
                //ESP_LOGI(__func__, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

inline void uart2_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_2, RX2_BUFFER_SIZE * 2, RX2_BUFFER_SIZE * 2, 20, &uart2_queue, 0);
    uart_param_config(UART_NUM_2, &uart_config);

    //Set UART pins (using uart2 default pins ie no changes.)
    uart_set_pin(UART_NUM_2, GPIO_NUM_6, GPIO_NUM_7, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    //uart_enable_pattern_det_baud_intr(UART_NUM_2, '!', 1, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    //uart_pattern_queue_reset(UART_NUM_2, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart2_event_task, "uart2_event_task", 2048, NULL, 12, NULL);
}

#endif
