/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart1.h"
#include "uart2.h"

void uart1_rx_callback(char* data, size_t size)
{
    ESP_LOGI(__func__, "Received: %s", data);
}

void uart2_rx_callback(char* data, size_t size)
{
    ESP_LOGI(__func__, "Received: %s", data);
}

void app_main(void)
{
    uart1_init();
    uart2_init();

    uart1_data_cb = uart1_rx_callback;
    uart2_data_cb = uart2_rx_callback;

    int counter = 0;
    char buffer[64];
    while (1)
    {
        for (int i = 0; i < 10; i++)
        {
            snprintf(buffer, sizeof(buffer), "Hello World %d", counter);

            uart_write_bytes(UART_NUM_1, buffer, strlen(buffer));
            
            uart_write_bytes(UART_NUM_2, buffer, strlen(buffer));

            counter++;
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }

        for (int i = 0; i < 10; i++)
        {
            snprintf(buffer, sizeof(buffer), "Hello World %d", counter);

            uart_write_bytes(UART_NUM_1, buffer, strlen(buffer));
            
            vTaskDelay(500/portTICK_PERIOD_MS);

            uart_write_bytes(UART_NUM_2, buffer, strlen(buffer));

            counter++;
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
    }
}
