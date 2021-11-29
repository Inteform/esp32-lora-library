#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

void task_tx(void *p)
{
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        lora_send_packet((uint8_t*)"Hello", 5);
        printf("packet sent...\n");
    }
}

void app_main()
{
    printf("Starting...\n");
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();
    xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
}