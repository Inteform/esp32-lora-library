#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

uint8_t buf[255];

void task_rx(void *p)
{
    int x;
    for(;;) {
        lora_receive();    // put into receive mode
        while(lora_received()) {
            x = lora_receive_packet(buf, sizeof(buf));
            buf[x] = 0;
            int rssi = lora_packet_rssi();
            printf("Received: %s, RSSI: %d\n", buf, rssi);
            lora_receive();
        }
        vTaskDelay(1);
    }
}

void app_main()
{
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();

    printf("Lora Initialized: %d\n", lora_initialized());

    xTaskCreate(&task_rx, "task_rx", 2048, NULL, 5, NULL);
}