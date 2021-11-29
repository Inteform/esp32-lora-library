# esp32-lora-library
## Which is
** esp32-lora-library ** is a C component to be integrated with ESP32-IDF to send and receive data through a LoRa transceiver based on Semtech's SX127_ ICs.
The library is a fork of the **esp32-lora-library ** (https://github.com/Inteform/esp32-lora-library) library of Inteform for ESP32.

## How to install
Simply clone the repository and copy the `` `components/lora``` directory into your ESP-IDF project directory or into the ` `` components/ `` `path of your $IDF_PATH (it will be public to all your projects) .
You can then just `` `#include" lora.h "` `` and use its functions.
Using `` `make menuconfig``` there will be LoRa options to configure (like pin numbers)

`` `bash
git clone https://github.com/JN513/esp32-lora-library
cp -r esp32-lora-library/components/ path/to/my/esp32/project
cd /path/to/my/esp32/project
make menuconfig
I do
#etc
`` `

## Basic usage
A simple **sender** program...
```c
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
   lora_init();
   lora_set_frequency(915e6);
   lora_enable_crc();
   xTaskCreate(&task_tx, "task_tx", 2048, NULL, 5, NULL);
}

```
Meanwhile in the **receiver** program...
```c
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
```

More examples in examples:

## Connection with the RF module
By default, the pins used to control the RF transceiver are--

Pin | Signal
--- | ------
CS | IO15
RST | IO32
MISO | IO13 
MOSI | IO12
SCK | IO14

but you can reconfigure the pins using ```make menuconfig``` and changing the options in the "LoRa Options --->"
