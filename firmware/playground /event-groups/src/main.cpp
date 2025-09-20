#include <Arduino.h>
#include "freertos/FreeRTOS.h"

QueueHandle_t data_queue_handle;

void producer_task(void* pvParameters) {
    uint8_t x = 0;

    for(;;) {

        // create dummy data 
        x++;

        // send to queue
        xQueueSend(data_queue_handle, &x, 500);

        Serial.printf("Produced: %d\n", x);

        // slow down 
        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }
}

void consumer1_task(void* pvParameters) {
    uint8_t r;

    for(;;) {
        xQueueReceive(data_queue_handle, &r, 500);

        Serial.printf("Consumer 1 received %d\n", r);

        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void consumer2_task(void* pvParameters) {

    uint8_t t;

    for(;;) {
        xQueueReceive(data_queue_handle, &t, 500);
        Serial.printf("Consumer 2 received %d\n", t);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void consumer3_task(void* pvParameters) {

    uint8_t u;

    for(;;) {

        xQueueReceive(data_queue_handle, &u, 500);
        Serial.printf("Consumer 3 received %d\n", u);

        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    data_queue_handle = xQueueCreate(5, sizeof(uint8_t));
    if(data_queue_handle != NULL) {
        Serial.println("Queue created OK");
    }

    xTaskCreatePinnedToCore(
        consumer1_task,
        "consumer1_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        consumer2_task,
        "consumer2_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        consumer3_task,
        "consumer3_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        producer_task,
        "producer_task",
        2048,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL,
        1
    );

}

void loop() {

}