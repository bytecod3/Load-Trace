#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "defines.h"
#include "config.h"
#include "states.h"
#include "esp_log.h"

/**
 * System logging parameters 
 */
static const char* log_tag = "SYSTEM";

/**
 * SYSTEM PARAMETERS STRUCT 
 */
typedef struct sys_params {
    uint8_t no_of_tasks;
    size_t free_heap; 
} sys_parameters_type_t;

sys_parameters_type_t system_parameters;
sys_parameters_type_t* system_params_ptr = &system_parameters;

/**
 * SYSTEM STATES 
 */
states_type_t sm_state = states_type_t::STATE_DEVICE_BOOT; // initial state

#if UNIT_TEST

    states_type_t states_array[7] = {
        STATE_DEVICE_BOOT,  /*!< device is booting */
        STATE_WIFI_CONNECTING, 
        STATE_WIFI_CONNECTED,
        STATE_WIFI_TIMEOUT,
        STATE_DEVICE_IDLE,  /*!< device is idle */
        STATE_DEVICE_FIRMWARE_TEST,  /*!< device is testing user firmware */
        STATE_DEVICE_FAULT  /*!< device is booting */
    };

#endif

/**
 * GLOBAL SYSTEM FLAGS
 */
bool wifi_connected = false;

/**
 * TIMER HANDLES 
 */
TimerHandle_t fsm_test_timer_handle;

/**
 * TASK HANDLES 
 */
xTaskHandle x_task_heap_monitor_task_handle;
xTaskHandle x_task_oled_control_handle;
xTaskHandle x_task_buzzer_control_handle;
xTaskHandle x_task_ntp_time_update_handle;
xTaskHandle x_task_finite_state_machine_handle;    

/**
 * QUEUE HANDLES 
 */
QueueHandle_t system_parameters_queue_handle;

/**
 * TASKS
 */

 /**
  * @brief This task monitors the global free heap memory from TCB 
  * todo: We will check which heap we are using (heap 4)
  * 
  * Get free task size and sent to system parameters queue 
  */
void x_task_heap_monitor(void* pvParameters ) {

    for(;;) {
        size_t free_heap = xPortGetFreeHeapSize();

        // send to queue
        if(xQueueSend(system_parameters_queue_handle, &free_heap, portMAX_DELAY) != pdTRUE) {
            // todo: log error 
            ESP_LOGI(log_tag, "Failed to send free_heap to queue");
        }

        vTaskDelay(pdMS_TO_TICKS(1));   // prevent task starvation 
    }
}

/**
 * @brief This task updates the OLED screen UI 
 * 
 */
void x_task_oled_control(void* pvParameters ) {

    size_t recvd_free_heap;

    for(;;) {
        // for now monitor this in serial, when we add hardware we will control OLED 
        if(xQueueReceive(system_parameters_queue_handle, &recvd_free_heap, portMAX_DELAY) != pdTRUE) {
            ESP_LOGI(log_tag, "could not receive from queue");
        } else {
            // print to monitor 
            //ESP_LOGI(log_tag, "free heap: %d\n", recvd_free_heap);
        }

    }
}

/**
 * @brief This task keeps control of NTP time if WIFI is connected/ configured
 */

void x_task_ntp_time_update(void* pvParameters) {
    for(;;) {

    }
}

/**
 * @brief This task controls the switching of the global state machine 
 * we will shadow the global system state 
 * 
 */
void x_task_finite_state_machine(void* pvParameters) {
    static const char* FSM_TAG = "FSM_TASK";
    for(;;) {
        switch (sm_state) {
            case states_type_t::STATE_DEVICE_BOOT:
                ESP_LOGI(FSM_TAG, "IN BOOT");
                break;

            case states_type_t::STATE_WIFI_CONNECTING:

                #if (USE_WIFI)
                    ESP_LOGI(FSM_TAG, "IN WIFI CONNECTING");
                #endif
            
                break;

            case states_type_t::STATE_WIFI_CONNECTED:
                ESP_LOGI(FSM_TAG, "IN WIFI CONNECTED");
                break;

            case states_type_t::STATE_WIFI_TIMEOUT:
                ESP_LOGI(FSM_TAG, "IN WIFI TIMEOUT");
                break;

            case states_type_t::STATE_DEVICE_FAULT:
                ESP_LOGI(FSM_TAG, "IN DEVICE FAULT");
                break;

            case states_type_t::STATE_DEVICE_FIRMWARE_TEST:
                ESP_LOGI(FSM_TAG, "IN FIRMWARE RESET");
                break;
        
            default:
                ESP_LOGI(FSM_TAG, "INVALID STATE ");
                break;
        }

    }
}

/**
 * @brief This function is the callback for testing FSM using software timer
 * 
 */
void fsm_timer_callback(TimerHandle_t xTimer) {
    states_type_t s = sm_state;

    static uint8_t i = 0;
    i++;

    if(i > NUM_STATES) {
        i = 0;
    } else {
        sm_state = states_array[i];
    }

    // debug states 
    ESP_LOGI("FSM", "SM_STATE: %s\n", states_to_str(sm_state));

}


void setup() {

    Serial.begin(SERIAL1_BAUDRATE);

    esp_log_level_set(log_tag, ESP_LOG_INFO);

    /**
     * Create software timers 
     */
    fsm_test_timer_handle =  xTimerCreate( "FMS _test_time",
                                        pdMS_TO_TICKS(SOFT_TIMER_PERIOD),
                                        pdTRUE,
                                        0,
                                        fsm_timer_callback);

    if(xTimerStart(fsm_test_timer_handle, 300) != pdPASS) {    // start timer
        ESP_LOGI(log_tag, "FSM timer failed to start");
    }  else {
        ESP_LOGI(log_tag, "FSM timer started OK");
    }

    /**
     * Create queues 
     * Always create queues before tasks 
     */
    // system_parameters_queue_handle = xQueueCreate(5, sizeof(sys_parameters_type_t));
    system_parameters_queue_handle = xQueueCreate(5, sizeof(size_t));
    if(system_parameters_queue_handle != NULL) {
        ESP_LOGI(log_tag, "system parameters queue created Okay");
    } else {
        ESP_LOGE(log_tag, "system parameters queue failed to create"); // todo: add color 
    }

    /**
     * Task creation
     */

    uint32_t esp_core_id = xPortGetCoreID(); // gets the ESP32 core ID

    BaseType_t heap_monitor_status = xTaskCreatePinnedToCore(       // HEAP MONITOR TASK
        x_task_heap_monitor,   // task function
        "x_task_heap_monitor", // task name to debug
        TASK_STACK_DEPTH,           // task stack depth in CPU words
        NULL,                       // parameters
        tskIDLE_PRIORITY + 1,       // task priority
        &x_task_heap_monitor_task_handle,      // task handle
        esp_core_id                 // which core to run on 

    );

    if(heap_monitor_status == pdPASS) { // todo: maybe log this to system log file
        ESP_LOGI(log_tag, "x_task_heap_monitor_task_handle created OK");
    } else {
        ESP_LOGE(log_tag, "x_task_monitor_task failed to create");
    }

    BaseType_t ntp_time_update_status = xTaskCreatePinnedToCore(       // NTP TIME TASK
        x_task_ntp_time_update,   // task function
        "x_task_heap_monitor_task", // task name to debug
        TASK_STACK_DEPTH,           // task stack depth in CPU words
        NULL,                       // parameters
        tskIDLE_PRIORITY + 1,       // task priority
        &x_task_heap_monitor_task_handle,      // task handle
        esp_core_id                 // which core to run on 

    );

    if(ntp_time_update_status == pdPASS) { // todo: maybe log this to system log file
        ESP_LOGI(log_tag, "ntp_time_update_status_task created OK");
    } else {
        ESP_LOGE(log_tag, "ntp_time_update_status_task failed to create");
    }

    BaseType_t oled_control_task_status = xTaskCreatePinnedToCore(       // OLED CONTROL TASK
            x_task_oled_control,// task function
        "x_task_oled_control", // task name to debug
        TASK_STACK_DEPTH,           // task stack depth in CPU words
        NULL,                       // parameters
        tskIDLE_PRIORITY + 1,       // task priority
        &x_task_oled_control_handle,      // task handle
        esp_core_id                 // which core to run on 

    );

    if(oled_control_task_status == pdPASS) { 
        ESP_LOGI(log_tag, "oled_control_task created OK");
    } else {
        ESP_LOGE(log_tag, "oled_control_task failed to create");
    }

    BaseType_t finite_state_machine_status = xTaskCreatePinnedToCore(       // FSM CONTROL TASK
            x_task_finite_state_machine,// task function
        "x_task_finite_state_machine", // task name to debug
        TASK_STACK_DEPTH,           // task stack depth in CPU words
        NULL,                       // parameters
        tskIDLE_PRIORITY + 1,       // task priority
        &x_task_finite_state_machine_handle,      // task handle
        esp_core_id                 // which core to run on 

    );

    if(finite_state_machine_status == pdPASS) { 
        ESP_LOGI(log_tag, "x_task_finite_state_machine created OK");
    } else {
        ESP_LOGE(log_tag, "x_task_finite_state_machine failed to create");
    }

}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10));
}