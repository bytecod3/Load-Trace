#include <Arduino.h>

#include <AsyncTCP.h> /** !include this before WIFI.h */
#include <WiFi.h>

#include <freertos/FreeRTOS.h>
#include "defines.h"
#include "config.h"
#include "states.h"
#include "esp_log.h"
#include "files.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

/* File system variables */
#define FORMAT_LITTLEFS_IF_FAILED true

/**
 * Create a basic web server to allow me to feed my WIFI
 * and password to the ESP
 */

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
 * Local functions
 */

/**
 * @brief initialise file system
 */
 static const char* files_debug_tag = "File Operations";
void LittleFS_mount() {
    ESP_LOGI(files_debug_tag, "Mounting Little FS");
    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        ESP_LOGE(files_debug_tag, "LittleFS mount failed. Formatting...");
        if (LittleFS.format()) {
            ESP_LOGI(files_debug_tag, "Little FS successfully formatted");
        } else {
            ESP_LOGE(files_debug_tag, "Error formatting Little FS");
        }

        if (!LittleFS.begin()) {
            ESP_LOGE(files_debug_tag, "Little FS could not be mounted after formatting");
        } else {
            ESP_LOGI(files_debug_tag, "Little FS successfully mounted after formatting");
        }
    } else {
        ESP_LOGI(files_debug_tag, "Little FS mounted successfully");
    }
}

/**
 * @brief This function will initialize files and check if they exist
 * or not
 * create JSON templates too
 */
 void file_operations() {
    File file = LittleFS.open(wificonfig_folder_path);
    if(!file) {
        ESP_LOGI(files_debug_tag, "Wifi config folder does not exist. Creating...");
        createDir(LittleFS, wificonfig_folder_path);
    } else {
        // directory exists, check what is in the directory
        listDir(LittleFS, wificonfig_folder_path, 1);

        // check if files exist
        File a = LittleFS.open(wifi_config_bit_filepath);
        if(!a) {
            ESP_LOGI(files_debug_tag, "Wifi config bit file does not exist. Creating...");

            //create a JSON object to hold the bit
            JsonDocument wifi_config_doc;
            char out_str[10];
            wifi_config_doc["wifi_set"] = 0; // initially wifi is not set till we provision it via server

            serializeJson(wifi_config_doc, out_str);
            writeFile(LittleFS, wifi_config_bit_filepath, out_str);
        } else {
            // if file exists, extract the contents and check if WIFI was
            // configured before
            ESP_LOGI(files_debug_tag, "Wifi config bit file exists. Checking...");

            readFile(LittleFS, wifi_config_bit_filepath);
            JsonDocument d;
            deserializeJson(d, file_data_buffer);

            uint8_t wifi_set_status = d["wifi_set"];

            ESP_LOGI(files_debug_tag, "WIFI set bit: %d\n", wifi_set_status);

            if(wifi_set_status == 0) {
                ESP_LOGI(files_debug_tag, "WIFI not set. Requesting...");
                sm_state = states_type_t::STATE_WIFI_REQUEST;
            } else {
                ESP_LOGI(files_debug_tag, "WIFI set. Connecting...");
                sm_state = states_type_t::STATE_WIFI_CONNECT;
            }

        }
    }

 }


 /**
  * @brief This function create a web server toa llow for device parameters provision
  *
  */
 AsyncWebServer server(80);
const char* config_wifi_ssid = "wifi-ssid";
const char* config_wifi_password = "wifi-password";

/**
* Device Index Page
*/
const char index_html[] PROGMEM = R"rawliteral(

<!DOCTYPE HTML>
<html>
<head>
  <title>Load Trace WIFI config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head>
    <body>
    <h3> Load Trace WIFI config </h3>
    <form action="/get">

        <p>WiFi Config</p>
        WiFi SSID: <input type="text" name="wifi_ssid"> <br><br>

        WiFi Password: <input type="text" name="wifi_password"> <br><br>

    <input type="submit" value="SAVE">
    </form><br>

    </form><br>
</body>
</html>

)rawliteral";

const char success_response[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Load Trace WIFI config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head>
    <body>
     <p>Parameters Saved!</p> <br><br>
     <br><a href="/network">Return to Home Page</a>
</body>
</html>
)rawliteral";

void web_server_not_found(AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Server not found");
}

uint8_t wifi_configured_bit = 0;
void web_server_init() {

     // set handlers
     // todo: use const char variables for URLs
     server.on("/network", HTTP_GET, [](AsyncWebServerRequest *request){
         request->send(200, "text/html", index_html);
     });

     server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
         String inputMessage;
         String inputParam;

         String config_wifi_name;
         String config_wifi_psd;

         // Wi-Fi SSID (ipaddress/get?wifi-ssid=12345
         if(request->hasParam("wifi_ssid")) {

             config_wifi_name = request->getParam("wifi_ssid")->value();

             if(config_wifi_name != "") { /* SSID must exist */
                 inputParam = config_wifi_ssid;
             }
         }

         if(request->hasParam("wifi_password")) { // Wi-Fi password
             config_wifi_psd = request->getParam("wifi_password")->value();

             if (config_wifi_psd != "") {
                 inputParam = config_wifi_psd;
             }
         }

         // here, save the credentials in JSON structure in FILE

         // create JSON structure to save to memory
         //createJSON(config_wifi_name, config_wifi_psd, gsm_apn, gsm_password);

         JsonDocument wifi_parameters_doc;
         char wifi_parameters_string[100]; // todo: remove magic number

         wifi_parameters_doc["ssid"] = config_wifi_name;
         wifi_parameters_doc["password"] = config_wifi_psd;

         serializeJson(wifi_parameters_doc, wifi_parameters_string);

         // update wifi config file
         File f = LittleFS.open(wifi_config_filepath);
         if(f) {
             // maybe we should update the exisiting wifi data.
             // or in case of multiple SSIDs , use an array of SSIDS and passwords
             // then loop through them to try and connect to each
             writeFile(LittleFS, wifi_config_filepath, wifi_parameters_string);
         } else {
             ESP_LOGE(files_debug_tag, "WIFI config file failed to open");
         }

         /* update the device configured status, after successful device configuration */
         wifi_configured_bit = 1;

         JsonDocument doc;
         readFile(LittleFS, wifi_config_bit_filepath);
         deserializeJson(doc, file_data_buffer);
         doc["wifi_set"] = wifi_configured_bit;

         char status_buffer_string[128];
         serializeJson(doc, status_buffer_string);
         writeFile(LittleFS, wifi_config_bit_filepath, status_buffer_string);

         delay(300); /* very necessary. ;) */

         request->send(200, "text/html", success_response);
     });

     // begin server
     server.onNotFound(web_server_not_found);
     server.begin();
     ESP_LOGI(files_debug_tag, "Provisioning server started, waiting for config parameters...");
 }

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
    static long last_request_time = 0;
    for(;;) {
        switch (sm_state) {
            case states_type_t::STATE_DEVICE_BOOT:
                ESP_LOGI(FSM_TAG, "IN BOOT");
                break;

            case states_type_t::STATE_WIFI_REQUEST:
                web_server_init();
                last_request_time = millis(); // use get tick count
                sm_state = states_type_t::WIFI_WAIT_CREDENTIALS;

                break;

            case states_type_t::WIFI_WAIT_CREDENTIALS:

                break;

            case states_type_t::STATE_WIFI_CONNECT:
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

    if (i > NUM_STATES) {
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
     * File operations init
     */
    LittleFS_mount();
    file_operations();

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