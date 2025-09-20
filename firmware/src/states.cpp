
#include "states.h"

 /**
  * This function will convert state to string
  */
 const char* states_to_str(states_type_t state) {

    switch (state) {
        case STATE_DEVICE_BOOT:
            return "STATE_DEVICE_BOOT";
            break;

            case STATE_WIFI_CONNECTING:
            return "STATE_WIFI_CONNECTING";
            break;

            case STATE_WIFI_TIMEOUT:
            return "STATE_WIFI_TIMEOUT";
            break;

            case STATE_DEVICE_IDLE:
            return "STATE_DEVICE_IDLE";
            break;

            case STATE_DEVICE_FIRMWARE_TEST:
            return "STATE_DEVICE_FIRMWARE_TEST";
            break;

            case STATE_DEVICE_FAULT:
            return "STATE_DEVICE_FAULT";
            break;

        default:
            return "INVALID STATE";
            break; // no reached
    }

 }