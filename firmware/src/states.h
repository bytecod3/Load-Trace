/**
 * @file states.h
 * @brief This file declares the device states
 * @author Edwin
 * 
 */
#pragma once

 #define NUM_STATES 7

 typedef enum state {
    STATE_DEVICE_BOOT,  /*!< device is booting */
    STATE_WIFI_REQUEST,
     WIFI_WAIT_CREDENTIALS,
    STATE_WIFI_CONNECT,
    STATE_WIFI_CONNECTING, 
    STATE_WIFI_CONNECTED,
    STATE_WIFI_TIMEOUT,
    STATE_DEVICE_IDLE,  /*!< device is idle */
    STATE_DEVICE_FIRMWARE_TEST,  /*!< device is testing user firmware */
    STATE_DEVICE_FAULT  /*!< device is booting */

 } states_type_t;

 /**
  * This function will convert state to string
  */
 const char* states_to_str(states_type_t);