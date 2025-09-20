/**
 * @file defines.h
 * @brief declare some system wide stuff
 * @author Edwin 
 */

#pragma once 

#include <Arduino.h>

#define SERIAL1_BAUDRATE    (115200)
#define DEBUG_EN            (1)
#define TASK_STACK_DEPTH    (2048)

#define QUEUE_SEND_TIME_WAIT (500) //todo: increase 

/** UNIT TESTING VARIABLES */
#define SOFT_TIMER_PERIOD (1000)




/* DEBUG shortcuts */
#if DEBUG_EN

#define DEBUG(x)    Serial.print(x)
#define DEBUGLN(x)  Serial.println(x);

#endif
