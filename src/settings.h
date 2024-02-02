#include <Arduino.h>
#include <ardupilotmega/mavlink.h>
#include <FreeRTOS.h>
#include "task.h"

#define target_system 100
#define target_component 1
#define this_component MAV_COMP_ID_GPS

#define heartbeat_ms 1000
#define rangefinder1_ms 50
#define recieve_ms 10
