#include <Arduino.h>
#include <ardupilotmega/mavlink.h>
#include <FreeRTOS.h>
#include "task.h"
#include <EEPROM.h>
#include <semphr.h>

#define SERIAL_MAVLINK Serial1
#define SERIAL_SENSOR_1 Serial2
#define SERIAL_SENSOR_2 Serial3
#define SERIAL_MAVFWD Serial4

#define TARGET_SYSTEM 100
#define TARGET_COMPONENT 1
#define THIS_COMPONENT MAV_COMP_ID_GPS

#define heartbeat_ms 1000
#define rangefinder1_ms 50
#define recieve_ms 10

#define PARAMETER_COUNT 10

struct Param
{
    const char *name;
    uint16_t index;
    uint8_t type;
    float value;
};


struct MavNode
{
    SemaphoreHandle_t parameterSemaphore = NULL;
    Param mavnodeParam[PARAMETER_COUNT] = {
        {"PARAM1", 1, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM2", 2, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM3", 3, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM4", 4, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM5", 5, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM6", 6, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM7", 7, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM8", 8, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM9", 9, MAV_PARAM_TYPE_UINT16, 0},
        {"PARAM10", 10, MAV_PARAM_TYPE_UINT16, 0},
    };

};

void gps2raw_request(MavNode *mavnode);     // commands
void mav_request_data(MavNode *mavnode);    // commands
void rangefinder1_update(MavNode *mavnode); // sensors
void rangefinder2_update(MavNode *mavnode); // sensors
void heartbeat_update(MavNode *mavnode);    // commands
void param_init(MavNode *mavnode);      // parameters

// commands.cpp
void reboot(MavNode *mavnode);
void gcs_status(const char gcstext[]);

// parameters.cpp
void send_parameter(MavNode *mavnode, Param parameter);
void send_all_parameters(MavNode *mavnode);
Param get_parameter(MavNode *mavnode, const char *name);
Param get_parameter(MavNode *mavnode, int number);

void receive_parameter(MavNode *mavnode, mavlink_message_t *msg);
void set_parameter(MavNode *mavnode, char *name, float value);
void read_params_from_memory(MavNode *mavnode);
void write_param_to_memory(MavNode *mavnode, int i);

// receive.cpp
void receive_update(MavNode *mavnode);
void recv_heartbeat(MavNode *mavnode, mavlink_message_t *msg);
void recv_gps2raw(MavNode *mavnode, mavlink_message_t *msg);

// // sensors.cpp
// uint16_t rangefinder1_dist;