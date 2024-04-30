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

#define PARAMETER_COUNT 11

struct Param
{
    const char *name;
    uint16_t index;
    uint8_t type;
    int32_t value;
};


struct MavNode
{
    SemaphoreHandle_t parameterSemaphore = NULL;
    Param mavnodeParam[PARAMETER_COUNT] = {
        {"BATTID", 1, MAV_PARAM_TYPE_INT32, 0},
        {"STRAIN_NLOW", 2, MAV_PARAM_TYPE_INT32, 0},
        {"STRAIN_NMID", 3, MAV_PARAM_TYPE_INT32, 0},
        {"STRAIN_ZERO", 4, MAV_PARAM_TYPE_INT32, 0},
        {"STRAIN_MID", 5, MAV_PARAM_TYPE_INT32, 0},
        {"STRAIN_HIGH", 6, MAV_PARAM_TYPE_INT32, 0},
        {"NLOW", 7, MAV_PARAM_TYPE_INT32, 0},
        {"NMID", 8, MAV_PARAM_TYPE_INT32, 0},
        {"ZERO", 9, MAV_PARAM_TYPE_INT32, 0},
        {"MID", 10, MAV_PARAM_TYPE_INT32, 0},
        {"HIGH", 11, MAV_PARAM_TYPE_INT32, 0},
    };

};

struct strainmap_type {
    int32_t nlow;
    int32_t nmid;
    int32_t zero;
    int32_t mid;
    int32_t high;
    
    int32_t strain_nlow;
    int32_t strain_nmid;
    int32_t strain_zero;
    int32_t strain_mid;
    int32_t strain_high;
};

void gps2raw_request(MavNode *mavnode);     // commands
void mav_request_data(MavNode *mavnode);    // commands
void rangefinder1_update(MavNode *mavnode); // sensors
void servo1_update(MavNode *mavnode); // sensors
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
float get_parameter_value(MavNode *mavnode, const char *name);
void wipe_param_memory(MavNode *mavnode);

void receive_parameter(MavNode *mavnode, mavlink_message_t *msg);
void set_parameter(MavNode *mavnode, char *name, float value);
void read_params_from_memory(MavNode *mavnode);
void write_param_to_memory(MavNode *mavnode, int i);

// receive.cpp
void receive_update(MavNode *mavnode);
void recv_heartbeat(MavNode *mavnode, mavlink_message_t *msg);
void recv_gps2raw(MavNode *mavnode, mavlink_message_t *msg);

// sensors.cpp
int32_t map_strain(int32_t strain_reading, strainmap_type &strainmap);