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

#define PARAMETER_COUNT 2

struct Param
{
    const char *name;
    uint16_t index;
    uint8_t type;
    float value;
};

class MavNode
{
    public:
        void gps2raw_request();     // commands
        void mav_request_data();    // commands
        void receive_update();      // receive
        void rangefinder1_update(); // sensors
        void rangefinder2_update(); // sensors
        void heartbeat_update();    // commands
        void param_init(void);      // parameters

        Param mavnodeParam[PARAMETER_COUNT] = {
            {"PARAM1", 1, MAV_PARAM_TYPE_UINT16, 0},
            {"PARAM2", 2, MAV_PARAM_TYPE_UINT16, 0}
        };

        SemaphoreHandle_t parameterSemaphore = NULL;
    private:
        // commands.cpp
        void reboot();
        void gcs_status(const char gcstext[]);

        //parameters.cpp
        void send_parameter(Param parameter);
        void send_all_parameters(void);
        Param get_parameter(const char *name);
        Param get_parameter(int number);

        void receive_parameter(mavlink_message_t *msg);
        void set_parameter(char* name, float value);
        void read_params_from_memory(void);
        void write_params_to_memory(void);

        // receive.cpp
        void recv_heartbeat(mavlink_message_t *msg);
        void recv_gps2raw(mavlink_message_t *msg);

        // sensors.cpp
        uint16_t rangefinder1_dist;
};