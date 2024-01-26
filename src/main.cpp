#include <Arduino.h>
#include <ardupilotmega/mavlink.h>
#include <rtos.h>
#include "mbed.h"

#define target_system 100
#define target_component 1
#define this_component MAV_COMP_ID_GPS

const unsigned long heartbeat_ms = 1000;
const unsigned long rangefinder1_ms = 50;
const unsigned long recieve_ms = 50;

rtos::Thread heartbeat_thread;
rtos::Thread rangefinder1_thread;
rtos::Thread recieve_mavlink;

/*
Example of how to send a command
*/
void reboot()
{
    uint16_t command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN; // Specific command for PX4
    uint8_t confirmation = 0;
    float param1 = 1;
    float param2 = 0;
    float param3 = 0;
    float param4 = 0;
    float param5 = 0;
    float param6 = 0;
    float param7 = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(target_system, this_component, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    Serial2.write(buf, len);
}


/*

*/
void gps2raw_request()
{
    uint16_t command = MAV_CMD_SET_MESSAGE_INTERVAL; // Specific command for PX4
    uint8_t confirmation = 0;
    float param1 = 124;    // GPS2_RAW
    float param2 = 100000; // 10Hz
    float param3 = 0;
    float param4 = 0;
    float param5 = 0;
    float param6 = 0;
    float param7 = 0;

    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_command_long_pack(target_system, this_component, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    Serial2.write(buf, len);
}


/*
for a given value, send this as a rangefinder data point
*/
void rangefinder_send(mavlink_distance_sensor_t *dist)
{

}


/*
Request data streams from the autopilot
*/
void mav_request_data()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // To be setup according to the needed information to be requested from the Pixhawk
    const int maxStreams = 1;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS};
    const uint16_t MAVRates[maxStreams] = {0x0A};

    for (int i = 0; i < maxStreams; i++)
    {
        mavlink_msg_request_data_stream_pack(target_system, this_component, &msg, target_system, target_component, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        Serial2.write(buf, len);
    }
}

/*
Routine to let the host ardupilot know we're still here
We only need to create this message once
*/
void heartbeat_update()
{
    unsigned long previous_run = millis();

    int type = MAV_TYPE_GPS; ///< This system is an airplane / fixed wing

    // Define the system type, in this case an airplane
    uint8_t system_type = MAV_TYPE_FIXED_WING;
    uint8_t autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;

    uint8_t system_mode = MAV_MODE_MANUAL_ARMED; ///< Booting up
    uint32_t custom_mode = 0;                    ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_ACTIVE;     ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(target_system, this_component, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    while (true)
    {
        Serial2.write(buf, len);

        rtos::ThisThread::sleep_for(heartbeat_ms - (millis() - previous_run));
        previous_run = millis();
    }
}

/*
Routine for sending new rangefinder information
*/
void rangefinder_update()
{
    unsigned long previous_run = millis();
    int i = 0;

    mavlink_distance_sensor_t dist;
    dist.time_boot_ms = millis();
    dist.min_distance = 0;
    dist.max_distance = UINT16_MAX;
    dist.type = MAV_DISTANCE_SENSOR_UNKNOWN;
    dist.id = 0;
    // the orientation needs to match the orientation set in ardupilot otherwise it'll be rejected
    dist.orientation = MAV_SENSOR_ROTATION_PITCH_270;
    dist.covariance = UINT8_MAX;
    dist.horizontal_fov = 0;
    dist.vertical_fov = 0;
    dist.signal_quality = 0;
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    while (true)
    {
        dist.current_distance = i++;

        mavlink_msg_distance_sensor_encode(target_system, this_component, &msg, &dist);

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        Serial2.write(buf, len);
        
        rtos::ThisThread::sleep_for(rangefinder1_ms - (millis() - previous_run));
        previous_run = millis();
    }
}

/*
Get new mavlink packets we're interested in
*/
void recieve_update()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    unsigned long previous_run = millis();

    while (true)
    {
        while (Serial2.available() > 0)
        {
            uint8_t c = Serial2.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
            {

                switch (msg.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                    uint8_t systemType = heartbeat.type;
                    uint8_t autopilotType = heartbeat.autopilot;
                    uint8_t baseMode = heartbeat.base_mode;

                    Serial.println("Heartbeat");
                    break;
                }

                case MAVLINK_MSG_ID_GPS2_RAW:
                {
                    mavlink_gps2_raw_t gps2raw;
                    mavlink_msg_gps2_raw_decode(&msg, &gps2raw);

                    int32_t lat;
                    int32_t lon;
                    int32_t alt;

                    Serial.print("Alt:");
                    Serial.println(alt);
                    break;
                }

                default:
                {
                    break;
                }
                }
                // else{
                // Serial.print("No Heartbeat\n");
                //}
            }
        }
        rtos::ThisThread::sleep_for(recieve_ms - (millis() - previous_run));
        previous_run = millis();
    }
}


/*
Setup comms links, start threads
*/
void setup()
{
    Serial2.begin(57600);
    Serial.begin(9600);
    Serial.println("started");

    mav_request_data();
    gps2raw_request();

    //heartbeat_thread.set_priority(osPriorityLow);
    heartbeat_thread.start(heartbeat_update);

    //rangefinder1_thread.set_priority(osPriorityRealtime2);
    rangefinder1_thread.start(rangefinder_update);

    //recieve_mavlink.set_priority(osPriorityRealtime1);
    recieve_mavlink.start(recieve_update);
}


/*
Our loop does nothing as all the activities are in threads
*/
void loop()
{
    rtos::ThisThread::sleep_for(1000);
}