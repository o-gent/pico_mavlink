#include "mavnode.h"


/*
Routine to let the host ardupilot know we're still here
We only need to create this message once
*/
void heartbeat_update(MavNode *mavnode)
{
    unsigned long previous_run = millis();

    int type = MAV_TYPE_GPS; ///< This system is an airplane / fixed wing

    uint8_t autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;

    uint8_t system_mode = MAV_MODE_MANUAL_ARMED; ///< Booting up
    uint32_t custom_mode = 0;                    ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_ACTIVE;     ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(TARGET_SYSTEM, THIS_COMPONENT, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    while (true)
    {
        SERIAL_MAVLINK.write(buf, len);

        vTaskDelay(heartbeat_ms - (millis() - previous_run));
        previous_run = millis();
    }
}


/*
Routine for sending new rangefinder information
*/
void rangefinder1_update(MavNode *mavnode)
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

        mavlink_msg_distance_sensor_encode(TARGET_SYSTEM, THIS_COMPONENT, &msg, &dist);

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        SERIAL_MAVLINK.write(buf, len);
        
        vTaskDelay(rangefinder1_ms - (millis() - previous_run));
        previous_run = millis();
    }
}


/*
Routine for sending new rangefinder information
*/
void rangefinder2_update(MavNode *mavnode)
{
    unsigned long previous_run = millis();
    int i = 0;

    mavlink_distance_sensor_t dist;
    dist.time_boot_ms = millis();
    dist.min_distance = 0;
    dist.max_distance = UINT16_MAX;
    dist.type = MAV_DISTANCE_SENSOR_UNKNOWN;
    dist.id = 1;
    // the orientation needs to match the orientation set in ardupilot otherwise it'll be rejected
    dist.orientation = MAV_SENSOR_ROTATION_NONE;
    dist.covariance = UINT8_MAX;
    dist.horizontal_fov = 0;
    dist.vertical_fov = 0;
    dist.signal_quality = 0;
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    while (true)
    {
        i++;
        dist.current_distance = i++;

        mavlink_msg_distance_sensor_encode(TARGET_SYSTEM, THIS_COMPONENT, &msg, &dist);

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        SERIAL_MAVLINK.write(buf, len);
        
        vTaskDelay(rangefinder1_ms - (millis() - previous_run));
        previous_run = millis();
    }
}