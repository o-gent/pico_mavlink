#include "mavnode.h"


/*
Example of how to send a command
*/
void reboot(MavNode *mavnode)
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

    mavlink_msg_command_long_pack(TARGET_SYSTEM, THIS_COMPONENT, &msg, TARGET_SYSTEM, TARGET_COMPONENT, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    SERIAL_MAVLINK.write(buf, len);
}


/*

*/
void gps2raw_request(MavNode *mavnode)
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

    mavlink_msg_command_long_pack(TARGET_SYSTEM, THIS_COMPONENT, &msg, TARGET_SYSTEM, TARGET_COMPONENT, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // Send the message (.write sends as bytes)
    SERIAL_MAVLINK.write(buf, len);
}


/*
Request data streams from the autopilot
*/
void mav_request_data(MavNode *mavnode)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // To be setup according to the needed information to be requested from the Pixhawk
    const int maxStreams = 1;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS};
    const uint16_t MAVRates[maxStreams] = {0x0A};

    for (int i = 0; i < maxStreams; i++)
    {
        mavlink_msg_request_data_stream_pack(TARGET_SYSTEM, THIS_COMPONENT, &msg, TARGET_SYSTEM, TARGET_COMPONENT, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        SERIAL_MAVLINK.write(buf, len);
    }
}

/*

*/
void gcs_status(const char gcstext[])
{
    mavlink_statustext_t statustext;
    memset(&statustext, 0, sizeof(statustext));

    // Set the text of the message
    strncpy(statustext.text, gcstext, sizeof(statustext.text));

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_statustext_encode(TARGET_SYSTEM, THIS_COMPONENT, &msg, &statustext);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    SERIAL_MAVLINK.write(buf, len);
}