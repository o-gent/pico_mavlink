#include "mavnode.h"

/*
Get new mavlink packets we're interested in
*/
void MavNode::receive_update()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    unsigned long previous_run = millis();
    
    while (true)
    {
        while (SERIAL_MAVLINK.available() > 0)
        {
            uint8_t c = SERIAL_MAVLINK.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
            {
                switch (msg.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    recv_heartbeat(&msg);
                    break;
                }

                case MAVLINK_MSG_ID_GPS2_RAW:
                {
                    mavlink_gps2_raw_t gps2raw;
                    mavlink_msg_gps2_raw_decode(&msg, &gps2raw);

                    int32_t lat;
                    int32_t lon;
                    int32_t alt = gps2raw.alt;

                    Serial.print("Alt:");
                    Serial.println(alt);
                    break;
                }

                // https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
                case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                {
                    mavlink_param_request_list_t paramrequestlist;
                    mavlink_msg_param_request_list_decode(&msg, &paramrequestlist);
                    // this->send_all_parameters();
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
        // vTaskDelay((recieve_ms - (millis() - previous_run)));
        // previous_run = millis();
    }
}

void MavNode::recv_heartbeat(mavlink_message_t *msg)
{
    static bool led_toggle = false;
    digitalWrite(LED_BUILTIN, HIGH);
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);

    uint8_t systemType = heartbeat.type;
    uint8_t autopilotType = heartbeat.autopilot;
    uint8_t baseMode = heartbeat.base_mode;

    Serial.println("Heartbeat");
    if (led_toggle)
    {
        digitalWrite(LED_BUILTIN, LOW);
        led_toggle = false;
    }
    else
    {
        digitalWrite(LED_BUILTIN, HIGH);
        led_toggle = true;
    }
}