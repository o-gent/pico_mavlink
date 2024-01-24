#include <Arduino.h>
#include <ardupilotmega/mavlink.h>

void reboot(){

  //TARGET DRONE
  uint8_t target_system = 1; // Target drone id
  uint8_t target_component = 0; // Target component, 0 = all
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

 mavlink_msg_command_long_pack( 1, 255, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial2.write(buf, len);
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RAW_SENSORS};
  const uint16_t MAVRates[maxStreams] = {0x02,0x05};

    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial2.write(buf, len);
    }
}

void heartbeat() {
    int sysid = 100;                   ///< ID 20 for this airplane
    int compid = MAV_COMP_ID_GPS;     ///< The component sending the message is the IMU, it could be also a Linux process
    int type = MAV_TYPE_GPS;   ///< This system is an airplane / fixed wing

    // Define the system type, in this case an airplane
    uint8_t system_type = MAV_TYPE_FIXED_WING;
    uint8_t autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;

    uint8_t system_mode = MAV_MODE_MANUAL_ARMED; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight
    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Serial2.write(buf,len);
}

void setup()
{
    Serial2.begin(57600);
    Serial.begin(9600);
    Serial.println("started");
    Mav_Request_Data();
}
void loop()
{
    heartbeat();
    mavlink_message_t msg;
    mavlink_status_t status;

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

                    Serial.print("Message received\n");
                    Serial.print("System type: ");
                    Serial.println(systemType);
                    Serial.print("Autopilot type: ");
                    Serial.println(autopilotType);
                    Serial.print("Base mode: ");
                    Serial.println(baseMode);
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
}