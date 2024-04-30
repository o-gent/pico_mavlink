#include "mavnode.h"
#include <Adafruit_NAU7802.h>
#include <SparkFun_MCP9600.h>
#include "AS5600.h"

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
void servo1_update(MavNode *mavnode)
{
    Adafruit_NAU7802 nau;
    MCP9600 mcp;
    AS5600 as5600;


    nau.begin();
    


    Wire.begin();
    Wire.setClock(100000);
    
    mcp.begin(0x66);

    as5600.begin();
    as5600.setDirection(AS5600_CLOCK_WISE);

    unsigned long previous_run = millis();
    int i = 0;

    mavlink_battery_status_t battery;
    battery.id = 4;
    battery.id = get_parameter_value(mavnode, "BATTID");

    strainmap_type *strainmap = new strainmap_type();

    strainmap->strain_nlow = -100000;
    strainmap->strain_nmid = -50000;
    strainmap->strain_zero = 30000;
    strainmap->strain_mid = 80000;
    strainmap->strain_high = 120000;

    strainmap->nlow = -1000;
    strainmap->nmid = -500;
    strainmap->zero = 0;
    strainmap->mid = 500;
    strainmap->high = 1000;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    while (true)
    {

        int32_t strain_reading = nau.read();
        uint16_t servo_angle = as5600.readAngle();
        float temperature = mcp.getThermocoupleTemp();
        
        battery.voltages[0] = 0;
        battery.temperature = servo_angle;
        battery.current_battery = map_strain(strain_reading, *strainmap);
        battery.battery_remaining = temperature;

        Serial.print(strain_reading);
        Serial.print(" - ");
        Serial.print(temperature);
        Serial.print(" - ");
        Serial.println(servo_angle);


        mavlink_msg_battery_status_encode(TARGET_SYSTEM, THIS_COMPONENT, &msg, &battery);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        SERIAL_MAVLINK.write(buf, len);

        vTaskDelay(rangefinder1_ms - (millis() - previous_run));
        previous_run = millis();
    }
}

int32_t map_strain(int32_t strain_reading, strainmap_type &strainmap)
{
    if (strain_reading >= strainmap.strain_zero)
    {
        if (strain_reading >= strainmap.strain_mid)
        {
            return map(strain_reading, strainmap.strain_mid, strainmap.strain_high, strainmap.mid, strainmap.high);
        }
        else
        {
            return map(strain_reading, strainmap.strain_zero, strainmap.strain_mid, strainmap.zero, strainmap.mid);
        }
    }
    else
    {
        if (strain_reading <= strainmap.strain_nmid)
        {
            return map(strain_reading, strainmap.strain_nmid, strainmap.strain_nlow, strainmap.nmid, strainmap.nlow);
        }
        else
        {
            return map(strain_reading, strainmap.strain_zero, strainmap.strain_nmid, strainmap.zero, strainmap.nmid);
        }
    }
}