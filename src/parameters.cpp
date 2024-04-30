#include "mavnode.h"

/*
https://mavlink.io/en/services/parameter.html
*/

void param_init(MavNode *mavnode)
{
    mavnode->parameterSemaphore = xSemaphoreCreateMutex();
    if (mavnode->parameterSemaphore == NULL)
    {
        gcs_status("Semaphore creation failed");
    }
    // delay(2000);
    // wipe_param_memory(mavnode);
    read_params_from_memory(mavnode);
}

/*
Send a parameter to the autopilot
*/
void send_parameter(MavNode *mavnode, Param parameter)
{
    mavlink_param_value_t param;

    // Set the text of the message
    strncpy(param.param_id, parameter.name, sizeof(param.param_id));

    param.param_count = PARAMETER_COUNT;
    param.param_index = parameter.index;
    param.param_type = parameter.type;
    param.param_value = parameter.value;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_param_value_encode(TARGET_SYSTEM, THIS_COMPONENT, &msg, &param);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    SERIAL_MAVLINK.write(buf, len);
}

/*
iterate parameters to send to the autopilot
*/
void send_all_parameters(MavNode *mavnode)
{
    for (int i = 0; i < PARAMETER_COUNT; i++)
    {
        Param parameter = get_parameter(mavnode, i);
        send_parameter(mavnode, parameter);
    }
}

/*
Recieve an updated parameter from the autopilot
*/
void receive_parameter(MavNode *mavnode, mavlink_message_t *msg)
{
    mavlink_param_set_t paramset;
    mavlink_msg_param_set_decode(msg, &paramset);

    set_parameter(mavnode, paramset.param_id, paramset.param_value);
}

/*
Threadsafe method to get a parameter by name
*/
Param get_parameter(MavNode *mavnode, const char *name)
{
    if (mavnode->parameterSemaphore != NULL)
    {
        if (xSemaphoreTake(mavnode->parameterSemaphore, (TickType_t)10) == pdTRUE)
        {
            for (int i = 0; i < PARAMETER_COUNT; i++)
            {
                if (strcmp(mavnode->mavnodeParam[i].name, name) == 0)
                {
                    Param param = mavnode->mavnodeParam[i];
                    xSemaphoreGive(mavnode->parameterSemaphore);
                    return param;
                }
            }
        }
    }
    gcs_status("get_parameter error");
    Param param;
    return param;
}

float get_parameter_value(MavNode *mavnode, const char *name)
{
    Param result = get_parameter(mavnode, name);
    return result.value;
}

/*
Threadsafe method to get a parameter by index
*/
Param get_parameter(MavNode *mavnode, const int number)
{
    
    if (mavnode->parameterSemaphore != NULL)
    {
        if (xSemaphoreTake(mavnode->parameterSemaphore, (TickType_t)10) == pdTRUE)
        {
            Param param = mavnode->mavnodeParam[number];
            xSemaphoreGive(mavnode->parameterSemaphore);
            return param;
        }
    }
    gcs_status("get_parameter error");
    Param param;
    return param;
}

/*
Threadsafe method to set a parameter
*/
void set_parameter(MavNode *mavnode, char* name, float value)
{
    if (mavnode->parameterSemaphore != NULL)
    {
        if (xSemaphoreTake(mavnode->parameterSemaphore, (TickType_t)10) == pdTRUE)
        {
            for (int i = 0; i < PARAMETER_COUNT; i++)
            {
                if (strcmp(mavnode->mavnodeParam[i].name, name) == 0)
                {
                    mavnode->mavnodeParam[i].value = value;
                    write_param_to_memory(mavnode, i);
                    xSemaphoreGive(mavnode->parameterSemaphore);
                    return;
                }
            }
        }
    }
    gcs_status("set_parameter failed");
}

/*
read parameters from filesystem
*/
void read_params_from_memory(MavNode *mavnode)
{
    for (int i = 0; i < PARAMETER_COUNT; i++)
    {
        EEPROM.get(i, mavnode->mavnodeParam[i].value);
        Serial.println(mavnode->mavnodeParam[i].value);
    }
}

/*
save parameters to filesystem
*/
void write_param_to_memory(MavNode *mavnode, int i)
{
    EEPROM.put(i, mavnode->mavnodeParam[i].value);
    EEPROM.commit();
}

/*
wipe parameters
*/
void wipe_param_memory(MavNode *mavnode)
{
    for (int i = 0; i < PARAMETER_COUNT; i++)
    {
        EEPROM.put(i, mavnode->mavnodeParam[i].value);
    }
    EEPROM.commit();
}
