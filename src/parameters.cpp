// #include "mavnode.h"

// void MavNode::param_init()
// {
//     this->parameterSemaphore = xSemaphoreCreateMutex();
//     this->read_params_from_memory();
// }

// /*
// Send a parameter to the autopilot
// */
// void MavNode::send_parameter(Param &parameter)
// {
//     mavlink_param_value_t param;

//     // Set the text of the message
//     strncpy(param.param_id, parameter.name, sizeof(param.param_id));

//     param.param_count = PARAMETER_COUNT;
//     param.param_index = parameter.index;
//     param.param_type = parameter.type;
//     param.param_value = parameter.value;

//     mavlink_message_t msg;
//     uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//     mavlink_msg_param_value_encode(TARGET_SYSTEM, THIS_COMPONENT, &msg, &param);

//     // Copy the message to the send buffer
//     uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//     SERIAL_MAVLINK.write(buf, len);
// }

// /*
// iterate parameters to send to the autopilot
// */
// void MavNode::send_all_parameters()
// {
//     for (int i = 0; i < PARAMETER_COUNT; i++)
//     {
//         Param parameter = this->get_parameter(i);
//         this->send_parameter(parameter);
//     }
// }

// /*
// Recieve an updated parameter from the autopilot
// */
// void MavNode::receive_parameter(mavlink_message_t *msg)
// {
//     mavlink_param_value_t param;
//     mavlink_msg_param_value_decode(msg, &param);

//     Param parameter;
//     parameter.index = param.param_index;
//     parameter.type = param.param_type;
//     parameter.value = param.param_value;
//     // some bodgy casting to get around const char
//     strncpy(const_cast<char*>(parameter.name), param.param_id, sizeof(param.param_id));

//     this->set_parameter(parameter);
// }

// /*
// Threadsafe method to get a parameter by name
// */
// Param MavNode::get_parameter(const char *name)
// {
//     if (parameterSemaphore != NULL)
//     {
//         if (xSemaphoreTake(parameterSemaphore, (TickType_t)10) == pdTRUE)
//         {
//             for (int i = 0; i < PARAMETER_COUNT; i++)
//             {
//                 if (strcmp(mavnodeParam[i].name, name) == 0)
//                 {
//                     Param param = mavnodeParam[i];
//                     xSemaphoreGive(parameterSemaphore);
//                     return param;
//                 }
//             }
//         }
//     }
//     this->gcs_status("get parameter error");
//     Param param;
//     return param;
// }

// /*
// Threadsafe method to get a parameter by index
// */
// Param MavNode::get_parameter(const int number)
// {
//     if (parameterSemaphore != NULL)
//     {
//         if (xSemaphoreTake(parameterSemaphore, (TickType_t)10) == pdTRUE)
//         {
//             Param param = mavnodeParam[number];
//             xSemaphoreGive(parameterSemaphore);
//             return param;
//         }
//         else
//         {
//             this->gcs_status("get_parameter semaphore error");
//             Param param;
//             return param;
//         }
//     }
//     else
//     {
//         this->gcs_status("parameters not initialized");
//         Param param;
//         return param;
//     }
// }

// /*
// Threadsafe method to set a parameter
// */
// void MavNode::set_parameter(Param &parameter)
// {
//     if (parameterSemaphore != NULL)
//     {
//         if (xSemaphoreTake(parameterSemaphore, (TickType_t)10) == pdTRUE)
//         {
//             for (int i = 0; i < PARAMETER_COUNT; i++)
//             {
//                 if (strcmp(mavnodeParam[i].name, parameter.name) == 0)
//                 {
//                     mavnodeParam[i] = parameter;
//                     this->write_params_to_memory();
//                     xSemaphoreGive(parameterSemaphore);
//                     return;
//                 }
//             }
//         }
//         else
//         {
//             this->gcs_status("set_parameter semaphore error");
//         }
//     }
//     else
//     {
//         this->gcs_status("parameters not initialized");
//     }
// }

// /*
// read parameters from filesystem
// */
// void MavNode::read_params_from_memory()
// {
//     for (int i = 0; i < PARAMETER_COUNT; i++)
//     {
//         mavnodeParam[i].value = EEPROM.read(i);
//     }
// }

// /*
// save parameters to filesystem
// */
// void MavNode::write_params_to_memory()
// {
//     for (int i = 0; i < PARAMETER_COUNT; i++)
//     {
//         EEPROM.write(i, mavnodeParam[i].value);
//     }
//     EEPROM.commit();
// }
