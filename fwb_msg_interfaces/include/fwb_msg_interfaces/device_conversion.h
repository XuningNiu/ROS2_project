#ifndef FW_MSG_INTERFACES__DEVICE_CONVERSION_H_
#define FW_MSG_INTERFACES__DEVICE_CONVERSION_H_

#include <stdint.h>
#include <stdbool.h>
#include <json-c/json.h>
#include "fwb_msg_interfaces/msg/device.h"

// Convert JSON to Device message
bool json_to_device(const char *json_str, fwb_msg_interfaces__msg__Device *msg);

// Convert Device message to JSON
char *device_to_json(const fwb_msg_interfaces__msg__Device *msg);

#endif // FW_MSG_INTERFACES__DEVICE_CONVERSION_H_
