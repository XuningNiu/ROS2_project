#include <stdlib.h>
#include <string.h>
#include <json-c/json.h>
#include "fwb_msg_interfaces/device_conversion.h"

bool json_to_device(const char *json_str, fwb_msg_interfaces__msg__Device *msg) {
    struct json_object *parsed_json;
    struct json_object *id;
    struct json_object *name;
    struct json_object *email;
    struct json_object *age;
    struct json_object *is_active;
    struct json_object *newsletter;
    struct json_object *notifications;

    parsed_json = json_tokener_parse(json_str);
    if (parsed_json == NULL) {
        return false;
    }

    if (!json_object_object_get_ex(parsed_json, "id", &id) ||
        !json_object_object_get_ex(parsed_json, "name", &name) ||
        !json_object_object_get_ex(parsed_json, "email", &email) ||
        !json_object_object_get_ex(parsed_json, "age", &age) ||
        !json_object_object_get_ex(parsed_json, "is_active", &is_active) ||
        !json_object_object_get_ex(parsed_json, "newsletter", &newsletter) ||
        !json_object_object_get_ex(parsed_json, "notifications", &notifications)) {
        json_object_put(parsed_json);
        return false;
    }

    msg->id = json_object_get_int(id);
    strncpy(msg->name.data, json_object_get_string(name), msg->name.capacity);
    strncpy(msg->email.data, json_object_get_string(email), msg->email.capacity);
    msg->age = json_object_get_int(age);
    msg->is_active = json_object_get_boolean(is_active);
    msg->newsletter = json_object_get_boolean(newsletter);
    msg->notifications = json_object_get_boolean(notifications);

    json_object_put(parsed_json);
    return true;
}

char *device_to_json(const fwb_msg_interfaces__msg__Device *msg) {
    struct json_object *json_obj = json_object_new_object();
    struct json_object *id = json_object_new_int(msg->id);
    struct json_object *name = json_object_new_string(msg->name.data);
    struct json_object *email = json_object_new_string(msg->email.data);
    struct json_object *age = json_object_new_int(msg->age);
    struct json_object *is_active = json_object_new_boolean(msg->is_active);
    struct json_object *newsletter = json_object_new_boolean(msg->newsletter);
    struct json_object *notifications = json_object_new_boolean(msg->notifications);

    json_object_object_add(json_obj, "id", id);
    json_object_object_add(json_obj, "name", name);
    json_object_object_add(json_obj, "email", email);
    json_object_object_add(json_obj, "age", age);
    json_object_object_add(json_obj, "is_active", is_active);
    json_object_object_add(json_obj, "newsletter", newsletter);
    json_object_object_add(json_obj, "notifications", notifications);

    const char *json_str = json_object_to_json_string(json_obj);
    char *result = strdup(json_str);

    json_object_put(json_obj);
    return result;
}
