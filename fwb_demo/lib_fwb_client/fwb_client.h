#ifndef FWB_ROS_CLIENT_H
#define FWB_ROS_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <netinet/in.h>

typedef struct {
    int sock;
    struct sockaddr_in serv_addr;
} ros_client_t;

// 连接到服务器
int connect_ros(ros_client_t *client, const char *server_ip, int port);

// 发送数据到服务器
int send_ros(ros_client_t *client, const char *topic_name, const char *message);

// 关闭连接
void close_ros(ros_client_t *client);

#ifdef __cplusplus
}
#endif

#endif // FWB_ROS_CLIENT_H
