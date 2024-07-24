#include "fwb_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>

int connect_ros(ros_client_t *client, const char *server_ip, int port) {
    // 创建socket
    if ((client->sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    client->serv_addr.sin_family = AF_INET;
    client->serv_addr.sin_port = htons(port);

    // 将IPv4和IPv6地址从文本转换为二进制形式
    if (inet_pton(AF_INET, server_ip, &client->serv_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        return -1;
    }

    // 连接到服务器
    if (connect(client->sock, (struct sockaddr *)&client->serv_addr, sizeof(client->serv_addr)) < 0) {
        perror("Connection Failed");
        return -1;
    }

    return 0;
}

int send_ros(ros_client_t *client, const char *topic_name, const char *message) {
    // 计算封装后的字符串的长度
    size_t buffer_size = strlen("topic:") + strlen(topic_name) + strlen(",data:") + strlen(message) + 10;
    char *buffer = (char *)malloc(buffer_size);

    if (buffer == NULL) {
        perror("Malloc error");
        return -1;
    }

    // 创建封装后的字符串
    snprintf(buffer, buffer_size, "topic:%s,data:%s\n\n", topic_name, message);

    //printf("send msg : ---> %s\n", buffer);
    // 发送数据到服务器
    if (send(client->sock, buffer, strlen(buffer), 0) < 0) {
        perror("Send error");
        free(buffer);
        return -1;
    }

    free(buffer);
    return 0;
}

void close_ros(ros_client_t *client) {
    // 关闭socket
    close(client->sock);
}
