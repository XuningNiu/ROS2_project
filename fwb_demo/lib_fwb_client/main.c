#include "fwb_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <signal.h>

#define MAX_TOPICS 10  // Maximum number of topics

int main(int argc, char *argv[]) {
    if (argc < 2 || argc > MAX_TOPICS + 1) {
        printf("Usage: %s <topic1> [<topic2> ... <topic%d>]\n", argv[0], MAX_TOPICS);
        return 1;
    }
    signal(SIGPIPE, SIG_IGN); 
    
    for (int i = 1; i < argc; i++) {
        printf("input %s\n", argv[i]);
    }
    ros_client_t client;
    const char *server_ip = "127.0.0.1";  // Replace with server IP if needed
    int port = 8080;

    if (connect_ros(&client, server_ip, port) < 0) {
        return -1;
    }

    int counter = 0;
    while (1) {
        counter++;
        char message[50];
        snprintf(message, sizeof(message), "Counter: %d", counter);

        for (int i = 1; i < argc; i++) {
            printf("send count %d\n", counter);
            if (send_ros(&client, argv[i], message) < 0) {
                if (errno == EPIPE) {
                    fprintf(stderr, "Main: Send error: socket closed by peer\n");
                } 
                break;
            }
        }

        sleep(5);
    }

    close_ros(&client);
    return 0;
}
