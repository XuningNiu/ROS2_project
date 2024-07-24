CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -fPIC -g
LDFLAGS = -L. -lfwbclient -Wl,-rpath,.

# 包含必要的 include 路径
INCLUDES = -I. \
           -I/home/yrh/ros2/install/fwb_msg_interfaces/include \
           -I/home/yrh/ros2/install/fwb_msg_interfaces/include/fwb_msg_interfaces \
           -I/home/yrh/ros2/install/json-c/include

# 链接必要的库
LIBS = -L/home/yrh/ros2/install/fwb_msg_interfaces/lib \
       -L/home/yrh/ros2/install/json-c/lib \
       -lfwb_msg_interfaces \
       -ljson-c

LIBRARY = libfwbclient.so
SOURCES = fwb_client.c
HEADERS = fwb_client.h
EXECUTABLE = fw_client_test
MAIN_SOURCE = main.c

all: $(LIBRARY) $(EXECUTABLE)

$(LIBRARY): $(SOURCES)
	$(CC) $(CFLAGS) $(INCLUDES) -shared -o $(LIBRARY) $(SOURCES) $(LIBS)

$(EXECUTABLE): $(MAIN_SOURCE) $(LIBRARY)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(EXECUTABLE) $(MAIN_SOURCE) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(LIBRARY) $(EXECUTABLE)