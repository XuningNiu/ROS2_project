CC = gcc
CFLAGS = -Wall -Wextra -Wpedantic -fPIC -g
LDFLAGS = -L. -lfwbclient -Wl,-rpath,.

LIBRARY = libfwbclient.so
SOURCES = fwb_client.c
HEADERS = fwb_client.h
EXECUTABLE = fw_client_test
MAIN_SOURCE = main.c

all: $(LIBRARY) $(EXECUTABLE)

$(LIBRARY): $(SOURCES)
	$(CC) $(CFLAGS) -shared -o $(LIBRARY) $(SOURCES)

$(EXECUTABLE): $(MAIN_SOURCE) $(LIBRARY)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(MAIN_SOURCE) $(LDFLAGS)

clean:
	rm -f $(LIBRARY) $(EXECUTABLE)