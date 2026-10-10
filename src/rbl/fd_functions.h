#ifndef H_fd_functions_h
#define H_fd_functions_h
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#define INCLUDE_SOCKET_FUNCTIONS = 1

typedef int socket_handle_t;

// #include "ErrorCodes.h"

#ifndef SOCKET_STATUS_GOOD
#define SOCKET_STATUS_GOOD  0
#define SOCKET_STATUS_EOF   -1001
#define SOCKET_STATUS_ERROR -1002
#define SOCKET_STATUS_EAGAIN -1003

#endif
void socket_throw_error(socket_handle_t socket, int errorno, const char* message);

void socket_report_error(const char* format, ...);

//
//
//
socket_handle_t sockt_create_listener_on_port(int port);

//
// Connect to a host/port and return the socket that established the connection.
// Returns the fd of the openned socket that implements the connection.
//
// ipAddress is a char* representation of the host/port ip address.
//
// Uses conditional compilation to include/exclude code to handle ipv6
//
socket_handle_t socket_connect_host_port( char* hostname, unsigned short port, int* status);

socket_handle_t socket_create_listener_on_port(int port);

socket_handle_t socket_set_blocking(socket_handle_t socket);

bool    socket_is_blocking(socket_handle_t socket);

int fd_set_non_blocking(int fd);

void socket_set_option(socket_handle_t socket);

void socket_set_reuse_addr(socket_handle_t socket);

void socket_set_recvtimeout(socket_handle_t socket, int time_interval_seconds);

void socket_shutdown_read(socket_handle_t socket);

void socket_shutdown_write(socket_handle_t socket);

void socket_wait_for_write_flush(socket_handle_t socket);

bool socket_write_data(socket_handle_t socket, void* buffer, int buffer_length, int* status);

int socket_read_data( socket_handle_t socket, void* buffer, int buffer_length, int* status);

void socket_close(socket_handle_t socket);

#endif
