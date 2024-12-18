//
//  Socket.m
//  threaded
//
//  Created by ROBERT BLACKWELL on 9/30/15.
//  Copyright © 2015 Blackwellapps. All rights reserved.
//

#include "fd_functions.h"
#include <string>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#if 0
void socket_throw_error(socket_handle_t socket, int errorno, const char* message)
{
    const char* m = message;
    int eno = (int) errorno;
    int socket_fd = (int) socket;
    fprintf(stderr, " SocketError[%s ], errno:%d (%s)  socket_fd: %d", m, eno, strerror(errorno), socket_fd );
    assert(0);
    //throw std::runtime_error(msg);
}

void socket_report_error(const char* format, ...)
{
    const char* fmt = format;
    va_list ap;
    va_start(ap, format); //Requires the last fixed parameter (to get the address)
    printf("socket_report_error\n");
    vprintf(fmt, ap);
    va_end(ap);
    return;
}

//
//
//
socket_handle_t socket_create_listener_on_port(int port)
{
    socket_handle_t tmp_socket;
    if( (tmp_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1 )
    {
        socket_throw_error(0, errno, "socket call failed");
    }
    struct sockaddr_in sin;
    
    memset(&sin, 0, sizeof(sin));
    // sin.sin_len = sizeof(sin);
    sin.sin_family = AF_INET; // or AF_INET6 (address family)
    sin.sin_port = htons(port);
//    sin.sin_addr.s_addr= INADDR_ANY;
    sin.sin_addr.s_addr = inet_addr("127.0.0.1");
    int result;
    int yes = 1;
    if( (result = setsockopt(tmp_socket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes))) != 0 )
    {
        socket_throw_error(tmp_socket, errno, "the bind failed %d");
    }
    if( (result = bind(tmp_socket, (struct sockaddr *)&sin, sizeof(sin))) != 0)
    {
        socket_throw_error(tmp_socket, errno, "the bind failed %d");
    }
    if((result = listen(tmp_socket, SOMAXCONN)) != 0)
    {
        socket_throw_error(tmp_socket, errno, "the listen failed %d");
    }
    return tmp_socket;
}

//
// Connect to a host/port and return the socket that established the connection.
// Returns the fd of the openned socket that implements the connection.
//
// ipAddress is a NSString representation of the host/port ip address.
//
// Uses conditional compilation to include/exclude code to handle ipv6
//
socket_handle_t socket_connect_host_port( char* hostname, unsigned short port, int* status )
{
#ifdef USE_IPV6
    struct addrinfo hints;
    char portstr[10];
    int gaierr;
    struct addrinfo* ai;
    struct addrinfo* ai2;
    struct addrinfo* aiv4;
    struct addrinfo* aiv6;
    struct sockaddr_in6 sa;
#else /* USE_IPV6 */
    struct hostent *he;
    struct sockaddr_in sa;
#endif /* USE_IPV6 */
    int sa_len, sock_family, sock_type, sock_protocol;
    int sockfd;
    
    (void) memset( (void*) &sa, 0, sizeof(sa) );
    
#ifdef USE_IPV6
    
    (void) memset( &hints, 0, sizeof(hints) );
    hints.ai_family = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    (void) snprintf( portstr, sizeof(portstr), "%d", (int) port );
    if ( (gaierr = getaddrinfo( hostname, portstr, &hints, &ai )) != 0 ){
        socket_throw_error(0, errno, "getaddrinfo failed");
        *status = errno;
        return 0;
    }
    /* Find the first IPv4 and IPv6 entries. */
    aiv4 = (struct addrinfo*) 0;
    aiv6 = (struct addrinfo*) 0;
    for ( ai2 = ai; ai2 != (struct addrinfo*) 0; ai2 = ai2->ai_next )
    {
        switch ( ai2->ai_family )
        {
            case AF_INET:
                if ( aiv4 == (struct addrinfo*) 0 )
                    aiv4 = ai2;
                break;
            case AF_INET6:
                if ( aiv6 == (struct addrinfo*) 0 )
                    aiv6 = ai2;
                break;
        }
    }
    
    /* If there's an IPv4 address, use that, otherwise try IPv6. */
    if ( aiv4 != (struct addrinfo*) 0 )
    {
        if ( sizeof(sa) < aiv4->ai_addrlen )
        {
            (void) fprintf(
                           stderr, "%s - sockaddr too small (%lu < %lu)\n",
                           hostname, (unsigned long) sizeof(sa),
                           (unsigned long) aiv4->ai_addrlen );
            exit( 1 );
        }
        sock_family = aiv4->ai_family;
        sock_type = aiv4->ai_socktype;
        sock_protocol = aiv4->ai_protocol;
        sa_len = aiv4->ai_addrlen;
        (void) memmove( &sa, aiv4->ai_addr, sa_len );
        goto ok;
    }
    if ( aiv6 != (struct addrinfo*) 0 )
    {
        if ( sizeof(sa) < aiv6->ai_addrlen )
        {
            (void) fprintf(
                           stderr, "%s - sockaddr too small (%lu < %lu)\n",
                           hostname, (unsigned long) sizeof(sa),
                           (unsigned long) aiv6->ai_addrlen );
            exit( 1 );
        }
        sock_family = aiv6->ai_family;
        sock_type = aiv6->ai_socktype;
        sock_protocol = aiv6->ai_protocol;
        sa_len = aiv6->ai_addrlen;
        (void) memmove( &sa, aiv6->ai_addr, sa_len );
        goto ok;
    }
    
    socket_throw_error(sockfd, errno, "Unknown host." );
    
ok:
    freeaddrinfo( ai );
    
#else /* USE_IPV6 */
    
    he = gethostbyname( hostname );
    if ( he == (struct hostent*) 0 ){
        printf("ERROR gethostname %d %s\n", errno, strerror(errno));
        socket_throw_error(sockfd, errno,   "Unknown host." );
        *status = errno;
        return 0;
    }
    sock_family = sa.sin_family = he->h_addrtype;
    sock_type = SOCK_STREAM;
    sock_protocol = 0;
    sa_len = sizeof(sa);
    (void) memmove( &sa.sin_addr, he->h_addr, he->h_length );
    sa.sin_port = htons( port );
    
#endif /* USE_IPV6 */
    
    sockfd = socket( sock_family, sock_type, sock_protocol );
    if ( sockfd < 0 ){
        printf("ERROR socket %d %s\n", errno, strerror(errno));
        socket_throw_error(sockfd, errno,  "socket call refused." );
        *status = errno;
        return 0;
    }
//    int result;
//    int yes = 1;
//    if( (result = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes))) != 0 )
//    {
//        printf("ERROR setsockoptt %d %s\n", errno, strerror(errno));
//        socket_throw_error(sockfd, errno, "the setsockopt failed %d");
//    }

    int value = 1;
    // setsockopt(sockfd, SOL_SOCKET, SO_NOSIGPIPE, &value, sizeof(value));

    if ( connect( sockfd, (struct sockaddr*) &sa, sa_len ) < 0 ){
        printf("ERROR connect %d %s\n", errno, strerror(errno));
        socket_throw_error(sockfd, errno,  "Connection refused." );
        *status = errno;
        return 0;
    }
    *status = 0;
    return sockfd;
}
bool    socket_is_blocking(socket_handle_t socket)
{
    int     val;
    bool    is_blocking = false;
    int junk = O_NONBLOCK;
    int tmp;
    int tmp2;
    
    if ( (val = fcntl(socket, F_GETFL, 0)) < 0)
    {
        socket_throw_error(socket, errno,  "FCNTL failed" );
    }
    else
    {
        int t1 = 0x0004;
        int t2 = 0xFFF2;
        int t3 = t1 & t2;
        int t4 = ! t3;
        
        tmp = (val & O_NONBLOCK);
        tmp2 = ! tmp;
        is_blocking = !(val & O_NONBLOCK);
    }
    return is_blocking;
}

int fd_set_blocking(int fd)
{
    //
    // Ensure socket is in blocking mode
    //
    int flags = fcntl(fd, F_GETFL, 0);
    int modFlags2 = flags & ~O_NONBLOCK;
    int fres = fcntl(fd, F_SETFL, modFlags2);
    if( fres < 0 ){
        socket_throw_error(socket, errno, "non-blocking fcntl failed");
    }
    return fd;
}
#endif
socket_handle_t fd_set_non_blocking(socket_handle_t socket)
{
    //
    // Ensure socket is in blocking mode
    //
    int flags = fcntl(socket, F_GETFL, 0);
    int modFlags2 = flags | O_NONBLOCK;
    int fres = fcntl(socket, F_SETFL, modFlags2);
    if( fres < 0 ){
        throw std::string("non_blocking failed");
    }
    return socket;
}
#if 0
void socket_shutdown_read(socket_handle_t socket)
{
    shutdown(socket, SHUT_RD);
}

void socket_shutdown_write(socket_handle_t socket)
{
    shutdown(socket, SHUT_RD);
}

void socket_set_option(socket_handle_t socket)
{
    int yes = 1;
    bool result = (
                   // ( setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes)) < 0 )
                   ( setsockopt(socket, SOL_SOCKET, SO_LINGER, &yes, sizeof(yes)) < 0 )
                   // || setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes)) < 0 )
                   // ||  ( setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)) < 0)
                   );
    if( ! result )
        socket_throw_error(socket, errno, "setsockopt failed");
}

void socket_set_reuse_addr(socket_handle_t socket)
{
    int result;
    int yes = 1;
    if( (result = setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes))) < 0 )
    {
        socket_report_error("the setsockopt REUSEADDR failed %d ", result);
    }
}

void socket_set_recvtimeout(socket_handle_t socket, int time_interval_seconds)
{
    struct timeval timeout;
    timeout.tv_sec = time_interval_seconds;
    timeout.tv_usec = 0;
    int errno_saved = 0;
    int fd = (int)socket;
    char* msg;
    int status = setsockopt (fd, SOL_SOCKET, SO_RCVTIMEO, (void *)&timeout, sizeof(timeout));
    if(status < 0) {
        errno_saved = errno;
        msg = strerror(errno_saved);
        printf("socket_set_recvto failed socket: %d errno: %d %s\n", socket, errno_saved, strerror(errno_saved));
        assert(false);
    }
}

void socket_wait_for_write_flush(socket_handle_t socket)
{
    assert(0);
    // int raw_sock = socket;
    
    // shutdown(raw_sock, SHUT_WR);
    // char junk;
    // int junk_len = sizeof(junk);
    // long recv_res = recv(raw_sock, &junk, junk_len, 0);
    // if( recv_res < 0){
    //     socket_throw_error(socket, errno, "wait on flush recv call failed");
    // }
    
    // long count=3;
    // socklen_t len = sizeof(count);
    // int r = getsockopt(raw_sock, SOL_SOCKET, SO_NWRITE, &count, &len);
    // if( r < 0 ){
    //     socket_throw_error(socket, errno, "SO_NWRITE call failed");
    // }
    // if( count > 0){
    //     socket_throw_error(socket, errno, "SO_NWRITE call returned count > 0 treat as io error");
    // }
}

bool socket_write_data(socket_handle_t socket,  void* buffer, int buffer_length, int* status)
{
    int res = (int)write(socket, buffer, buffer_length);
    
    //
    // assuming blocking write so that will not return until all done - unless error
    //
    // still need more error checking than this
    //
    
    if( res > 0 && (res == buffer_length) )
    {
        return true;
    }
    else if( res > 0 && (res < buffer_length) )
    {
        assert(false);
    }
    else if( res < 0 )
    {
        socket_throw_error(socket, errno, "write failed r < 0");
        *status = SOCKET_STATUS_ERROR;
        return false;
    }
    else
    {   
        socket_throw_error(socket, errno, "write failed else");
        *status = SOCKET_STATUS_ERROR;
        return false;
    }
    
}

int socket_read_data( socket_handle_t socket, void* buffer, int buffer_length, int* status)
{
    
    int howMuch = (int)read(socket, buffer, buffer_length);
    
    int saved_errno = errno;
    
    bool isError = (howMuch == -1 && saved_errno != EAGAIN && saved_errno != EINTR);
    *status = 0;
    
    if( howMuch > 0 )
    {        
        //printf("%d read got some %d \n", socket, howMuch);
        *status = SOCKET_STATUS_GOOD;
        
        
    }else if( howMuch == 0 && (! isError) )
    {
//        socket_throw_error(socket, errno, "read error");
//        shutdown(socket, SHUT_RD);
        *status = SOCKET_STATUS_EOF;
    }
    else
    {
        socket_throw_error(socket, errno, "read failed");
//        shutdown(socket, SHUT_RDWR);
        if( saved_errno == EAGAIN )
            *status = SOCKET_STATUS_EAGAIN;
        else
            *status = SOCKET_STATUS_ERROR;
    }
    return howMuch;
}

void socket_close(socket_handle_t socket)
{
    close(socket);
}

#endif