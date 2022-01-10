#ifndef SOCKET_H
#define SOCKET_H

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <netdb.h>
#include <pthread.h>
#include <sys/select.h>
#include <arpa/inet.h>

struct socket {
	int fd;
	struct sockaddr_in info;
	char *ip;
	int port;
	int type;
	int blocking;
};

extern struct socket udp_socket;
extern pthread_mutex_t udp_socket_mutex;

int SocketClose(struct socket *sock);
int SocketSetup(struct socket *sock);
int SocketSelect(struct socket *sock, int timeout);
int SocketSend(struct socket *sock, int *data, int bytes, int timeout);

#endif
