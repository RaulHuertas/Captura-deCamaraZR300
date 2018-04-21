#ifndef NETWORK_UTILS_HPP
#define NETWORK_UTILS_HPP

#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


void transmitirPorUDP(
        int ancho, int alto,
        int formatoPixel,
        const void* data,
        int rawDataSize,
        int min_value, int minPosX, int minPosY, float depthScale,
        int sockfd, struct sockaddr_in& serveraddr, int serverlen
);

#endif // NETWORK_UTILS_HPP
