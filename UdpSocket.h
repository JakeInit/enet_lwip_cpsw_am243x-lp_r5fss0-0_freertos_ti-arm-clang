/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __UDPSOCKET_H_
#define __UDPSOCKET_H_

#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include <kernel/dpl/DebugP.h>

#define UDP_CONN_PORT 5001        // server port to listen on/connect to

#define MAX_BUFFER_LENGTH 8192

struct BufferData
{
    char buffer[MAX_BUFFER_LENGTH];
    unsigned int length;
};

struct InetAddress
{
    char ipAddress[INET_ADDRSTRLEN];
    unsigned short port;
};

struct UdpInstance
{
    int socket;
    void (*write)(struct InetAddress* destinationAddress, struct BufferData* bufferData, int* socket);
    int (*read)(struct BufferData* bufferData, int bufferMaxSize, int* socket);
    uint8_t (*sendMessage)(struct InetAddress* destinationAddress, struct BufferData* bufferData,
            char* response, uint64_t timeout_ms, int* socket);

    void (*open)(int* socket);
    void (*close)(int* socket);
};

void print_app_header(void);

void udpSocketWrite(struct InetAddress* destinationAddress,
                    struct BufferData* bufferData, int* socket);

int udpSocketRead(struct BufferData* bufferData, int bufferMaxSize, int* socket);

uint8_t udpSocketSendMessage(struct InetAddress* destinationAddress,
                          struct BufferData* bufferData, char* response,
                          uint64_t timeout_ms, int* socket);

void udpSocketOpen(int* socket);

void udpSocketClose(int* socket);

uint8_t isConnected(int* socket);

struct UdpInstance* UdpInstance_new();

#endif /* __UDPSOCKET_H_ */
