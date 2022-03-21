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

/** Connection handle for a UDP Server session */

#include "UdpSocket.h"

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>

int socket = 0;

int getSocket()
{
    return socket;
}

static void print_udp_conn_stats(struct sockaddr_in from)
{
	DebugP_log("Connected to %s port %d\r\n", inet_ntoa(from.sin_addr),
				ntohs(from.sin_port));
}

void udpSocketWrite(struct InetAddress* destinationAddress, struct BufferData* bufferData)
{
    struct sockaddr_in to;
    to.sin_family = AF_INET;
    inet_pton(AF_INET, destinationAddress->ipAddress, &to.sin_addr.s_addr);

    to.sin_port = htons(destinationAddress->port);

    if(sendto(socket, bufferData->buffer, bufferData->length , 0,
           (struct sockaddr*)&to, sizeof(struct sockaddr)))
    {
        DebugP_log("Error in write\r\n\r");
    }
}

int udpSocketRead(struct BufferData* bufferData, int bufferMaxSize)
{
    if(!socketIsOpen())
    {
        return 0;
    }

    int numBytes = 0;
	struct sockaddr_in addr;
	socklen_t fromlen = sizeof(addr);

    numBytes = recvfrom(socket, bufferData->buffer, bufferData->length, 0,
                             (struct sockaddr *)&addr, &fromlen);
    if(numBytes <= 0)
    {
        return 0; // Received 0 bytes of data
    }

    return numBytes;
}

bool udpSocketSendMessage(struct InetAddress* destinationAddress,
                          struct BufferData* bufferData, char* response,
                          uint64_t timeout_ms)
{
    if(!socketIsOpen())
    {
        return false;
    }

    uint64_t startTime_us = ClockP_getTimeUsec();
    udpSocketWrite(destinationAddress, bufferData);
    uint64_t timePast_us = 0;

    uint64_t timeout_us = timeout_ms * 1000;
    while((ClockP_getTimeUsec() - startTime_us) < timeout_us)
    {
        uint64_t totalTimePast_us = ClockP_getTimeUsec() - startTime_us;
        if(totalTimePast_us != timePast_us)
        {
            timePast_us = totalTimePast_us;
            udpSocketWrite(destinationAddress, bufferData);
        }
    }

    struct BufferData readBufferData;
    int numBytes = udpSocketRead(&readBufferData, MAX_BUFFER_LENGTH);

    if(numBytes != 0)
    {
        char* result;
        char responseBuffer[numBytes];
        strncpy(responseBuffer, readBufferData.buffer, numBytes); // copy contents of buffer into responseBuffer
        result = strpbrk(responseBuffer, response);
        if(result)
        {
            return true;
        }
    }

    return false;
}

bool udpSocketOpen(uint16_t srcPort)
{
	err_t err;
	struct sockaddr_in addr;

	socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket <= 0)
	{
		DebugP_log("UDP server: Error creating Socket\r\r\n");
		return false;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(srcPort);     // UDP_CONN_PORT
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	err = bind(socket, (struct sockaddr *)&addr, sizeof(addr));
	if (err != ERR_OK)
	{
		DebugP_log("UDP server: Error on bind: %d\r\r\n", err);
		udpSocketClose();
		return false;
	}

	return true;
}

void udpSocketClose()
{
    udpCloseThisSocket(&socket);
}

void udpCloseThisSocket(int* socket)
{
    if(socketIsOpen())
    {
        close(*socket);
        socket = 0;
    }
}

bool socketIsOpen()
{
    if(socket == 0)
    {
        return false;
    }

    return true;
}
