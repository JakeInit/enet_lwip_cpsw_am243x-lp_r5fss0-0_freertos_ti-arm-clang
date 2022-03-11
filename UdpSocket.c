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

static int sock = 0;

void print_app_header(void)
{
	DebugP_log("UDP server listening on port %d\r\n", UDP_CONN_PORT);
}

static void print_udp_conn_stats(struct sockaddr_in from)
{
	DebugP_log("Connected to %s port %d\r\n", inet_ntoa(from.sin_addr),
				ntohs(from.sin_port));
}

void udpSocketWrite(struct InetAddress* destinationAddress,
                    struct BufferData* bufferData)
{
    struct sockaddr_in to;
    to.sin_family = AF_INET;
    inet_pton(AF_INET, destinationAddress->ipAddress, &to.sin_addr.s_addr);

    to.sin_port = htons(destinationAddress->port);

    if(sendto(sock, bufferData->buffer, bufferData->length , 0,
           (struct sockaddr*)&to, sizeof(struct sockaddr)))
    {
        DebugP_log("Error in write\r\n\r");
    }
}

int udpSocketRead(struct BufferData* bufferData, int bufferMaxSize)
{
    if(!isConnected())
    {
        return 0;
    }

    int numBytes = 0;
	struct sockaddr_in addr;
	socklen_t fromlen = sizeof(addr);

    numBytes = recvfrom(sock, bufferData->buffer, MAX_BUFFER_LENGTH, 0,
                             (struct sockaddr *)&addr, &fromlen);
    if(numBytes <= 0)
    {
        return 0; // Received 0 bytes of data
    }

    return numBytes;
}

uint8_t udpSocketSendMessage(struct InetAddress* destinationAddress,
                          struct BufferData* bufferData, char* response,
                          uint64_t timeout_ms)
{
    if(!isConnected())
    {
        return 0;
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
        strncpy(responseBuffer, readBufferData.buffer, numBytes);
        result = strpbrk(responseBuffer, response);
        if(result)
        {
            return 1;
        }
    }

    return 0;
}

void udpSocketOpen(void *arg)
{
	err_t err;
	struct sockaddr_in addr;

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		DebugP_log("UDP server: Error creating Socket\r\r\n");
		return;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(UDP_CONN_PORT);     // srcPort
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	err = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (err != ERR_OK)
	{
		DebugP_log("UDP server: Error on bind: %d\r\r\n", err);
		udpSocketClose();
		return;
	}
}

void udpSocketClose()
{
    close(sock);
    sock = 0;
}

uint8_t isConnected()
{
    if(sock == 0)
    {
        return 0;
    }

    return 1;
}