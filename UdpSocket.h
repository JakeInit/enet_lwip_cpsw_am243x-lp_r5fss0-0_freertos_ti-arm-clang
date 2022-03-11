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

#define INTERIM_REPORT_INTERVAL 5 // seconds between periodic bandwidth reports

#define UDP_CONN_PORT 5001        // server port to listen on/connect to

#define UDP_RECV_BUFSIZE 8192     // Was 1500

/* used as indices into kLabel[] */
enum
{
    KCONV_UNIT,
    KCONV_KILO,
    KCONV_MEGA,
    KCONV_GIGA,
};

/* used as type of print */
enum measure_t
{
    BYTES,
    SPEED
};

/* Report type */
enum report_type
{
    INTER_REPORT,      // The Intermediate report
    UDP_DONE_SERVER,   // The server side test is done
    UDP_ABORTED_REMOTE // Remote side aborted the test
};

struct interim_report
{
    u64_t start_time;
    u64_t last_report_time;
    u32_t total_bytes;
    u32_t cnt_datagrams;
    u32_t cnt_dropped_datagrams;
};

struct perf_stats
{
    u8_t client_id;
    u64_t start_time;
    u64_t end_time;
    u64_t total_bytes;
    u64_t cnt_datagrams;
    u64_t cnt_dropped_datagrams;
    u32_t cnt_out_of_order_datagrams;
    s32_t expected_datagram_id;
    struct interim_report i_report;
};

struct BufferData
{
    char buffer[UDP_RECV_BUFSIZE];
    unsigned int length;
};

struct InetAddress
{
    char ipAddress[INET_ADDRSTRLEN];
    unsigned short port;
};

void print_app_header(void);
void udpSocketWrite(struct InetAddress* destinationAddress,
                    struct BufferData* bufferData);
int udpSocketRead(struct BufferData* bufferData, int bufferMaxSize);
uint8_t udpSocketSendMessage(struct InetAddress* destinationAddress,
                          struct BufferData* bufferData, char* response,
                          uint64_t timeout_ms);
void udpSocketOpen(void *arg);
void udpSocketClose();
uint8_t isConnected();

#endif /* __UDPSOCKET_H_ */
