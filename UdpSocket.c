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

extern struct netif server_netif;
static struct perf_stats server;

int sock = 0;

/* labels for formats [KMG] */
const char udperf_kLabel[] = { ' ', 'K', 'M', 'G' };

/* Report interval in ms */
#define REPORT_INTERVAL_TIME (INTERIM_REPORT_INTERVAL * 1000)

void print_app_header(void)
{
	DebugP_log("UDP server listening on port %d\r\n", UDP_CONN_PORT);
}

static void print_udp_conn_stats(struct sockaddr_in from)
{
	DebugP_log("Connected to %s port %d\r\n", inet_ntoa(from.sin_addr),
				ntohs(from.sin_port));

	DebugP_log("[ ID] Interval\t     Transfer     Bandwidth\t");
	DebugP_log("    Lost/Total Datagrams\r\n");
}

static void stats_buffer(char* outString,
		double data, enum measure_t type)
{
	int conv = KCONV_UNIT;
	const char *format;
	double unit = 1024.0;

	if (type == SPEED)
		unit = 1000.0;

	while (data >= unit && conv <= KCONV_GIGA)
	{
		data /= unit;
		conv++;
	}

	/* Fit data in 4 places */
	if (data < 9.995) //9.995 rounded to 10.0
	{
		format = "%4.2f %c"; // #.##
	}
	else if (data < 99.95) // 99.95 rounded to 100
	{
		format = "%4.1f %c"; // ##.#
	}
	else
	{
		format = "%4.0f %c"; // ####
	}
	sprintf(outString, format, data, udperf_kLabel[conv]);
}

/** The report function of a TCP server session */
static void udp_conn_report(u64_t diff,
		enum report_type report_type)
{
	u64_t total_len, cnt_datagrams, cnt_dropped_datagrams, total_packets;
	u32_t cnt_out_of_order_datagrams;
	double duration, bandwidth = 0;
	char data[16], perf[16], time[64], drop[64];

	if (report_type == INTER_REPORT)
	{
		total_len = server.i_report.total_bytes;
		cnt_datagrams = server.i_report.cnt_datagrams;
		cnt_dropped_datagrams = server.i_report.cnt_dropped_datagrams;
	}
	else
	{
		server.i_report.last_report_time = 0;
		total_len = server.total_bytes;
		cnt_datagrams = server.cnt_datagrams;
		cnt_dropped_datagrams = server.cnt_dropped_datagrams;
		cnt_out_of_order_datagrams = server.cnt_out_of_order_datagrams;
	}

	total_packets = cnt_datagrams + cnt_dropped_datagrams;
	/* Converting duration from milliseconds to secs,
	 * and bandwidth to bits/sec .
	 */
	duration = diff / 1000.0; /* secs */
	if (duration)
		bandwidth = (total_len / duration) * 8.0;

	stats_buffer(data, total_len, BYTES);
	stats_buffer(perf, bandwidth, SPEED);
	/* On 32-bit platforms, xil_printf is not able to print
	 * u64_t values, so converting these values in strings and
	 * displaying results
	 */
	sprintf(time, "%4.1f-%4.1f sec",
			(double)server.i_report.last_report_time,
			(double)(server.i_report.last_report_time + duration));
	sprintf(drop, "%4llu/%5llu (%.2g%%)", cnt_dropped_datagrams,
			total_packets,
			(100.0 * cnt_dropped_datagrams)/total_packets);
	DebugP_log("[%3d] %s  %sBytes  %sbits/sec  %s\r\n\r", server.client_id,
			time, data, perf, drop);

	if (report_type == INTER_REPORT)
	{
		server.i_report.last_report_time += duration;
	}
	else if ((report_type != INTER_REPORT) && cnt_out_of_order_datagrams)
	{
		DebugP_log("[%3d] %s  %u datagrams received out-of-order\r\n\r",
				server.client_id, time,
				cnt_out_of_order_datagrams);
	}
}

static void reset_stats(void)
{
	server.client_id++;
	/* Save start time */
	server.start_time = sys_now();
	server.end_time = 0; /* ms */
	server.total_bytes = 0;
	server.cnt_datagrams = 0;
	server.cnt_dropped_datagrams = 0;
	server.cnt_out_of_order_datagrams = 0;
	server.expected_datagram_id = 0;

	/* Initialize Interim report parameters */
	server.i_report.start_time = 0;
	server.i_report.total_bytes = 0;
	server.i_report.cnt_datagrams = 0;
	server.i_report.cnt_dropped_datagrams = 0;
	server.i_report.last_report_time = 0;
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

    numBytes = recvfrom(sock, bufferData->buffer, UDP_RECV_BUFSIZE, 0,
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
    int numBytes = udpSocketRead(&readBufferData, UDP_RECV_BUFSIZE);

    if(numBytes != 0)
    {
        char* result;
        result = (char *) memchr( readBufferData.buffer, response, numBytes);
        if(result != NULL)
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
