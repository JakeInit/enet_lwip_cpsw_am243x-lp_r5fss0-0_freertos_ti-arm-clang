/*
 * Copyright (c) 2001,2002 Florian Schulze.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * test.c - This file is part of lwIP test
 *
 */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>

/* lwIP core includes */
#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/api.h"

#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"

/* lwIP netif includes */
#include "lwip/etharp.h"
#include "netif/ethernet.h"

/* applications includes */
#include "lwip/apps/netbiosns.h"
#include "lwip/apps/httpd.h"
#include "apps/httpserver/httpserver-netconn.h"
#include "apps/netio/netio.h"
#include "apps/ping/ping.h"
#include "apps/rtp/rtp.h"
#include "apps/chargen/chargen.h"
#include "apps/shell/shell.h"
#include "apps/tcpecho/tcpecho.h"
#include "apps/udpecho/udpecho.h"
#include "apps/tcpecho_raw/tcpecho_raw.h"
#include "apps/socket_examples/socket_examples.h"

#include "examples/lwiperf/lwiperf_example.h"
#include "examples/mdns/mdns_example.h"
#include "examples/snmp/snmp_example.h"
#include "examples/tftp/tftp_example.h"
#include "examples/sntp/sntp_example.h"
#include "examples/mqtt/mqtt_example.h"
#include "examples/httpd/cgi_example/cgi_example.h"
#include "examples/httpd/fs_example/fs_example.h"
#include "examples/httpd/ssi_example/ssi_example.h"

#include "default_netif.h"

#include "netif/ppp/ppp_opts.h"

/* include the port-dependent configuration */
#include "lwipcfg.h"
#include "test_enet_lwip.h"
#include "UdpSocket.h"

#ifndef LWIP_EXAMPLE_APP_ABORT
#define LWIP_EXAMPLE_APP_ABORT() 0
#endif


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* globales variables for netifs */

struct dhcp netif_dhcp; // dhcp struct for the ethernet netif
struct autoip netif_autoip; // autoip struct for the ethernet netif


static void status_callback(struct netif *state_netif)
{
  if (netif_is_up(state_netif))
  {
    DebugP_log("status_callback==UP, local interface IP is %s\r\n", ip4addr_ntoa(netif_ip4_addr(state_netif)));
  }
  else
  {
    DebugP_log("status_callback==DOWN\r\n");
  }
}

static void link_callback(struct netif *state_netif)
{
  if (netif_is_link_up(state_netif))
  {
    DebugP_log("link_callback==UP\r\n");
  }
  else
  {
    DebugP_log("link_callback==DOWN\r\n");
  }
}

/* This function initializes all network interfaces */
static void test_netif_init(void)
{
  ip4_addr_t ipaddr, netmask, gw;
  err_t err;

  ip4_addr_set_zero(&gw);
  ip4_addr_set_zero(&ipaddr);
  ip4_addr_set_zero(&netmask);

  DebugP_log("Starting lwIP, local interface IP is dhcp-enabled\r\n");

  init_default_netif(&ipaddr, &netmask, &gw);

  netif_set_status_callback(netif_default, status_callback);
  netif_set_link_callback(netif_default, link_callback);

  DebugP_log("Setting autoIp struct\n");
  autoip_set_struct(netif_default, &netif_autoip);
  DebugP_log("Setting dhcp struct\n");
  dhcp_set_struct(netif_default, &netif_dhcp);

  DebugP_log("Setting up netif\n");
  netif_set_up(netif_default);

  err = dhcp_start(netif_default);
  LWIP_ASSERT("dhcp_start failed", err == ERR_OK);
}

/* This function initializes applications */
static void apps_init(void)
{
  // tcpecho_init();
  udpecho_init();

#if LWIP_SOCKET_EXAMPLES_APP && LWIP_SOCKET
  socket_examples_init();
#endif /* LWIP_SOCKET_EXAMPLES_APP && LWIP_SOCKET */

  lwiperf_example_init();

  print_app_header();
  sys_thread_new("UDP Iperf", udpSocketOpen, NULL, DEFAULT_THREAD_STACKSIZE,
                             DEFAULT_THREAD_PRIO);
}

/* This function initializes this lwIP test. When NO_SYS=1, this is done in
 * the main_loop context (there is no other one), when NO_SYS=0, this is done
 * in the tcpip_thread context */
static void test_init(void * arg)
{

/* remove compiler warning */
  sys_sem_t *init_sem;
  LWIP_ASSERT("arg != NULL", arg != NULL);
  init_sem = (sys_sem_t*)arg;

  /* init randomizer again (seed per thread) */
  srand((unsigned int)sys_now()/1000);

  /* init network interfaces */
  test_netif_init();

  /* init apps */
  apps_init();

  sys_sem_signal(init_sem);
}

/* This is somewhat different to other ports: we have a main loop here:
 * a dedicated task that waits for packets to arrive. This would normally be
 * done from interrupt context with embedded hardware, but we don't get an
 * interrupt in windows for that :-) */
void main_loop(void * a0)
{
  err_t err;
  sys_sem_t init_sem;

  /* initialize lwIP stack, network interfaces and applications */
  err = sys_sem_new(&init_sem, 0);
  LWIP_ASSERT("failed to create init_sem", err == ERR_OK);
  LWIP_UNUSED_ARG(err);
  tcpip_init(test_init, &init_sem); // Create thread to run inserted function

  sys_sem_wait(&init_sem); // wait for initialization to finish
  sys_sem_free(&init_sem);

  /* MAIN LOOP for driver update */
  while (!LWIP_EXAMPLE_APP_ABORT())
  {
    default_netif_poll();
    sys_msleep(1);
    {
        void print_cpu_load(); // implemented in test_enet.c
        print_cpu_load();
    }
  }

  default_netif_shutdown();
}
