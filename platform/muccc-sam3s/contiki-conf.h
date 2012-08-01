/*
 * Copyright (c) 2010, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 * $Id: contiki-conf.h,v 1.2 2010/10/27 14:05:24 salvopitru Exp $
 */
/*---------------------------------------------------------------------------*/
/**
* \file
*			contiki-conf.h for MB851.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*			Chi-Anh La <la@imag.fr>
*     Simon Duquennoy <simonduq@sics.se>
*/
/*---------------------------------------------------------------------------*/

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include "platform-conf.h"

#if RF230BB || RF212BB
#undef PACKETBUF_CONF_HDR_SIZE                  //Use the packetbuf default for header size
#else
#define PACKETBUF_CONF_HDR_SIZE    0            //RF230 combined driver/mac handles headers internally
#endif /*RF230BB || RF212BB*/

/* Radio and 802.15.4 params */
/* 802.15.4 radio channel */
#define RF_CHANNEL                              0
/* 802.15.4 PAN ID */
#define IEEE802154_CONF_PANID                   0xCCCC
/* Use EID 64, enable hardware autoack and address filtering */
#define RIMEADDR_CONF_SIZE                      8
#define UIP_CONF_LL_802154                      1
/* Number of buffers for incoming frames */
#define RADIO_RXBUFS                            2
/* Set to 0 for non ethernet links */
#define UIP_CONF_LLH_LEN                        0

/* RDC params */
/* TX routine passes the cca/ack result in the return parameter */
#define RDC_CONF_HARDWARE_ACK                   1
/* TX routine does automatic cca and optional backoff */
#define RDC_CONF_HARDWARE_CSMA                  0
/* RDC debug with LED */
#define RDC_CONF_DEBUG_LED                      1
/* Channel check rate (per second) */
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE    8
/* Use ACK for optimization (LPP, XMAC) */
#define WITH_ACK_OPTIMIZATION                   0

/* Netstack config */
//#define NETSTACK_CONF_MAC                       csma_driver
#define NETSTACK_CONF_MAC                       nullmac_driver
#define NETSTACK_CONF_RDC                       sicslowmac_driver
//#define NETSTACK_CONF_RDC                       nullrdc_driver
#define NETSTACK_CONF_FRAMER                    framer_802154
#define NETSTACK_CONF_RADIO                     rf212_driver

/* ContikiMAC config */
#define CONTIKIMAC_CONF_COMPOWER                1
#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT    0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS           0

/* CXMAC config */
#define CXMAC_CONF_ANNOUNCEMENTS                0
#define CXMAC_CONF_COMPOWER                     1

/* XMAC config */
#define XMAC_CONF_ANNOUNCEMENTS                 0
#define XMAC_CONF_COMPOWER                      1

/* Other (RAM saving) */
#define ENERGEST_CONF_ON                        0
#define QUEUEBUF_CONF_NUM                       8
#define QUEUEBUF_CONF_REF_NUM                   2
//#define UIP_CONF_DS6_NBR_NBU                    4
//#define UIP_CONF_DS6_ROUTE_NBU                  4
#define RPL_CONF_MAX_PARENTS_PER_DAG            4
#define RPL_CONF_MAX_INSTANCES                  1
#define RPL_CONF_MAX_DAG_PER_INSTANCE           1
#define PROCESS_CONF_NUMEVENTS                  16

#if WITH_UIP6

/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD   63
#define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER  0

#define UIP_CONF_ROUTER				                  0
#define UIP_CONF_IPV6_RPL			                  0
#define UIP_CONF_ND6_SEND_RA			              0

#define UIP_CONF_IPV6				                    1
#define UIP_CONF_IPV6_QUEUE_PKT			            1
#define UIP_CONF_IPV6_CHECKS			              1
#define UIP_CONF_IPV6_REASSEMBLY		            0
#define UIP_CONF_ND6_MAX_PREFIXES		            2
#define UIP_CONF_ND6_MAX_NEIGHBORS		          2
#define UIP_CONF_ND6_MAX_DEFROUTERS		          1
#define UIP_CONF_IP_FORWARD			                0
#define UIP_CONF_BUFFER_SIZE			              140
#define UIP_CONF_MAX_CONNECTIONS		            4
#define UIP_CONF_MAX_LISTENPORTS		            8
#define UIP_CONF_UDP_CONNS			                4

#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#endif /* SICSLOWPAN_CONF_FRAG */
//#ifndef SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS
//#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
//#endif /* SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS */
#ifndef SICSLOWPAN_CONF_MAXAGE
#define SICSLOWPAN_CONF_MAXAGE                  3
#endif /* SICSLOWPAN_CONF_MAXAGE */

#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */
#define NETSTACK_CONF_NETWORK rime_driver

#endif /* WITH_UIP6 */


#if UIP_CONF_IPV6
#define UIP_CONF_ICMP6            1
#define UIP_CONF_UDP              1
#define UIP_CONF_TCP              1
#endif /* UIP_CONF_IPV6 */

/* See uip-ds6.h */
#define UIP_CONF_DS6_NBR_NBU      20
#define UIP_CONF_DS6_DEFRT_NBU    2
#define UIP_CONF_DS6_PREFIX_NBU   3
#define UIP_CONF_DS6_ROUTE_NBU    20
#define UIP_CONF_DS6_ADDR_NBU     3
#define UIP_CONF_DS6_MADDR_NBU    0
#define UIP_CONF_DS6_AADDR_NBU    0

/* 10 bytes per stateful address context - see sicslowpan.c */
/* Default is 1 context with prefix aaaa::/64 */
/* These must agree with all the other nodes or there will be a failure to communicate! */
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 1
#define SICSLOWPAN_CONF_ADDR_CONTEXT_0 {addr_contexts[0].prefix[0]=0xaa;addr_contexts[0].prefix[1]=0xaa;}
#define SICSLOWPAN_CONF_ADDR_CONTEXT_1 {addr_contexts[1].prefix[0]=0xbb;addr_contexts[1].prefix[1]=0xbb;}
#define SICSLOWPAN_CONF_ADDR_CONTEXT_2 {addr_contexts[2].prefix[0]=0x20;addr_contexts[2].prefix[1]=0x01;addr_contexts[2].prefix[2]=0x49;addr_contexts[2].prefix[3]=0x78,addr_contexts[2].prefix[4]=0x1d;addr_contexts[2].prefix[5]=0xb1;}

/* 211 bytes per queue buffer */

/* Take the default TCP maximum segment size for efficiency and simpler wireshark captures */
/* Use this to prevent 6LowPAN fragmentation (whether or not fragmentation is enabled) */
//#define UIP_CONF_TCP_MSS      48

/* 30 bytes per TCP connection */
/* 6LoWPAN does not do well with concurrent TCP streams, as new browser GETs collide with packets coming */
/* from previous GETs, causing decreased throughput, retransmissions, and timeouts. Increase to study this. */

#define UIP_CONF_FWCACHE_SIZE    0


#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_TCP_SPLIT       1
#define UIP_CONF_DHCP_LIGHT      1


#define CHANNEL_802_15_4          0
/* AUTOACK receive mode gives better rssi measurements, even if ACK is never requested */
#define RF230_CONF_AUTOACK        1
#define RF212_CONF_AUTOACK        1
/* Request 802.15.4 ACK on all packets sent (else autoretry). This is primarily for testing. */
#define SICSLOWPAN_CONF_ACK_ALL   1
/* Number of auto retry attempts 0-15 (0 implies don't use extended TX_ARET_ON mode with CCA) */
#define RF230_CONF_AUTORETRIES    2
#define RF212_CONF_AUTORETRIES    2
/* How long to wait before terminating an idle TCP connection. Smaller to allow faster sleep. Default is 120 seconds */
#define UIP_CONF_WAIT_TIMEOUT     5


/* Logging adds 200 bytes to program size */
#define LOG_CONF_ENABLED         1


#endif /* __CONTIKI_CONF_H__ */
