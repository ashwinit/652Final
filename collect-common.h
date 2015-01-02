/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         Common code between collect client and server
 * \original author
 *         Niclas Finne <nfi@sics.se>
 * \modified code author
 *         Yash Goyal <ygoyal@usc.edu>
 *         Ashwini Telang <ashwinit@usc.edu>
 *         Subhashini Sudaresan <subhashi@usc.edu>
 */ 

#ifndef __COLLECT_COMMON_H__
#define __COLLECT_COMMON_H__

#include "contiki.h"
#include "net/uip.h"
#include "net/rime/rimeaddr.h"

#define MAX_BUF_LEN   10  /* Keep it low to save the memory */
#define NUM_NODES	    12  /* Number of the nodes in the network */ 
#define NUM_PARENTS	  5   /* Maximum number of entries in the neighbor table */
#define	FIRST_NODEID  2   /* Nodes are sequential, thus the first node in the network  */
#define DBG_PRINT	    0   /* To disable the print statement */
#define SCENARIO      0   /* 0 - Scenario 2: Location based objective function using RSSI */
                          /* 1 - Scenario 1: RPL default MRHOF using ETX */

/* Region Rank */
 typedef enum {
 	ROOT,
  BUILDING,
 	LEVEL,
 	ROOM,
 	SENSOR,
  ANONYMOUS
 }identity_id;

/* Query Request Packet */
typedef struct query_pkt {
  uint8_t   type;                 // 1 - QUERY PACKET 
  uint8_t   multicast_flag;
  identity_id id;                    
  uint8_t   query_id;
}query_pkt_t;

/* Query Reply Packet */
typedef struct query_reply {
  uint16_t parent_etx;
}query_reply_t;

/* Identity Packet */
typedef struct identity_pkt {
  uint8_t 	type;                // 2 - IDENTITY PACKET
  identity_id   id;
  uint8_t	loc_addr[4];
}identity_pkt_t;

/* Routing table format */
typedef struct rt_table {
	uint8_t 		nodeid[2];
	uint8_t	 		loc_addr[4];
	unsigned int    combi_loc;
	unsigned int	loc_mask;
	uip_ipaddr_t 	ipv6_addr;
}rt_table_t;

/* Neighbor table format */
typedef struct nbr_feature_table {
	identity_id 	id;
	signed char 	rssi;
	uip_ipaddr_t 	ipaddr;
}nbr_feature_table_t;

/* Number of neighbors in the present neighbor table*/
int cur_nbr;
nbr_feature_table_t 	neighbour[NUM_PARENTS]; /* Neighbor table instance, global variable */
uint8_t region, multicast_flag;               /* Its own region and multicast flag */
void collect_common_net_init(void);
void collect_common_net_print(void);
void collect_common_set_sink(void);
void collect_common_send(void);
void query_send_sink(uint8_t b, uint8_t f, uint8_t r, uint8_t sid);
void broadcast_identity();
uint8_t is_ipaddress_exists(int i, uip_ipaddr_t* ipaddr);
void collect_common_recv(const rimeaddr_t *originator, uint8_t seqno,
                         uint8_t hops,
                         uint8_t *payload,
                         uint16_t payload_len);
void collect_common_set_send_active(int active);

PROCESS_NAME(collect_common_process);

#endif /* __COLLECT_COMMON_H__ */
