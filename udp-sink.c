/*
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
 * This file is part of the Contiki operating system.
 *
 * \modified code author
 *         Yash Goyal <ygoyal@usc.edu>
 *         Ashwini Telang <ashwinit@usc.edu>
 *         Subhashini Sudaresan <subhashi@usc.edu>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/uip.h"
#include "net/rpl/rpl.h"
#include "net/rime/rimeaddr.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/serial-line.h"
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "collect-common.h"
#include "collect-view.h"

#define DEBUG DBG_PRINT
#include "net/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

static struct uip_udp_conn *server_conn;

uint8_t unique_id = 1;
// char payload_data[5] = "Hello";
unsigned long sendtime, recvtime;
static unsigned long time_offset = 0;
rt_table_t        entry[NUM_NODES];

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process,&collect_common_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_print(void)
{
  printf("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
static unsigned long
get_time(void)
{
  /* To know the time when we send and receive the packet */
  return clock_time() + time_offset;
}

uip_ipaddr_t
get_ipv6(uint8_t b, uint8_t f, uint8_t r, uint8_t sid){
  /* Search the request query in the routing table */
  /* Maps the location address to the respective IPv6 address */
  unsigned int locaddr = (b << 12) + (f << 8) + (r << 4) + sid;
  unsigned int maskloc;
  uip_ipaddr_t ret;
  int i = 0;

#if DBG_PRINT
  printf("***********Testing**********\n");
  printf("Location Addr (B.F.R.SID): %d.%d.%d.%d, ", b,f,r,sid);
  printf("Combined Location Addr: %u\n", locaddr);
#endif

  if(b == 0){
    goto notfound;
  }

  if(f == 0 && (r != 0 || sid != 0)){
    goto notfound;
  }

  if(r == 0 && sid !=0){
    goto notfound;
  }

  for (i = NUM_NODES-1; i >= 0; i--){
      maskloc = locaddr & entry[i].combi_loc;
#if DBG_PRINT
      printf("Comparing with mask: %u, entered loc mask: %u\n", entry[i].combi_loc, maskloc);
#endif
      if(maskloc == entry[i].combi_loc){
        if(locaddr != entry[i].combi_loc)
          goto notfound;
#if DBG_PRINT
        printf("IPv6: ");
        PRINT6ADDR(&entry[i].ipv6_addr);
        printf("\n");
        printf("****************************\n"); 
#endif
        return entry[i].ipv6_addr;
      }
  }

notfound:
#if DBG_PRINT
  printf("No match found!\n");
  printf("****************************\n");
#endif
  uip_ip6addr(&ret, 0, 0, 0, 0, 0, 0, 0, 0); 
  return ret;

}
void 
collect_common_send(void)
{

}

uint8_t
is_ipaddress_exists(int i, uip_ipaddr_t* ipaddr)
{
  return 0;
}

void
query_send_sink(uint8_t b, uint8_t f, uint8_t r, uint8_t sid)
{
  /* Query is send in the network */
  uip_ipaddr_t addr;
  query_pkt_t qpkt;

  qpkt.query_id = unique_id;
  unique_id++;                              
  /* Unique Id associated with each query to reduce looping */
  qpkt.type = 1;
  qpkt.multicast_flag = multicast_flag; /* 1 - if multicast address, 0 - otherwise */
  qpkt.id = ROOT;                       /* Query is always generated from the root */
                                        /* This field is changed in case of multicast query */
                                        /* the common parent change this to its own region and 
                                           then sends the unicast query to its descendants */
  addr = get_ipv6(b,f,r,sid);
  if(addr.u16[0] == 0){
    printf("No match found, Location address incorrect!\n");
    printf("Query not sent, exiting..\n");
    return;
  }
  PRINT6ADDR(&addr);

  printf("Query Pkt, Sending to: ");
  printf("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", 
    addr.u8[0], addr.u8[1], addr.u8[2], addr.u8[3],
    addr.u8[4],addr.u8[5],addr.u8[6],addr.u8[7],
    addr.u8[8],addr.u8[9],addr.u8[10],addr.u8[11],
    addr.u8[12],addr.u8[13],addr.u8[14],addr.u8[15]);
    printf("\n");

  sendtime = get_time();
  printf("Sent time: %lu %lu\n", ((sendtime >> 16) & 0xffff), sendtime & 0xffff);
  /* Send the udp packet to the destination */
  uip_udp_packet_sendto(server_conn, &qpkt, sizeof(qpkt),   
                        &addr, UIP_HTONS(UDP_CLIENT_PORT));
}
/*---------------------------------------------------------------------------*/
void
collect_common_net_init(void)
{
#if CONTIKI_TARGET_Z1
  uart0_set_input(serial_line_input_byte);
#else
  uart1_set_input(serial_line_input_byte);
#endif
  serial_line_init();

  PRINTF("I am sink!\n");
}
/*---------------------------------------------------------------------------*/
void 
broadcast_identity(void){

}

static void
tcpip_handler(void)
{ 
  /* Handle the query reply */
  uint8_t *appdata;
  uint8_t hops;
  query_reply_t *qrpkt;

  if(uip_newdata()) {
    recvtime = get_time();
    printf("Query Reply: From- ");
    printf("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", 
      UIP_IP_BUF->srcipaddr.u8[0], UIP_IP_BUF->srcipaddr.u8[1], UIP_IP_BUF->srcipaddr.u8[2], UIP_IP_BUF->srcipaddr.u8[3],
      UIP_IP_BUF->srcipaddr.u8[4],UIP_IP_BUF->srcipaddr.u8[5],UIP_IP_BUF->srcipaddr.u8[6],UIP_IP_BUF->srcipaddr.u8[7],
      UIP_IP_BUF->srcipaddr.u8[8],UIP_IP_BUF->srcipaddr.u8[9],UIP_IP_BUF->srcipaddr.u8[10],UIP_IP_BUF->srcipaddr.u8[11],
      UIP_IP_BUF->srcipaddr.u8[12],UIP_IP_BUF->srcipaddr.u8[13],UIP_IP_BUF->srcipaddr.u8[14],UIP_IP_BUF->srcipaddr.u8[15]);
    printf("\n");
    printf("Recv time: %lu %lu\n", ((recvtime >> 16) & 0xffff), recvtime & 0xffff);
    appdata = (uint8_t *)uip_appdata;
    hops = uip_ds6_if.cur_hop_limit - UIP_IP_BUF->ttl + 1;
    printf("Hops- %d\n", hops);

    qrpkt = (query_reply_t*)(appdata);

    printf("Parent ETX: %d\n", qrpkt->parent_etx);
    printf("Latency: %lu %lu\n", (((recvtime - sendtime) >> 16) & 0xffff), (recvtime -sendtime) & 0xffff);
  }
}

/* Populate the routing table */
static void 
populate_rt_table(){

  uint16_t temp1, temp2, temp3;
  int i = 0 ;

  uint8_t loc_addrs[NUM_NODES][4] = {
                                    {1,0,0,0}, //2 //Building = 2
                                    {2,0,0,0}, //3
                                    {1,1,0,0}, //4 //Floor = 1
                                    {2,1,0,0}, //5
                                    {1,1,1,0}, //6 //Room = 1
                                    {2,1,1,0}, //7 
                                    {2,1,2,0}, //8
                                    {1,1,1,1}, //9 //SID = 2
                                    {1,1,1,2}, //10
                                    {2,1,1,1}, //11
                                    {2,1,1,2}, //12
                                    {2,1,2,1}  //13
                                    };

  uint8_t loc_masks[NUM_NODES][4] = {
                                    {0xF,0,0,0}, //2 //Building = 2
                                    {0xF,0,0,0}, //3
                                    {0xF,0xF,0,0}, //4 //Floor = 1
                                    {0xF,0xF,0,0}, //5
                                    {0xF,0xF,0xF,0}, //6 //Room = 1
                                    {0xF,0xF,0xF,0}, //7 
                                    {0xF,0xF,0xF,0}, //8
                                    {0xF,0xF,0xF,0xF}, //9 //SID = 2
                                    {0xF,0xF,0xF,0xF}, //10
                                    {0xF,0xF,0xF,0xF}, //11
                                    {0xF,0xF,0xF,0xF}, //12
                                    {0xF,0xF,0xF,0xF} //13
                                    };

  for (i = 0; i < NUM_NODES; ++i){
    entry[i].nodeid[0] = 0;
    entry[i].nodeid[1] = i + FIRST_NODEID;

    entry[i].loc_addr[0] = loc_addrs[i][0];
    entry[i].loc_addr[1] = loc_addrs[i][1];
    entry[i].loc_addr[2] = loc_addrs[i][2];
    entry[i].loc_addr[3] = loc_addrs[i][3];

    entry[i].loc_mask = (loc_masks[i][0] << 12) +
                        (loc_masks[i][1] << 8) +
                        (loc_masks[i][2] << 4) +
                        (loc_masks[i][3]);

    entry[i].combi_loc = (entry[i].loc_addr[0] << 12) +
                        (entry[i].loc_addr[1] << 8) +
                        (entry[i].loc_addr[2] << 4) +
                        (entry[i].loc_addr[3]);

    temp1 = 0x7400 + FIRST_NODEID + i;
    temp2 = i + FIRST_NODEID;
    temp3 = (temp2 << 8) + temp2;
    uip_ip6addr(&entry[i].ipv6_addr, 0xaaaa, 0, 0, 0, 0x212, temp1, temp2, temp3); 
  }

  /* Sort in descending order based on the mask */

}

static void 
print_rt_table(void){
  /* Print the routing table */
  int i = 0;
  printf("*********************************\n");
  printf("ROUTING TABLE\n");
  for (i = 0; i < NUM_NODES; ++i){
      printf("%d-- ", i);
      printf("NODEID: %d.%d; ", entry[i].nodeid[0], entry[i].nodeid[1]);
      printf("LOC ADDR B.F.R.SID: %d.%d.%d.%d ", entry[i].loc_addr[0],
                                      entry[i].loc_addr[1], entry[i].loc_addr[2],
                                      entry[i].loc_addr[3]);
      printf("(%u); ", entry[i].combi_loc);
      printf("LOC MASK: %u; ", entry[i].loc_mask);
      printf("IPv6: ");
      printf("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", 
        entry[i].ipv6_addr.u8[0], entry[i].ipv6_addr.u8[1], entry[i].ipv6_addr.u8[2], entry[i].ipv6_addr.u8[3],
        entry[i].ipv6_addr.u8[4],entry[i].ipv6_addr.u8[5],entry[i].ipv6_addr.u8[6],entry[i].ipv6_addr.u8[7],
        entry[i].ipv6_addr.u8[8],entry[i].ipv6_addr.u8[9],entry[i].ipv6_addr.u8[10],entry[i].ipv6_addr.u8[11],
        entry[i].ipv6_addr.u8[12],entry[i].ipv6_addr.u8[13],entry[i].ipv6_addr.u8[14],entry[i].ipv6_addr.u8[15]);
      printf("\n");
  }
  printf("*********************************\n");
}

/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);
  PRINTF("UDP server started\n");

 
#if UIP_CONF_ROUTER
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */

  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_RDC.off(1);

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));


  /* Populate the routing table based on the parameters */
  populate_rt_table();
  print_rt_table();
  /* Test if the location address is mapped to its right IPv6 address */
  // get_ipv6(1,0,0,0);
  // get_ipv6(2,1,0,0);
  // get_ipv6(2,1,1,0);
  // get_ipv6(1,2,1,0);
  // get_ipv6(2,1,2,2);
  // get_ipv6(1,2,1,2);
  // get_ipv6(0,1,1,2);
  // get_ipv6(1,0,1,2);
  // get_ipv6(1,1,0,2);
  // get_ipv6(1,1,1,0);

  /* Sink is always the ROOT */
  region = ROOT;
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiating global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
