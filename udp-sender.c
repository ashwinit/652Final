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
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"

#include "dev/serial-line.h"
#include "dev/cc2420.h"
#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
#include "collect-common.h"
//#include "collect-view.h"

#include <stdio.h>
#include <string.h>

#define UDP_CLIENT_PORT 8775
#define UDP_SERVER_PORT 5688

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
static struct uip_udp_conn *client_conn, *server_conn;
static uip_ipaddr_t server_ipaddr;

static uint8_t cur_query_id = 0;
//static uint8_t prev_query_id;

uint8_t sender_id = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process, &collect_common_process);
/*---------------------------------------------------------------------------*/
void
collect_common_set_sink(void)
{
  /* A udp client can never become sink */
}
/*---------------------------------------------------------------------------*/

void
collect_common_net_print(void)
{
  
}

void
collect_common_send()
{
  /* Send the query reply back to the sink */
  rpl_parent_t *preferred_parent;
  rpl_dag_t *dag;

  query_reply_t qrpkt;
  dag = rpl_get_any_dag();
  if(dag != NULL) {
    preferred_parent = dag->preferred_parent;
  if(preferred_parent != NULL) {
    uip_ds6_nbr_t *nbr;
  nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
  if(nbr != NULL) {
    // Use parts of the IPv6 address as the parent address, in reversed byte order. 
        qrpkt.parent_etx = (uint16_t)preferred_parent->link_metric + preferred_parent->rank;
      }
    }
  }
  uip_udp_packet_sendto(client_conn, &qrpkt, sizeof(qrpkt),
      &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}

void
print_nbr_table(){
  /* print the neighbor table */
  int i = 0;
  printf("**************************\n");
  for (i = 0; i < cur_nbr; ++i){
    printf("%d: ", i);
    printf("Identity- %d; ", neighbour[i].id);
    printf("IP Addr- ");
    printf("%02x", neighbour[i].ipaddr.u8[13]);
    printf("; ");
    printf("RSSI- %d\n", neighbour[i].rssi);
  }
  printf("**************************\n");
}

uint8_t
is_ipaddress_exists(int i, uip_ipaddr_t* ipaddr){
  /* check if the IP address exist in the table */
  /* helps remove the duplicate entry in the neighbor table */
  int j = 0;
  for (j = 0; j < 16; j++){
    if(ipaddr->u8[j] != neighbour[i].ipaddr.u8[j])
      return 0; // FALSE
  }
  return 1; // TRUE
}

void
add_new_nbr(identity_id id, uip_ipaddr_t ipaddr, signed char rssi ){
  /* Add neighbor if the message region == own region -1 */
  int i=0;
  uint8_t found = 0;
  if(cur_nbr == NUM_PARENTS)
    return;
  for (i = 0; i < cur_nbr; ++i){
    if(is_ipaddress_exists(i, &ipaddr)){
      found = 1;
      neighbour[i].id = id;
      neighbour[i].rssi = rssi;
      return;
    }
  }
  if(found == 0){
    neighbour[i].id = id;
    neighbour[i].rssi = rssi;
    memcpy(&neighbour[i].ipaddr, &ipaddr, sizeof(uip_ipaddr_t));
    cur_nbr++;
  }
}

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  /* handle the incoming packet */
  if(uip_newdata()) {
    if( ((query_pkt_t*)uip_appdata)->type == 1){
      /* Query Packet */
      query_pkt_t *appdata;
      appdata = (query_pkt_t*)uip_appdata;

        if(!appdata->multicast_flag)
          collect_common_send();
        else{
          if(appdata->query_id > cur_query_id) { /* Do not reply to the duplicate query */
            cur_query_id = appdata->query_id;
            if(region == SENSOR || region == ANONYMOUS){
              collect_common_send();
            }
            else if (appdata->id < region) { //else drop the query, received from the lower region
            /* send multicast query to the lower regions */
              appdata->id = region;
              uip_ipaddr_t addr;
              uip_create_linklocal_allnodes_mcast(&addr);
              uip_udp_packet_sendto(server_conn, appdata, sizeof(query_pkt_t),
                          &addr, UIP_HTONS(UDP_CLIENT_PORT));
            }
          } 
        }
    }
    else if( ((identity_pkt_t*)uip_appdata)->type == 2){
      /* Identity Packet */
      if(region > ((identity_pkt_t*)uip_appdata)->id )
        add_new_nbr(((identity_pkt_t*)uip_appdata)->id, UIP_IP_BUF->srcipaddr, cc2420_last_rssi);
      print_nbr_table();
    }
  }
}

void 
query_send_sink(uint8_t b, uint8_t f, uint8_t r, uint8_t sid){

}

/*---------------------------------------------------------------------------*/
void 
broadcast_identity(int id)
{
  /* Broadcast its own region rank */
  uip_ipaddr_t addr;

  identity_pkt_t ipkt;
  ipkt.type = 2;
  ipkt.id = id;

  /* Will be dynamic based on the packet transmission with the parent */
  ipkt.loc_addr[0] = sender_id;
  ipkt.loc_addr[1] = 0;
  ipkt.loc_addr[2] = 0;
  ipkt.loc_addr[3] = 0;

  uip_create_linklocal_allnodes_mcast(&addr);
  uip_udp_packet_sendto(server_conn, &ipkt, sizeof(ipkt),
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
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      sender_id = uip_ds6_if.addr_list[i].ipaddr.u8[13];
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  /* set server address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();

  PRINTF("UDP client process started\n");

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

  /* One socket to act as sever */
  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
        UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));
  
  /* Starting of the node is anonymous */
  /* user with the help of serial command line argument set the region of the node */
  region = ANONYMOUS;
  /* number of neighbor in the starting is always zero */
  cur_nbr = 0;

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
