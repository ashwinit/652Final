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
 *         Example of how the collect primitive works.
 * \author
 *         Adam Dunkels <adam@sics.se>
 * \modified code author
 *         Yash Goyal <ygoyal@usc.edu>
 *         Ashwini Telang <ashwinit@usc.edu>
 *         Subhashini Sudaresan <subhashi@usc.edu>
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "dev/serial-line.h"
#include "dev/leds.h"
#include "collect-common.h"

//#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

static unsigned long time_offset;
static int send_active = 1;

#ifndef PERIOD
#define PERIOD 30
#endif
#define RANDWAIT (PERIOD)

/*---------------------------------------------------------------------------*/
PROCESS(collect_common_process, "collect common process");
AUTOSTART_PROCESSES(&collect_common_process);
/*---------------------------------------------------------------------------*/
static unsigned long
get_time(void)
{
  return clock_seconds() + time_offset;
}
/*---------------------------------------------------------------------------*/
static unsigned long
strtolong(const char *data) {
  unsigned long value = 0;
  int i;
  for(i = 0; i < 10 && isdigit(data[i]); i++) {
    value = value * 10 + data[i] - '0';
  }
  return value;
}
/*---------------------------------------------------------------------------*/
void
collect_common_set_send_active(int active)
{
  send_active = active;
}
/*---------------------------------------------------------------------------*/
void
collect_common_recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops,
                    uint8_t *payload, uint16_t payload_len)
{
  uint16_t data;
  int i;
  get_time();
  
  for(i = 0; i < payload_len / 2; i++) {
    memcpy(&data, payload, sizeof(data));
    payload += sizeof(data);
  }
  leds_blink();
}
/*---------------------------------------------------------------------------*/
uint8_t b=0, f=0, r=0, sid=0;

PROCESS_THREAD(collect_common_process, ev, data)
{
  static struct etimer period_timer, wait_timer;
  PROCESS_BEGIN();

  collect_common_net_init();
  /* Send a packet every 60-62 seconds. */
  etimer_set(&period_timer, CLOCK_SECOND * PERIOD);
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == serial_line_event_message) {
      char *line;
      char query[10];
      char reg[2];
      line = (char *)data;
      if(strncmp(line, "collect", 7) == 0 ||
         strncmp(line, "gw", 2) == 0) {
        collect_common_set_sink();
      } else if(strncmp(line, "net", 3) == 0) {
        collect_common_net_print();
      } else if(strncmp(line, "time ", 5) == 0) {
        unsigned long tmp;
        line += 6;
        while(*line == ' ') {
          line++;
        }
        tmp = strtolong(line);
        time_offset = clock_seconds() - tmp;
      } else if(strncmp(line, "mac ", 4) == 0) {
        line +=4;
        while(*line == ' ') {
          line++;
        }
        if(*line == '0') {
          NETSTACK_RDC.off(1);
        } else {
          NETSTACK_RDC.on();
        }

      } else if(strncmp(line, "~K", 2) == 0 ||
                strncmp(line, "killall", 7) == 0) {
        /* Ignore stop commands */
      } else if(strncmp(line, "Query-",6) == 0){        
        /* To extract the region information of the requested query */
        strcpy(query, line+7);
        char *pch = NULL;

        pch = strtok (query,".");
        if(strncmp(pch, "x", 1) != 0) {
          b = atoi(pch);
          multicast_flag = 0;
        }
        else{
          b = 0;
          multicast_flag = 1;
#if SCENARIO
          goto BUILD;
#endif
        }

        pch = strtok (NULL, ".");
        if(strncmp(pch, "x", 1) != 0) {
          f = atoi(pch);
          multicast_flag = 0;
        }
        else{
          f = 0;
          multicast_flag = 1;
#if SCENARIO
          goto LEVL;
#endif
        }

        pch = strtok (NULL, ".");
        if(strncmp(pch, "x", 1) != 0) {
          r = atoi(pch);
          multicast_flag = 0;
        }
        else{
          r = 0;
          multicast_flag = 1;
#if SCENARIO
          goto ROM;
#endif
        }

        pch = strtok (NULL, ".");
        if(strncmp(pch, "x", 1) != 0) {
          sid = atoi(pch);
          multicast_flag = 0;
        }
        else{
          sid = 0;
          multicast_flag = 1;
#if SCENARIO
          goto SENSR;
#endif
        }
        goto SEND_QR;
#if SCENARIO
BUILD:  for(b=1;b<=3;b++)
         for(f=1;f<=3;f++)
          for(r=1;r<=3;r++)
            for(sid=1;sid<=3;sid++)
                    query_send_sink(b,f,r,sid);
        goto JUMP;

LEVL:      for(f=1;f<=3;f++)
              for(r=1;r<=3;r++)
                for(sid=1;sid<=3;sid++)
                    query_send_sink(b,f,r,sid);
        goto JUMP;

ROM:          for(r=1;r<=3;r++)
                for(sid=1;sid<=3;sid++)
                    query_send_sink(b,f,r,sid);
        goto JUMP;

SENSR:          for(sid=1;sid<=3;sid++)
                    query_send_sink(b,f,r,sid);
        goto JUMP;
        
#endif    
SEND_QR:                query_send_sink(b,f,r,sid);
JUMP:{}

      } else if(strncmp(line, "Region-",7) == 0){   //Region of the node
        strcpy(reg, line+8);

        if(strncmp(reg, "B", 1) == 0){
          region = BUILDING;
        }
        else if(strncmp(reg, "L", 1) == 0){
          region = LEVEL;
        }
        else if(strncmp(reg, "R", 1) == 0){
          region = ROOM;
        }
        else if(strncmp(reg, "S", 1) == 0){
          region = SENSOR;
        }
      }
      else {
        //printf("unhandled command: %s\n", line);
      }
    }
    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &period_timer) {
        etimer_reset(&period_timer);
        etimer_set(&wait_timer, random_rand() % (CLOCK_SECOND * RANDWAIT));
      } else if(data == &wait_timer) {
        if(send_active) {
          /* Time to send the data */
          /* Broadcast its own region rank */
          if( region == ROOM )          
            broadcast_identity(ROOM);
          else if ( region == BUILDING )
            broadcast_identity(BUILDING);
          else if ( region == LEVEL )
            broadcast_identity(LEVEL);
          else if ( region == SENSOR )
            broadcast_identity(SENSOR);
        }
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
