/*
 * Copyright (c) 2015, Swedish Institute of Computer Science.
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
 *         Scylla header file
 *
 * \author Hassan Iqbal <16060023@lums.edu.pk>
 */

#ifndef __SCYLLA_H__
#define __SCYLLA_H__

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-conf.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "orchestra-conf.h"


#define BLE_STACK_SUPPORT 1

/* Setting node either UDP server of UDP client*/

#define NODE_ROLE_SERVER 1

#if NODE_ROLE_SERVER
#define NODE_ROLE_CLIENT 0
#else
#define NODE_ROLE_CLIENT 1
#endif

#define CYCLES_10MS 656

#define STACK_IEEE 1
#define STACK_BLE 2

uint8_t ACTIVE_STACK;

extern rtimer_clock_t ble_master_wakeup;
extern uint16_t rx_count_for_ble;

void scylla_netstack_switch(uint8_t val); /* 1 = IEEE 802.15.4, 2 = BLE*/
void scylla_start_beacons(void); // Defined in tsch.c
void scylla_send_packet_now(void); // Defined in ble-l2cap.c
#endif /* __Scylla_H__ */
