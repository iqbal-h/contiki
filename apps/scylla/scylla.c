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
 * \file Scylla
 *
 *
 * \author  Hassan Iqbal <16060023@lums.edu.pk>
 */

#include "net/netstack.h"
#include "apps/scylla/scylla.h"


void scylla_netstack_switch(uint8_t stack){
#if  BLE_STACK_SUPPORT
	if (stack == STACK_BLE && ACTIVE_STACK != STACK_BLE) {

		NETSTACK_RADIO = NETSTACK_RADIO_BLE;
		NETSTACK_RDC = NETSTACK_RDC_BLE;
		NETSTACK_MAC = NETSTACK_MAC_BLE;
		ACTIVE_STACK = STACK_BLE;

	} else if (stack == STACK_IEEE && ACTIVE_STACK != STACK_IEEE) {

		NETSTACK_RADIO = NETSTACK_RADIO_IEEE;
		NETSTACK_RDC = NETSTACK_RDC_IEEE;
		NETSTACK_MAC = NETSTACK_MAC_IEEE;
		ACTIVE_STACK = STACK_IEEE;

	} else {
		PRINTF("Network Stack Switching Error.\n");
	}
#endif
}
