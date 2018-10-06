/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 *         A RPL+TSCH node able to act as either a simple node (6ln),
 *         DAG Root (6dr) or DAG Root with security (6dr-sec)
 *         Press use button at startup to configure.
 *
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "node-id.h"
#include "net/rpl/rpl.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/mac/tsch/tsch.h"
#include "net/rpl/rpl-private.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "cpu/cc26xx-cc13xx/dev/ble-hal.h"
#include "/home/user/contiki/apps/scylla/scylla.h"
#include "contiki-lib.h"
#include "net/ip/uip.h"
#include "net/netstack.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#if WITH_ORCHESTRA
#include "orchestra.h"
#endif /* WITH_ORCHESTRA */

#define DEBUG 1//DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define CONFIG_VIA_BUTTON 0//PLATFORM_HAS_BUTTON
#if CONFIG_VIA_BUTTON
#include "button-sensor.h"
#endif /* CONFIG_VIA_BUTTON */

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "RPL Node");
#if CONFIG_VIA_BUTTON
AUTOSTART_PROCESSES(&node_process, &sensors_process);
#else /* CONFIG_VIA_BUTTON */
AUTOSTART_PROCESSES(&node_process);
#endif /* CONFIG_VIA_BUTTON */

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define UDP_EXAMPLE_ID  190
//Sever
static struct uip_udp_conn *server_conn;
//Client
#if NODE_ROLE_CLIENT
static struct etimer periodic;
static int send_gap=0;
static struct ctimer backoff_timer;
static struct etimer lat_print_timer;
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static uint32_t seq_id = 120500; //For client 2. 0 for Client 1
static int reply;
#endif

uint8_t minute_counter = 1;
uint16_t rx_count_for_ble=0;
#ifndef PERIOD
#define PERIOD 10
#endif

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME			(CLOCK_SECOND)
#define MAX_PAYLOAD_LEN		30


/*---------------------------------------------------------------------------*/
static void print_network_status(void) {
	int i;
	uint8_t state;
	uip_ds6_defrt_t *default_route;
#if RPL_WITH_STORING
	uip_ds6_route_t *route;
#endif /* RPL_WITH_STORING */
#if RPL_WITH_NON_STORING
	rpl_ns_node_t *link;
#endif /* RPL_WITH_NON_STORING */


	PRINTF("Network Status Minute: %d \n");
	minute_counter++;
	/* Our IPv6 addresses */
	PRINTF("- Server IPv6 addresses:\n\r");
	for (i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if (uip_ds6_if.addr_list[i].isused
				&& (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINTF("-- ");PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);PRINTF("\n\r");
		}
	}

	/* Our default route */
	PRINTF("- Default route:\n\r");
	default_route = uip_ds6_defrt_lookup(uip_ds6_defrt_choose());
	if (default_route != NULL) {
		PRINTF("-- ");PRINT6ADDR(&default_route->ipaddr);PRINTF(" (lifetime: %lu seconds)\n\r", (unsigned long)default_route->lifetime.interval);
	} else {
		PRINTF("-- None\n\r");
	}

#if RPL_WITH_STORING
	/* Our routing entries */
	PRINTF("- Routing entries (%u in total):\n\r", uip_ds6_route_num_routes());


	route = uip_ds6_route_head();
	while (route != NULL) {
		PRINTF("-- ");PRINT6ADDR(&route->ipaddr);PRINTF(" via ");PRINT6ADDR(uip_ds6_route_nexthop(route));PRINTF(" (lifetime: %lu seconds)\n\r", (unsigned long)route->state.lifetime);
		route = uip_ds6_route_next(route);
	}

#endif

#if RPL_WITH_NON_STORING
	/* Our routing links */
	PRINTF("- Routing links (%u in total):\n\r", rpl_ns_num_nodes());
	link = rpl_ns_node_head();
	while(link != NULL) {
		uip_ipaddr_t child_ipaddr;
		uip_ipaddr_t parent_ipaddr;
		rpl_ns_get_node_global_addr(&child_ipaddr, link);
		rpl_ns_get_node_global_addr(&parent_ipaddr, link->parent);
		PRINTF("-- ");
		PRINT6ADDR(&child_ipaddr);
		if(link->parent == NULL) {
			memset(&parent_ipaddr, 0, sizeof(parent_ipaddr));
			PRINTF(" --- DODAG root ");
		} else {
			PRINTF(" to ");
			PRINT6ADDR(&parent_ipaddr);
		}
		PRINTF(" (lifetime: %lu seconds)\n\r", (unsigned long)link->lifetime);
		link = rpl_ns_node_next(link);
	}
#endif

	PRINTF("----------------------\n\r");

}
/*---------------------------------------------------------------------------*/
static void net_init(uip_ipaddr_t *br_prefix) {
	uip_ipaddr_t global_ipaddr;

	if (br_prefix) { /* We are RPL root. Will be set automatically
	 as TSCH pan coordinator via the tsch-rpl module */
		memcpy(&global_ipaddr, br_prefix, 16);
		uip_ds6_set_addr_iid(&global_ipaddr, &uip_lladdr);
		uip_ds6_addr_add(&global_ipaddr, 0, ADDR_AUTOCONF);
		rpl_set_root(RPL_DEFAULT_INSTANCE, &global_ipaddr);
		rpl_set_prefix(rpl_get_any_dag(), br_prefix, 64);
		rpl_repair_root(RPL_DEFAULT_INSTANCE);
	}

	NETSTACK_MAC.on();
}

/*---------------------------------------------------------------------------*/
static void tcpip_handler_server(void) {
	char *appdata;
	uint8_t i;
	if (uip_newdata()) {
		appdata = (char *) uip_appdata;
		appdata[uip_datalen()] = 0;
		printf("* %s %s\n",appdata,&appdata);
		if(array_ind + strlen(appdata)+1 < sizeof(ble_temp_buf)){
			for(i=0; i < 30; i++){ // 30 indicates the amount of bytes to be copied from the received packet
				ble_temp_buf[array_ind] = appdata[i];
				array_ind++;
			}
			ble_temp_buf[array_ind] = ' ';
			array_ind++;
			rx_count_for_ble++;
		}


#if BLE_STACK_SUPPORT
		if (array_ind +30  <= BLE_TEMP_BUF_SIZE) {
			memcpy(&ble_temp_buf[array_ind], appdata, strlen(appdata));
			array_ind += strlen(appdata);

		}
#endif

#if SERVER_REPLY
		PRINTF("DATA sending reply\n\r");
		uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
		uip_udp_packet_send(server_conn, "Reply", sizeof("Reply"));
		uip_create_unspecified(&server_conn->ripaddr);
#endif
	}
}


/*---------------------------------------------------------------------------*/
#if NODE_ROLE_CLIENT
static void tcpip_handler_client(void) {
	char *str;

	if (uip_newdata()) {
		str = uip_appdata;
		str[uip_datalen()] = '\0';
		reply++;
		printf("\n> %s %ld %d\n\r", str, seq_id, reply);
	}
}

/*---------------------------------------------------------------------------*/
static void send_packet(void *ptr) {
	char buf[MAX_PAYLOAD_LEN];

#ifdef SERVER_REPLY
	uint8_t num_used = 0;
	uip_ds6_nbr_t *nbr;

	nbr = nbr_table_head(ds6_neighbors);
	while(nbr != NULL) {
		nbr = nbr_table_next(ds6_neighbors, nbr);
		num_used++;
	}

	if(seq_id > 0) {

		ANNOTATE("#A r=%d/%d,color=%s,n=%d %d\n\r", reply, seq_id,
				reply == seq_id ? "GREEN" : "RED", uip_ds6_route_num_routes(), num_used);
	}
#endif /* SERVER_REPLY */

	seq_id++;
	sprintf(buf, "%lu 111222222223 ", seq_id); //Randome Data to send
	uip_udp_packet_sendto(client_conn, buf, strlen(buf), &server_ipaddr,
			UIP_HTONS(UDP_SERVER_PORT));

	ctimer_set(&backoff_timer, SEND_TIME*5, send_packet,NULL);

}

/*---------------------------------------------------------------------------*/
static void print_local_addresses_client(void) {
	int i;
	uint8_t state;

	PRINTF("NODE: Client IPv6 addresses: ");
	for (i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if (uip_ds6_if.addr_list[i].isused
				&& (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);PRINTF("\n\r");
			/* hack to make address "final" */
			if (state == ADDR_TENTATIVE) {
				uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
			}
		}
	}
}
#endif
/*---------------------------------------------------------------------------*/
static void print_local_addresses_server(void) {
	int i;
	uint8_t state;

	PRINTF("NODE: Server IPv6 addresses: ");
	for (i = 0; i < UIP_DS6_ADDR_NB; i++) {
		state = uip_ds6_if.addr_list[i].state;
		if (state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
			PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);PRINTF("\n\r");
			/* hack to make address "final" */
			if (state == ADDR_TENTATIVE) {
				uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
			}
		}
	}
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data) {
	static struct etimer et;

	PROCESS_BEGIN()
		;

		/* 3 possible roles:
		 * - role_6ln: simple node, will join any network, secured or not
		 * - role_6dr: DAG root, will advertise (unsecured) beacons
		 * - role_6dr_sec: DAG root, will advertise secured beacons
		 * */
		static int is_coordinator = 0;
		static enum {
			role_6ln, role_6dr, role_6dr_sec
		} node_role;
		node_role = role_6ln;

		int coordinator_candidate = 0;
		// Scylla
		memset(ble_temp_buf,' ',sizeof(ble_temp_buf));

#ifdef CONTIKI_TARGET_Z1
		/* Set node with MAC address c1:0c:00:00:00:00:01 as coordinator,
		 * convenient in cooja for regression tests using z1 nodes
		 * */
		extern unsigned char node_mac[8];
		unsigned char coordinator_mac[8] = {0xc1, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

		coordinator_candidate = (memcmp(node_mac, coordinator_mac, 8) == 0);
#elif CONTIKI_TARGET_COOJA
		coordinator_candidate = (node_id == 1);
#endif

		if (coordinator_candidate) {
			if (LLSEC802154_ENABLED) {
				node_role = role_6dr_sec;
			} else {
				node_role = role_6dr;
			}
		} else {
			node_role = role_6ln;
		}

		if (NODE_ROLE_SERVER)
			node_role = role_6dr;
		else
			node_role = role_6ln;

#if CONFIG_VIA_BUTTON
		{
#define CONFIG_WAIT_TIME 5

			SENSORS_ACTIVATE(button_sensor);
			etimer_set(&et, CLOCK_SECOND * CONFIG_WAIT_TIME);

			while (!etimer_expired(&et)) {
				printf("Init: current role: %s. Will start in %u seconds. Press user button to toggle mode.\n\r",
						node_role == role_6ln ? "6ln" :
						(node_role == role_6dr) ? "6dr" : "6dr-sec",
						CONFIG_WAIT_TIME);
				PROCESS_WAIT_EVENT_UNTIL(((ev == sensors_event) && (data == &button_sensor) && button_sensor.value(0) > 0) || etimer_expired(&et));
				if (ev == sensors_event && data == &button_sensor
						&& button_sensor.value(0) > 0) {
					node_role = (node_role + 1) % 3;
					if (LLSEC802154_ENABLED == 0 && node_role == role_6dr_sec) {
						node_role = (node_role + 1) % 3;
					}
					etimer_restart(&et);
				}
			}
		}

#endif /* CONFIG_VIA_BUTTON */
		//node_role = role_6dr; // Hassan
		printf("Init: node starting with role %s\n\r",
				node_role == role_6ln ? "6ln" :
				(node_role == role_6dr) ? "6dr" : "6dr-sec");

		tsch_set_pan_secured(
				LLSEC802154_ENABLED && (node_role == role_6dr_sec));
		is_coordinator = node_role > role_6ln;

		if (is_coordinator) {
			uip_ipaddr_t prefix;
			uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
			net_init(&prefix);
		} else {
			net_init(NULL);
		}

#if WITH_ORCHESTRA
		orchestra_init();
#endif /* WITH_ORCHESTRA */

// ********************************* UDP SERVER **************************************
#if NODE_ROLE_SERVER
		uip_ipaddr_t ipaddr;
		struct uip_ds6_addr *root_if;

#if UIP_CONF_ROUTER
#if 0
		/* Mode 1 - 64 bits inline */
		uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 1
		/* Mode 2 - 16 bits inline */
		uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00,
				1);
#else
		/* Mode 3 - derived from link local (MAC) address */
		uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
		uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif

		uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
		root_if = uip_ds6_addr_lookup(&ipaddr);
		if (root_if != NULL) {
			rpl_dag_t *dag;
			dag = rpl_set_root(RPL_DEFAULT_INSTANCE, (uip_ip6addr_t *) &ipaddr);
			uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
			rpl_set_prefix(dag, &ipaddr, 64);
			PRINTF("created a new RPL dag\n\r");
		} else {
			PRINTF("failed to create a new RPL DAG\n\r");
		}
#endif /* UIP_CONF_ROUTER */

		print_local_addresses_server();

		/* The data sink runs with a 100% duty cycle in order to ensure high
		 packet reception rates. */
		//NETSTACK_MAC.off(1);
		server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
		if (server_conn == NULL) {
			PRINTF("NODE: No UDP connection available, exiting the process!\n\r");
			PROCESS_EXIT();
		}
		udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

		PRINTF("NODE: Created a server connection with remote address ");
		PRINT6ADDR(&server_conn->ripaddr);
		PRINTF(" local/remote port %u/%u\n\r", UIP_HTONS(server_conn->lport),
				UIP_HTONS(server_conn->rport));

#else // ********************************* UDP CLIENT **************************************


#if NODE_ROLE_CLIENT

		uip_ipaddr_t ipaddr;

		uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
		uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
		uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

#if 0
		/* Mode 1 - 64 bits inline */
		uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
#elif 1
		/* Mode 2 - 16 bits inline */
		uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
		/* Mode 3 - derived from server link-local (MAC) address */
		uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
#endif //NODE_ROLE_CLIENT
		PRINTF("NODE: UDP client process started nbr:%d routes:%d\n\r",
				NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);

		print_local_addresses_client();

		/* new connection with remote host */
		client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
		if(client_conn == NULL) {
			PRINTF("NODE: No UDP connection available, exiting the process!\n\r");
			PROCESS_EXIT();
		}
		udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

		PRINTF("NODE: Created a connection with the server ");
		PRINT6ADDR(&client_conn->ripaddr);
		PRINTF(" local/remote port %u/%u\n\r",
				UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

		etimer_set(&periodic, SEND_INTERVAL);

#endif //NODE_ROLE_SERVER
		/* Print out routing tables every minute */
		etimer_set(&et, CLOCK_SECOND * 60);
		while (1) {

			if ((ev == PROCESS_EVENT_TIMER) && (data == &et)) {
				print_network_status();
				etimer_reset(&et);
			}


#if NODE_ROLE_SERVER
			if (ev == tcpip_event)
				tcpip_handler_server();
#else
			if(ev == tcpip_event) {
				tcpip_handler_client();
			}

			if (send_gap < 8) {
				if (etimer_expired(&periodic)) {
					etimer_reset(&periodic);
					send_gap++;
					if (send_gap == 8 && tsch_is_associated){
						ctimer_set(&backoff_timer, SEND_TIME, send_packet,NULL);
					}else if (send_gap == 8){
						send_gap=4;
					}
				}
			}

#endif //NODE_ROLE_SERVER

			PROCESS_YIELD();
		}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
