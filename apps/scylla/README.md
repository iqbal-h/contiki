# SCYLLA

## Overview

Scylla is software control feature which enables dynamic reconfiguration of network
stacks in contiki

## Requirements

Scylla requires a system running TSCH, RPL, Orchestra.
Devices need to support both IEEE 802.15.4 and BLE
Current implementation is for TI SensorTag CC2650

## Getting Started

To use Scylla, at least 3 Sensortag nodes are required.
Node 1: Scylla gateway
Node 2: BLE slave
Node 3: TSCH child node


## Configuration

#Scylla Server
To configure this, set the following parameters in Scylla.h 

NODE_ROLE_SERVER 1
BLE_STACK_SUPPORT 1 (set 0 to run standard TSCH functionality without BLE)
	Here, the role of BLE must be MASTER.

#BLE Slave
Follow the configuration of Spoerk. The role BLE must be SLAVE.

#TSCH Child node
NODE_ROLE_SERVER 0
BLE_STACK_SUPPORT 0


###********************************************************************###


## New Files created
/contiki/apps/scylla/scylla.h
/contiki/apps/scylla/scylla.c

## Files modified
/contiki/examples/ipv6/rpl-tsch/node.c
/contiki/core/net/mac/tsch/tsch-slot-operation.c

/contiki/cpu/cc26xx-cc13xx/rf-core/ble-hal/ble-hal-cc26xx.c
/contiki/cpu/cc26xx-cc13xx/net/l2cap.c
/contiki/cpu/cc26xx-cc13xx/dev/ble-hal.h

/contiki/core/net/netstack.h
/contiki/core/net/netstack.c

