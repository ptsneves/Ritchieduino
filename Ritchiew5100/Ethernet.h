/*
 * Ethernet.h
 *
 *  Created on: Feb 25, 2012
 *      Author: pneves
 */

#ifndef ETHERNET_H_
#define ETHERNET_H_

#include "w5100.h"

/*
 * Ethernet.c
 *
 *  Created on: Feb 25, 2012
 *      Author: pneves
 */


void TCPBegin(uint8_t *mac, uint8_t *ip, uint8_t *gateway, uint8_t *subnet);

uint8_t TCPListen(uint16_t port);

uint8_t TCPStatus();

uint16_t TCPAvailable();

uint8_t TCPConnected();

uint8_t TCPEstablished();

void TCPStop();

uint16_t TCPReceive(uint8_t *buffer, size_t length);

uint16_t TCPSend(const uint8_t * buffer, size_t length);





#endif /* ETHERNET_H_ */
