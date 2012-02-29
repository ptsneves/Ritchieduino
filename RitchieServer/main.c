/*
 * main.c
 *
 *  Created on: Jan 28, 2012
 *      Author: pneves
 */


#include "Ethernet.h"
#include "Arduino.h"


struct ComData {
	uint8_t mac[6];
	uint8_t ip[4];
	uint8_t gateway[4];
	uint8_t subnet[4];
	uint16_t port_number;
//	uint8_t buff[2];
//	uint8_t buff_size;
//	uint32_t time_of_last_activity;
//	uint32_t  allowed_connect_time;
} __attribute__((packed));


int main() {
	/*
	 * This has to come first to init timers interrupts
	 * and to disable the usart.
	 */
	init();

	struct ComData comunications = { { 0x90, 0xA2, 0xDA, 0x00, 0x24, 0xB9 },
									{ 192,168,1,177 },
						 			{192, 168, 1, 254 },
								{255, 255, 255, 0 }, 80,
			 						//{0,0}, 2, 0, 5UL * 60UL * 1000UL
			 					};
	char receive_buffer;
	uint8_t current_line_blank = 1;
	char *test_string =
			"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\nHello World\r\n\r\n";


	TCPBegin(comunications.mac, comunications.ip, comunications.gateway,
			comunications.subnet);

	TCPListen(comunications.port_number);
	for(;;) {
		while (TCPConnected()) {
			if (TCPAvailable()) {
				TCPReceive((uint8_t *)&receive_buffer,
						sizeof(receive_buffer));

				if (receive_buffer == '\n' && current_line_blank) {
					TCPSend((uint8_t *)test_string, strlen(test_string));
					TCPStop();
					TCPListen(comunications.port_number);
					break;
				}
				if (receive_buffer == '\n') {
					current_line_blank = 1;
				}
				else if (receive_buffer != '\r') {
					current_line_blank = 0;
				}
			}
		}
	}
	return 0;
}
