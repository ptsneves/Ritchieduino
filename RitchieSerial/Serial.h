/*
 * hserial.h
 *
 *  Created on: Jul 25, 2009
 *      Author: Orlando Arias
 *     License: GPLv3
 *
 *   Based off: HardwareSerial.h
 */

#ifndef HSERIAL_H_
#define HSERIAL_H_

#include <inttypes.h>
#include <avr/io.h>
#include <string.h>

#if defined(XCSDD)
	#define SERIAL_DEBUG_LOG(message) \
		SerialWriteString(serial, message, strlen(message))
	#define SERIAL_DEBUG_LOG_NUMBER(number) \
		SerialWriteNumber(serial, return_value, 10); \
		SerialWrite(serial,'\n');
#else
	#define SERIAL_DEBUG_LOG(message)
	#define SERIAL_DEBUG_LOG_NUMBER(number)
#endif


struct RingBuffer;

/*
 * Hardware Serial Registers
 */
struct Serial {
	struct RingBuffer *rx_bufferN;
	volatile uint8_t *ubrrh;
	volatile uint8_t *ubrrl;
	volatile uint8_t *ucsra;
	volatile uint8_t *ucsrb;
	volatile uint8_t *udr;
	uint8_t rxen;
	uint8_t txen;
	uint8_t rxcie;
	uint8_t udre;
	uint8_t u2x;
};

void SerialInit(struct Serial * const serial, struct RingBuffer *rx_bufferN,
		volatile uint8_t *ubrrh, volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
		volatile uint8_t *ucsrb, volatile uint8_t *udr, uint8_t rxen,
		uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x);
void SerialBegin(struct Serial * const serial, long baud);
void SerialWrite(struct Serial * const serial, uint8_t c);
void SerialWriteString(struct Serial * const serial, const char *string, uint16_t size);
int SerialAvailable(struct Serial * const serial);
int SerialRead(struct Serial * const serial);
void SerialEnd(struct Serial * const serial);
void SerialWriteNumber(struct Serial * const serial,
		unsigned long n, uint8_t base);

/*
 * Depending on the existence of various serial usarts
 * they will be available.
 */

#if defined(UBRRH) && defined(UBRRL)
	void Serial(struct Serial * const serial);
#elif defined(UBRR0H) && defined(UBRR0L)
	void Serial(struct Serial * const serial);
#elif defined(USBCON)
	#include "usb_api.h"
	#warning no serial port defined  (port 0)
#else
 	#error no serial port defined  (port 0)
#endif


#if defined(UBRR1H)
	void Serial1(struct Serial * const serial);
#endif
#if defined(UBRR2H)
	void Serial2(struct Serial * const serial);
#endif
#if defined(UBRR3H)
	void Serial3(struct Serial * const serial)
#endif



#endif /* HSERIAL_H_ */
