#ifndef HSERIAL_H_
#define HSERIAL_H_

#include <inttypes.h>
#include <avr/io.h>
#include <string.h>


#define SERIAL_DEBUG_LOG(serial_, message) \
	SerialWriteString(serial_, message, strlen(message))
#define SERIAL_DEBUG_LOG_NUMBER(serial, number) \
	SerialWriteNumber(serial_, number, 10); \
	SerialWrite(serial_, '\n')
#define SERIAL_DEBUG_LOG_NUMBER_WITHOUT_LINEFEED(serial_, number) \
	SerialWriteNumber(serial_, number, 10)

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

#if defined(UBRRH) && defined(UBRRL) || defined(UBRR0H) && defined(UBRR0L)
struct Serial * SerialBegin0(uint32_t baud);
#elif defined(USBCON)
#include "usb_api.h"
#warning no serial port defined  (port 0)
#else
#error no serial port defined  (port 0)
#endif


#if defined(UBRR1H)
struct Serial * SerialBegin1(uint32_t baud);
#endif
#if defined(UBRR2H)
struct Serial * SerialBegin2(uint32_t baud);
#endif
#if defined(UBRR3H)
struct Serial * SerialBegin3(uint32_t baud);
#endif



#endif /* HSERIAL_H_ */
