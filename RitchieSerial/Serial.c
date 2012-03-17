#include "Serial.h"
#include "wiring_private.h"


// this next line disables the entire HardwareSerial.cpp,
// this is so I can support Attiny series and any other chip without a uart
#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

void SerialInit(struct Serial * const serial, struct RingBuffer *rx_bufferN,
		volatile uint8_t *ubrrh, volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
		volatile uint8_t *ucsrb, volatile uint8_t *udr, uint8_t rxen,
		uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x);
void SerialBegin(struct Serial * const serial, uint32_t baud);

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#if (RAMEND < 1000)
#define RX_BUFFER_SIZE 32
#else
#define RX_BUFFER_SIZE 128
#endif

/*
 * Our received data is buffered in a ring buffer
 * whose structure is this
 */
struct RingBuffer {
	unsigned char buffer[RX_BUFFER_SIZE];
	int head;
	int tail;
};

/*
 * Function Called by the USART interrupts to pass the
 * received data to the ring buffer.
 */
void store_char(unsigned char c, struct RingBuffer *rx_bufferN) {
	int i = (unsigned int)(rx_bufferN->head + 1) % RX_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != rx_bufferN->tail) {
		rx_bufferN->buffer[rx_bufferN->head] = c;
		rx_bufferN->head = i;
	}
}


/*
 * Data structures shared by both USART types
 */
#if (defined(UBRRH) && defined(UBRRL)) || (defined(UBRR0H) && defined(UBRR0L))
static struct Serial serial0 = {};
static struct RingBuffer rx_buffer0  =  { { 0 }, 0, 0 };
#endif


/*
 * Notice the ring buffers are global so the interrupts can put data
 * on them. According to the USART type available to the chip
 * the serial inits are different. There is room to macrofy this
 * but the use of globals in macros is a waste of time
 * Usb implementation is open
 * uint32_t bauds are passed around because i think the time wasted dereferencing
 * pointers is enough to not justify uint32_t *. If im wrong correct me
 */
#if defined(UBRRH) && defined(UBRRL)
struct Serial * SerialBegin0(uint32_t baud) {
	SerialInit(&serial0, &rx_buffer0, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRE, U2X);
	SerialBegin(&serial0, baud);
	return &serial0;
}

#elif defined(UBRR0H) && defined(UBRR0L)
struct Serial * SerialBegin0(uint32_t baud) {
	SerialInit(&serial0, &rx_buffer0, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRE0, U2X0);
	SerialBegin(&serial0, baud);
	return &serial0;
}

#elif defined(USBCON)
#include "usb_api.h"
#warning no serial port defined  (port 0)
#else
#error no serial port defined  (port 0)
#endif


#if defined(UBRR1H)
static struct RingBuffer rx_buffer1  =  { { 0 }, 0, 0 };
static struct Serial serial1 = {};

struct Serial * SerialBegin1(uint32_t baud) {
	SerialInit(&serial1, &rx_buffer1, &UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1, RXEN1, TXEN1, RXCIE1, UDRE1, U2X1);
	SerialBegin(&serial1, baud);
	return &serial1;
}
#endif
#if defined(UBRR2H)
static struct RingBuffer rx_buffer2  =  { { 0 }, 0, 0 };
static struct Serial serial2 = {};

struct Serial * SerialBegin2(uint32_t baud) {
	SerialInit(&serial2, &rx_buffer2, &UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UDR2, RXEN2, TXEN2, RXCIE2, UDRE2, U2X2);
	SerialBegin(&serial2, baud);
	return &serial2;
}
#endif
#if defined(UBRR3H)
static struct RingBuffer rx_buffer3  =  { { 0 }, 0, 0 };
static struct Serial serial3 = {};

struct Serial * SerialBegin3(uint32_t baud) {
	SerialInit(&serial3, &rx_buffer3, &UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UDR3, RXEN3, TXEN3, RXCIE3, UDRE3, U2X3);
	SerialBegin(&serial3, baud);
	return &serial3;
}
#endif


/*
 * Interrupt handlers
 */
#if defined(USART_RX_vect)
ISR(USART_RX_vect)
{
#if defined(UDR0)
	unsigned char c  =  UDR0;
#elif defined(UDR)
	unsigned char c  =  UDR;  //  atmega8535
#else
#error UDR not defined
#endif
	store_char(c, &rx_buffer0);
}
#elif defined(SIG_USART0_RECV) && defined(UDR0)
ISR(SIG_USART0_RECV)
{
	unsigned char c  =  UDR0;
	store_char(c, &rx_buffer0);
}
#elif defined(SIG_UART0_RECV) && defined(UDR0)
ISR(SIG_UART0_RECV)
{
	unsigned char c  =  UDR0;
	store_char(c, &rx_buffer0);
}
//#elif defined(SIG_USART_RECV)
#elif defined(USART0_RX_vect)
// fixed by Mark Sproul this is on the 644/644p
//ISR(SIG_USART_RECV)
ISR(USART0_RX_vect)
{
#if defined(UDR0)
	unsigned char c  =  UDR0;
#elif defined(UDR)
	unsigned char c  =  UDR;  //  atmega8, atmega32
#else
#error UDR not defined
#endif
	store_char(c, &rx_buffer0);
}
#elif defined(SIG_UART_RECV)
// this is for atmega8
ISR(SIG_UART_RECV)
{
#if defined(UDR0)
	unsigned char c  =  UDR0;  //  atmega645
#elif defined(UDR)
	unsigned char c  =  UDR;  //  atmega8
#endif
	store_char(c, &rx_buffer0);
}
#elif defined(USBCON)
#warning No interrupt handler for usart 0
#warning SerialStart(0) is on USB interface
#else
#error No interrupt handler for usart 0
#endif

#if defined(USART1_RX_vect)
//ISR(SIG_USART1_RECV)
ISR(USART1_RX_vect)
{
	unsigned char c = UDR1;
	store_char(c, &rx_buffer1);
}
#elif defined(SIG_USART1_RECV)
#error SIG_USART1_RECV
#endif

#if defined(USART2_RX_vect) && defined(UDR2)
ISR(USART2_RX_vect)
{
	unsigned char c = UDR2;
	store_char(c, &rx_buffer2);
}
#elif defined(SIG_USART2_RECV)
#error SIG_USART2_RECV
#endif

#if defined(USART3_RX_vect) && defined(UDR3)
ISR(USART3_RX_vect)
{
	unsigned char c = UDR3;
	store_char(c, &rx_buffer3);
}
#elif defined(SIG_USART3_RECV)
#error SIG_USART3_RECV
#endif


/*
 * Takes a structure and initializes it.
 * This function is mainly for internal use as it takes one of the
 * ring buffers available and puts its memory address in the structure
 * The volatile variables come from the registers.
 *
 *
 */
void SerialInit(struct Serial * const serial, struct RingBuffer *rx_bufferN,
		volatile uint8_t *ubrrh, volatile uint8_t *ubrrl, volatile uint8_t *ucsra,
		volatile uint8_t *ucsrb, volatile uint8_t *udr, uint8_t rxen,
		uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x) {

	serial->rx_bufferN = rx_bufferN;
	serial->ubrrh = ubrrh;
	serial->ubrrl = ubrrl;
	serial->ucsra = ucsra;
	serial->ucsrb = ucsrb;
	serial->udr = udr;
	serial->rxen = rxen;
	serial->txen = txen;
	serial->rxcie = rxcie;
	serial->udre = udre;
	serial->u2x = u2x;
}

/*
 * Sets the Baud rates and prepares the pins for use
 */
void SerialBegin(struct Serial * const serial, uint32_t baud)
{
	uint16_t baud_setting;
	uint8_t use_u2x = 1;

#if F_CPU == 16000000UL
	// hardcoded exception for compatibility with the bootloader shipped
	// with the Duemilanove and previous boards and the firmware on the 8U2
	// on the Uno and Mega 2560.
	if (baud == 57600) {
		use_u2x = 0;
	}
#endif

	if (use_u2x) {
		*serial->ucsra = 1 << serial->u2x;
		baud_setting = (F_CPU / 4 / baud - 1) / 2;
	}
	else {
		*serial->ucsra = 0;
		baud_setting = (F_CPU / 8 / baud - 1) / 2;
	}

	// assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
	*serial->ubrrh = baud_setting >> 8;
	*serial->ubrrl = baud_setting;

	sbi(*serial->ucsrb, serial->rxen);
	sbi(*serial->ucsrb, serial->txen);
	sbi(*serial->ucsrb, serial->rxcie);
}

/*
 * Disable the serial comunication. No cleanup is made
 */
void SerialEnd(struct Serial * const serial) {
	cbi(*serial->ucsrb, serial->rxen);
	cbi(*serial->ucsrb, serial->txen);
	cbi(*serial->ucsrb, serial->rxcie);
}

void SerialFlush(struct Serial * const serial)
{
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of rx_buffer_head but before writing
	// the value to rx_buffer_tail; the previous value of rx_buffer_head
	// may be written to rx_buffer_tail, making it appear as if the buffer
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of rx_buffer_head but before writing
	// the value to rx_buffer_tail; the previous value of rx_buffer_head
	// may be written to rx_buffer_tail, making it appear as if the buffer
	// were full, not empty.
	serial->rx_bufferN->head = serial->rx_bufferN->tail;
}

//FIXME
void SerialWrite(struct Serial * const serial, uint8_t c) {
	while (!(*serial->ucsra & (1 << serial->udre)))
		;
	*serial->udr = c;
}

int SerialRead(struct Serial * const serial) {
	// if the head isn't ahead of the tail, we don't have any characters
	if (serial->rx_bufferN->head == serial->rx_bufferN->tail) {
		return -1;
	}
	else {
		unsigned char c = serial->rx_bufferN->buffer[serial->rx_bufferN->tail];
		serial->rx_bufferN->tail =
				(unsigned int)(serial->rx_bufferN->tail + 1) %
				RX_BUFFER_SIZE;
		return c;
	}
}
int SerialPeek(struct Serial * const serial)
{
	if (serial->rx_bufferN->head == serial->rx_bufferN->tail) {
		return -1;
	}
	else {
		return serial->rx_bufferN->buffer[serial->rx_bufferN->tail];
	}
}

int SerialAvailable(struct Serial * const serial) {
	//	SerialWriteNumber(serial, serial->rx_bufferN->head, 10);
	//	SerialWriteNumber(serial, serial->rx_bufferN->tail, 10);
	return (unsigned int)(RX_BUFFER_SIZE + serial->rx_bufferN->head -
			serial->rx_bufferN->tail) % RX_BUFFER_SIZE;
}


void SerialWriteString(struct Serial * const serial, const char *string, uint16_t size) {
	for(uint16_t i=0; i<size; i++) {
		SerialWrite(serial, string[i]);
	}
}

/*
 * Enables the printing of integers to the serial console
 */
void SerialWriteNumber(struct Serial * const serial,
		unsigned long n, uint8_t base) {
	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
	unsigned long i = 0;

	if (n == 0) {
		SerialWrite(serial, '0');
		return;
	}

	while (n > 0) {
		buf[i++] = n % base;
		n /= base;
	}

	for (; i > 0; i--)
		SerialWrite(serial, (char) (buf[i - 1] < 10 ?
				'0' + buf[i - 1] :
				'A' + buf[i - 1] - 10));
}

#endif
