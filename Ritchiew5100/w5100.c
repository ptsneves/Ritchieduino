/*
 * w5100.c
 *
 *  Created on: Jan 23, 2012
 *      Author: pneves
 */
#include "w5100.h"
#include "SPI.h"
#include "wiring.h"



#define TXBUF_BASE 0x4000
#define RXBUF_BASE 0x6000

static const uint16_t CH_BASE = 0x0400; //address of the first socket register
static const uint16_t CH_SIZE = 0x0100; //size of each socket register

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	static void initSS()    { DDRB  |=  _BV(4); };
	static void setSS()     { PORTB &= ~_BV(4); };
	static void resetSS()   { PORTB |=  _BV(4); };
#else
	static void initSS()    { DDRB  |=  _BV(2); };
	static void setSS()     { PORTB &= ~_BV(2); };
	static void resetSS()   { PORTB |=  _BV(2); };
#endif

#define __GP_REGISTER8(name, address)             \
  void W5100WriteToRegister##name(uint8_t _data) { \
    W5100WriteToRegisterSingleUint8(address, _data);                        \
  }                                               \
  uint8_t W5100ReadFromRegister##name() {            \
    return W5100ReadFromRegisterSingleUint8(address);                         \
  }
#define __GP_REGISTER16(name, address)            \
  void W5100WriteToRegister##name(uint16_t _data) {       \
    W5100WriteToRegisterSingleUint8(address,   _data >> 8);                 \
    W5100WriteToRegisterSingleUint8(address+1, _data & 0xFF);               \
  }                                               \
  uint16_t W5100ReadFromRegister##name() {                  \
    uint16_t res = W5100ReadFromRegisterSingleUint8(address);                 \
    res = (res << 8) + W5100ReadFromRegisterSingleUint8(address + 1);         \
    return res;                                   \
  }
#define __GP_REGISTER_N(name, address, size)      \
  uint16_t W5100WriteToRegister##name(uint8_t *buffer) {   \
    return W5100WriteToRegister(address, buffer, size);           \
  }                                               \
  uint16_t W5100ReadFromRegister##name(uint8_t *buffer) {    \
    return W5100ReadFromRegister(address, buffer, size);            \
  }

__GP_REGISTER8(MR,     0x0000)    // Mode
__GP_REGISTER_N(GAR,    0x0001, 4); // Gateway IP address
__GP_REGISTER_N(SUBR,   0x0005, 4); // Subnet mask address
__GP_REGISTER_N(SHAR,   0x0009, 6); // Source MAC address
__GP_REGISTER_N(SIPR,   0x000F, 4); // Source IP address
__GP_REGISTER8(IR,     0x0015);    // Interrupt
__GP_REGISTER8(IMR,    0x0016);    // Interrupt Mask
__GP_REGISTER16(RTR,    0x0017);    // Timeout address
__GP_REGISTER8(RCR,    0x0019);    // Retry count
__GP_REGISTER8(RMSR,   0x001A);    // Receive memory size
__GP_REGISTER8(TMSR,   0x001B);    // Transmit memory size
__GP_REGISTER8(PATR,   0x001C);    // Authentication type address in PPPoE mode
__GP_REGISTER8(PTIMER, 0x0028);    // PPP LCP Request Timer
__GP_REGISTER8(PMAGIC, 0x0029);    // PPP LCP Magic Number
__GP_REGISTER_N(UIPR,   0x002A, 4); // Unreachable IP address in UDP mode
__GP_REGISTER16(UPORT,  0x002E);    // Unreachable Port address in UDP mode

#undef __GP_REGISTER8
#undef __GP_REGISTER16
#undef __GP_REGISTER_N


#define __SOCKET_REGISTER8(name, address)                    \
  void W5100WriteToSocket##name(uint8_t _data) { \
	W5100WriteToSocketSingleUint8(address, _data);        \
  }                                                 \
  uint8_t W5100ReadFromSocket##name() {              \
    return W5100ReadFromSocketSingleUint8(address);                              \
  }
#define __SOCKET_REGISTER16(name, address)                   \
  void W5100WriteToSocket##name(uint16_t _data) {       \
	W5100WriteToSocketSingleUint8(address,   _data >> 8);                      \
	W5100WriteToSocketSingleUint8(address+1, _data & 0xFF);                    \
  }                                                          \
  uint16_t W5100ReadFromSocket##name() {                    \
    uint16_t res = W5100ReadFromSocketSingleUint8(address);                      \
    res = (res << 8) + W5100ReadFromSocketSingleUint8(address + 1);              \
    return res;                                              \
  }
#define __SOCKET_REGISTER_N(name, address, size)             \
  uint16_t W5100WriteToSocket##name(uint8_t *buffer) {   \
    return W5100WriteToSocket(address, buffer, size);                \
  }                                                          \
  uint16_t W5100ReadFromSocket##name(uint8_t *buffer) {    \
    return W5100ReadFromSocket(address, buffer, size);                 \
  }


  __SOCKET_REGISTER8(MR,        0x0000)        // Mode
  __SOCKET_REGISTER8(CR,        0x0001)        // Command
  __SOCKET_REGISTER8(IR,        0x0002)        // Interrupt
  __SOCKET_REGISTER8(SR,        0x0003)        // Status
  __SOCKET_REGISTER16(PORT,     0x0004)        // Source Port
  __SOCKET_REGISTER_N(DHAR,     0x0006, 6)     // Destination Hardw Addr
  __SOCKET_REGISTER_N(DIPR,     0x000C, 4)     // Destination IP Addr
  __SOCKET_REGISTER16(DPORT,    0x0010)        // Destination Port
  __SOCKET_REGISTER16(MSSR,     0x0012)        // Max Segment Size
  __SOCKET_REGISTER8(PROTO,     0x0014)        // Protocol in IP RAW Mode
  __SOCKET_REGISTER8(TOS,       0x0015)        // IP TOS
  __SOCKET_REGISTER8(TTL,       0x0016)        // IP TTL
  __SOCKET_REGISTER16(TX_FSR,   0x0020)        // TX Free Size
  __SOCKET_REGISTER16(TX_RD,    0x0022)        // TX Read Pointer
  __SOCKET_REGISTER16(TX_WR,    0x0024)        // TX Write Pointer
  __SOCKET_REGISTER16(RX_RSR,   0x0026)        // RX Free Size
  __SOCKET_REGISTER16(RX_RD,    0x0028)        // RX Read Pointer
  __SOCKET_REGISTER16(RX_WR,    0x002A)        // RX Write Pointer (supported?)

#undef __SOCKET_REGISTER8
#undef __SOCKET_REGISTER16
#undef __SOCKET_REGISTER_N


/*
 * Initialization of Ethernet Card
 * Notice that this delay is necessary and the register initialization
 * is in the datasheet.
 * Only one socket with 8Kb is available
 */
void W5100Start() {
	delay(300);
	SPIStart();
	initSS();

	W5100WriteToRegisterMR(1<<RST);
	W5100WriteToRegisterTMSR(0x03);
	W5100WriteToRegisterRMSR(0x03);

}

/*
 * The do while is because of the possibility
 * of the TX_FSR not being stable
 */
uint16_t W5100GetTXFreeSize() {
	uint16_t val=0, val1=0;

	do {
		val1 = W5100ReadFromSocketTX_FSR();
		if (val1 != 0)
		  val = W5100ReadFromSocketTX_FSR();
	}
	while (val != val1);
	return val;
}

uint16_t W5100GetRXReceivedSize() {
	uint16_t val = 0;
	uint16_t val1 = 0;
	do {
		val1 = W5100ReadFromSocketRX_RSR();
		if (val1 != 0)
			val = W5100ReadFromSocketRX_RSR();
	}
	while (val != val1);
	return val;
}

void W5100SendDataProcessing(uint8_t *data, uint16_t length)
{
  uint16_t ptr = W5100ReadFromSocketTX_WR();

  uint16_t offset = ptr & W5100SMASK;
  uint16_t dstAddr = offset + TXBUF_BASE;

  if (offset + length > W5100SSIZE) {
	// Wrap around circular buffer
	uint16_t size = W5100SSIZE - offset;
	W5100WriteToRegister(dstAddr, data, size);
	W5100WriteToRegister(TXBUF_BASE, data + size, length - size);
  }
  else {
	  W5100WriteToRegister(dstAddr, data, length);
  }

  ptr += length;
  W5100WriteToSocketTX_WR(ptr);
}

/*
 * @brief	This function is being used to copy the data from
 * Receive buffer of the chip to application buffer.
 *
 * It calculates the actual physical address where one has to read
 * the data from Receive buffer. Here also take care of the condition while
 * it exceeds the Rx memory uper-bound of socket.
 */
static void W5100ReadFromRegisterData(uint16_t src, uint8_t *destination, uint16_t length)
{
  uint16_t size;
  uint16_t src_mask;
  uint16_t src_ptr;

  src_mask = src & W5100RMASK;
  src_ptr = RXBUF_BASE + src_mask;

  if( (src_mask + length) > W5100RSIZE ) {
	size = W5100RSIZE - src_mask;
	W5100ReadFromRegister(src_ptr, destination, size);
	destination += size;
	W5100ReadFromRegister(RXBUF_BASE,  destination, length - size);
  }
  else
	W5100ReadFromRegister(src_ptr, destination, length);
}

void W5100ReceiveDataProcessing(uint8_t *data, uint16_t length)
{
  uint16_t ptr;
  ptr = W5100ReadFromSocketRX_RD();
  W5100ReadFromRegisterData(ptr, data, length);

  ptr += length;
  W5100WriteToSocketRX_RD(ptr);

}


uint8_t W5100WriteToRegisterSingleUint8(uint16_t address, uint8_t _data)
{
  setSS();
  SPITransfer(0xF0);
  SPITransfer(address >> 8);
  SPITransfer(address & 0xFF);
  SPITransfer(_data);
  resetSS();
  return 1;
}

uint16_t W5100WriteToRegister(uint16_t address, uint8_t *buffer, uint16_t _len)
{
  for (int i=0; i<_len; i++)
  {
    setSS();
    SPITransfer(0xF0);
    SPITransfer(address >> 8);
    SPITransfer(address & 0xFF);
    address++;
    SPITransfer(buffer[i]);
    resetSS();
  }
  return _len;
}

uint8_t W5100ReadFromRegisterSingleUint8(uint16_t address)
{
  setSS();
  SPITransfer(0x0F);
  SPITransfer(address >> 8);
  SPITransfer(address & 0xFF);
  uint8_t _data = SPITransfer(0);
  resetSS();
  return _data;
}

uint16_t W5100ReadFromRegister(uint16_t address, uint8_t *buffer, uint16_t _len)
{
  for (int i=0; i<_len; i++)
  {
    setSS();
    SPITransfer(0x0F);
    SPITransfer(address >> 8);
    SPITransfer(address & 0xFF);
    address++;
    buffer[i] = SPITransfer(0);
    resetSS();
  }
  return _len;
}


/*
 * According to the datasheet after writing to command register
 * when the command is executed it automatically resets to 0
 */
void W5100ExecuteCommand(uint8_t command) {
	W5100WriteToSocketCR(command);
	while (W5100ReadFromSocketCR())  ;
}

uint8_t W5100ReadFromSocketSingleUint8(uint16_t address) {
  return W5100ReadFromRegisterSingleUint8(CH_BASE + address);
}

uint8_t W5100WriteToSocketSingleUint8(uint16_t address, uint8_t _data) {
  return W5100WriteToRegisterSingleUint8(CH_BASE + address, _data);
}

uint16_t W5100ReadFromSocket(uint16_t address, uint8_t *buffer, uint16_t _len) {
  return W5100ReadFromRegister(CH_BASE + address, buffer, _len);
}

uint16_t W5100WriteToSocket(uint16_t address, uint8_t *buffer, uint16_t _len) {
  return W5100WriteToRegister(CH_BASE + address, buffer, _len);
}

uint16_t W5100GetGatewayIp(uint8_t *address) {
  return W5100ReadFromRegisterGAR(address);
}

void W5100SetGatewayIp(uint8_t *address) {
  W5100WriteToRegisterGAR(address);
}

uint16_t W5100GetSubnetMask(uint8_t *address) {
  return W5100ReadFromRegisterSUBR(address);
}

void W5100SetSubnetMask(uint8_t *address) {
  W5100WriteToRegisterSUBR(address);
}

uint16_t W5100GetMACAddress(uint8_t *address) {
	return W5100ReadFromRegisterSHAR(address);
}

void W5100SetMACAddress(uint8_t *address) {
  W5100WriteToRegisterSHAR(address);
}

uint16_t W5100GetIPAddress(uint8_t *address) {
	return W5100ReadFromRegisterSIPR(address);
}

void W5100SetIPAddress(uint8_t *address) {
  W5100WriteToRegisterSIPR(address);
}

void W5100SetRetransmissionTime(uint16_t _timeout) {
  W5100WriteToRegisterRTR(_timeout);
}

void W5100SetRetransmissionCount(uint8_t _retry) {
  W5100WriteToRegisterRCR(_retry);
}
