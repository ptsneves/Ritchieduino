/*
 * w5100.h
 *
 *  Created on: Jan 23, 2012
 *      Author: pneves
 */

#ifndef W5100_H_
#define W5100_H_

#include <avr/pgmspace.h>

#define IDM_OR  0x8000
#define IDM_AR0 0x8001
#define IDM_AR1 0x8002
#define IDM_DR  0x8003



#define __GP_REGISTER8(name, address)             \
  void W5100WriteToRegister##name(uint8_t _data);          \
  uint8_t W5100ReadFromRegister##name();
#define __GP_REGISTER16(name, address)            \
  void W5100WriteToRegister##name(uint16_t _data) ;  \
  uint16_t W5100ReadFromRegister##name();
#define __GP_REGISTER_N(name, address, size);       \
  uint16_t W5100ReadFromRegister##name(uint8_t *buff) ;

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
	void W5100WriteToSocket##name(uint8_t _data); \
	uint8_t W5100ReadFromSocket##name();
#define __SOCKET_REGISTER16(name, address)                   \
	void W5100WriteToSocket##name(uint16_t _data); \
	uint16_t W5100ReadFromSocket##name();
#define __SOCKET_REGISTER_N(name, address, size);          \
	uint16_t W5100WriteToSocket##name(uint8_t *buff); \
	uint16_t W5100ReadFromSocket##name(uint8_t *buff);


  __SOCKET_REGISTER8(MR,        0x0000)        // Mode
  __SOCKET_REGISTER8(CR,        0x0001)        // Command
  __SOCKET_REGISTER8(IR,        0x0002)        // Interrupt
  __SOCKET_REGISTER8(SR,        0x0003)        // Status
  __SOCKET_REGISTER16(PORT,     0x0004)        // Source Port
  __SOCKET_REGISTER_N(DHAR,     0x0006, 6)     // Destination Hardw Addr
  __SOCKET_REGISTER_N(DIPR,     0x000C, 4)    // Destination IP Addr
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
 * The reason i dont use enums but struct of uint8
 * is because we know that enums are considered ints by default
 * and that means 16bit at least for each record. I know it is
 * possible to set it to short through a flag but that has side-effects
 * that are not worth discussing
 */
struct W5100ModeRegister {
	uint8_t CLOSE;
	uint8_t TCP;
	uint8_t UDP;
	uint8_t IPRAW;
	uint8_t MACRAW;
	uint8_t PPPOE;
	uint8_t ND;
	uint8_t MULTI;
	};

struct W5100SocketCommands {
	uint8_t OPEN;
	uint8_t LISTEN;
	uint8_t CONNECT;
	uint8_t DISCON;
	uint8_t CLOSE;
	uint8_t SEND;
	uint8_t SEND_MAC;
	uint8_t SEND_KEEP;
	uint8_t RECV;
};

struct W5100InterruptRegister {
	uint8_t SEND_OK;
	uint8_t TIMEOUT;
	uint8_t RECV;
	uint8_t DISCON;
	uint8_t CON;
};

struct W5100StatusRegister {
	uint8_t CLOSED;
	uint8_t INIT;
	uint8_t LISTEN ;
	uint8_t SYNSENT;
	uint8_t SYNRECV;
	uint8_t ESTABLISHED;
	uint8_t FIN_WAIT;
	uint8_t CLOSING;
	uint8_t TIME_WAIT;
	uint8_t CLOSE_WAIT;
	uint8_t LAST_ACK;
	uint8_t UDP;
	uint8_t IPRAW;
	uint8_t MACRAW;
	uint8_t PPPOE;
};

struct W5100ProtocolTypes {
	uint8_t IP;
	uint8_t ICMP;
	uint8_t IGMP;
	uint8_t GGP;
	uint8_t TCP;
	uint8_t PUP;
	uint8_t UDP;
	uint8_t IDP;
	uint8_t ND;
	uint8_t RAW;
};

extern const struct W5100ModeRegister mode_register;
extern const struct W5100SocketCommands socket_commands;
extern const struct W5100InterruptRegister interrupt_register;
extern const struct W5100StatusRegister status_register;
extern const struct W5100ProtocolTypes protocol_types;

/**
 * @brief	 This function is being called by send()
 * and sendto() function also.
 *
 * This function read the Tx write pointer register
 * and after copy the data in buffer update the Tx write pointer
 * register. User should read upper byte first and lower byte
 * later to get proper value.
 */
void W5100SendDataProcessing(uint8_t *data, uint16_t length);

/**
* @brief	This function is being called by recv() also.
*
* This function read the Rx read pointer register
* and after copy the data from receive buffer update the Rx write pointer register.
* User should read upper byte first and lower byte later to get proper value.
*/
void W5100ReceiveDataProcessing(uint8_t *data, uint16_t length);

void W5100Start();
void W5100SetGatewayIp(uint8_t *addr);
uint16_t W5100GetGatewayIp(uint8_t *addr);

void W5100SetSubnetMask(uint8_t *addr);
uint16_t W5100GetSubnetMask(uint8_t *addr);

void W5100SetMACAddress(uint8_t * addr);
uint16_t W5100GetMACAddress(uint8_t * addr);

void W5100SetIPAddress(uint8_t * addr);
uint16_t W5100GetIPAddress(uint8_t * addr);

void W5100SetRetransmissionTime(uint16_t timeout);
void W5100SetRetransmissionCount(uint8_t _retry);

void W5100ExecuteCommand(uint8_t command);

uint16_t W5100GetTXFreeSize();
uint16_t W5100GetRXReceivedSize();

uint8_t W5100WriteToRegisterSingleUint8(uint16_t _address, uint8_t _data);
uint16_t W5100WriteToRegister(uint16_t _address, uint8_t *buffer, uint16_t length);
uint8_t W5100ReadFromRegisterSingleUint8(uint16_t _address);
uint16_t W5100ReadFromRegister(uint16_t _address, uint8_t *buffer, uint16_t length);

uint8_t W5100ReadFromSocketSingleUint8(uint16_t _addr);
uint8_t W5100WriteToSocketSingleUint8(uint16_t _addr, uint8_t _data);
uint16_t W5100ReadFromSocket(uint16_t _addr, uint8_t *buffer, uint16_t length);
uint16_t W5100WriteToSocket(uint16_t _addr, uint8_t *buffer, uint16_t length);


static const uint8_t  RST = 7; // Reset BIT


static const uint16_t W5100SSIZE = 8192; // Max Tx buffer size
static const uint16_t W5100RSIZE = 8192; // Max Rx buffer size
static const uint16_t W5100SMASK = 8192 - 1; // Tx buffer MASK
static const uint16_t W5100RMASK = 8192 - 1; // Rx buffer MASK
//uint16_t W5100SBASE; // Tx buffer base address
//uint16_t W5100RBASE; // Rx buffer base address




#endif /* W5100_H_ */
