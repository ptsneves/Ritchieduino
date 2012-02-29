/*
 * Ethernet.c
 *
 *  Created on: Feb 25, 2012
 *      Author: pneves
 */
#include "Ethernet.h"
#include "wiring.h"



/*
 * As per datasheet when the connection is terminated
 * the interrupt register will be set and as such we must
 * clear it by write it 1
 */
static void TCPClose()
{
	W5100ExecuteCommand(socket_commands.CLOSE);
	W5100WriteToSocketIR(0xFF);
}

/**
 * @brief	This Socket function initialize the channel in TCP mode, and set the port and wait for W5100 done it.
 * @return 	1 for success else 0.
 */
void W5100Socket(uint16_t port)
{
	static uint16_t local_port = 1055;

	if(port == 0)
		port = local_port++;

	TCPClose();
	W5100WriteToSocketMR(mode_register.TCP);
	//ValidateModeRegister(serial);

	W5100WriteToSocketPORT(port);
	W5100ExecuteCommand(socket_commands.OPEN);
}

/*
 * Gateway can be NULL and we will take care of that
 * The current implementation only uses socket 0, because in future only
 * that will be available.
 */
void TCPBegin(uint8_t *mac, uint8_t *ip, uint8_t *gateway, uint8_t *subnet) {


	W5100Start();



	W5100SetMACAddress(mac);

	//ValidateMacAddress(serial);


	W5100SetIPAddress(ip);
	//ValidateIpAddress(serial);


	if (gateway == NULL) {
		uint8_t default_gateway[4];
		gateway[0] = ip[0];
		gateway[1] = ip[1];
		gateway[2] = ip[2];
		gateway[3] = 1;
		W5100SetGatewayIp(default_gateway);
	}
	else {
		W5100SetGatewayIp(gateway);
	}
	//ValidateGatewayAddress(serial);


	W5100SetSubnetMask(subnet);
	//ValidateSubnetAddress(serial);
}

uint8_t TCPListen(uint16_t port)
{
	//Already in listening mode no need to init again
	if (W5100ReadFromSocketSR() == status_register.LISTEN) {

		return 1;
	}


	W5100Socket(port);



	if (W5100ReadFromSocketSR() != status_register.INIT) {

		return 0;
	}


	W5100ExecuteCommand(socket_commands.LISTEN);

	if (W5100ReadFromSocketSR() != status_register.LISTEN) {
		return 0;
	}

	return 1;
}

uint8_t TCPStatus() {

	uint8_t return_value =  W5100ReadFromSocketSR();
	return return_value;
}

uint16_t TCPAvailable() {

	uint16_t return_value = W5100GetRXReceivedSize();
	return return_value;
}

uint8_t TCPConnected() {
	uint8_t status = TCPStatus();
	uint8_t return_value = !(status == status_register.LISTEN || status == status_register.CLOSED ||
			status == status_register.FIN_WAIT ||
	(status == status_register.CLOSE_WAIT && !TCPAvailable()));
	return return_value;

}

uint8_t TCPEstablished() {
	if (W5100ReadFromSocketSR() == status_register.ESTABLISHED) {

		return 1;
	}
	else {

		return 0;
	}
}

void TCPStop() {
	W5100ExecuteCommand(socket_commands.DISCON);
	unsigned long start = millis();

	// wait a second for the connection to close
	while (TCPStatus() != status_register.CLOSED && millis() - start < 1000)
		delay(1);

	// if it hasn't closed, close it forcefully
	if (TCPStatus() != status_register.CLOSED)
		TCPClose();
}

/**
 * @brief	This function established  the connection for the channel in Active (client) mode.
 * 		This function waits for the untill the connection is established.
 *
 * @return	1 for success else 0.
 */
uint8_t TCPConnect(uint8_t * addr, uint16_t port)
{
  if ((addr[0] == 0xFF && addr[1] == 0xFF && addr[2] == 0xFF && addr[3] == 0xFF) ||
    (addr[0] == 0x00 && addr[1] == 0x00 && addr[2] == 0x00 && addr[3] == 0x00) ||
    (port == 0x00))
	  return 0;

  // set destination IP
  W5100WriteToSocketDIPR(addr);
  W5100WriteToSocketDPORT(port);
  W5100ExecuteCommand(socket_commands.CONNECT);

  return 1;
}

/*
 * Takes the socket to work on and a buffer to put the read data from the W5100 socket
 *	Size is in bytes and notice the uint16_t return code. Be careful with implicit
 *	castings to smaller types
 *	You can fully buffer this value once with the maximum RX_value
 */
uint16_t TCPReceive(uint8_t *buffer, size_t len)
{
	uint16_t ret=0;
	if ( len > 0 )
	{
		W5100ReceiveDataProcessing(buffer, len);
		W5100ExecuteCommand(socket_commands.RECV);
		ret = len;
	}
	return ret;
}

/**
 * @brief	This function used to W5100Send the data in TCP mode
 * @return	The size of the sent data. If the amount of data to send is bigger
 * than W5100SSIZE, then, the data sent will only have W5100SSIZE bytes and not
 * len bytes
 */
uint16_t TCPSend(const uint8_t * buffer, size_t len)
{
	uint8_t status=0;
	uint16_t ret=0;
	uint16_t freesize=0;

	if (len > W5100SSIZE)
		ret = W5100SSIZE; // check size not to exceed MAX size.
	else
		ret = len;

	// Wait for buffer space in the TXBuffer
	do {
		freesize = W5100GetTXFreeSize();
		status = W5100ReadFromSocketSR();
		if (status != status_register.ESTABLISHED &&
				status != status_register.CLOSE_WAIT) {
			ret = 0;
			break;
		}
	} while (freesize < ret);

	// copy data
	W5100SendDataProcessing((uint8_t *)buffer, ret);
	W5100ExecuteCommand(socket_commands.SEND);

	//Wait for the operation to complete
	while ((W5100ReadFromSocketIR() & interrupt_register.SEND_OK) != interrupt_register.SEND_OK ) {
	/* m2008.01 [bj] : reduce code */
		if ( W5100ReadFromSocketSR() == status_register.CLOSED )
		{
			TCPClose();
			return 0;
		}
	}
	/* +2008.01 bj */
	W5100WriteToSocketIR(interrupt_register.SEND_OK);
	return ret;
}
