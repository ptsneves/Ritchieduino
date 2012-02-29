#include "SPI.h"

#include "WProgram.h"
#include "pins_arduino.h"
uint8_t SPITransfer(uint8_t _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

void SPIAttachInterrupt() {
  SPCR |= _BV(SPIE);
}

void SPIDetachInterrupt() {
  SPCR &= ~_BV(SPIE);
}

void SPIStart() {
	// Set direction register for SCK and MOSI pin.
	  // MISO pin automatically overrides to INPUT.
	  // When the SS pin is set as OUTPUT, it can be used as
	  // a general purpose output port (it doesn't influence
	  // SPI operations).

	  pinMode(SCK, OUTPUT);
	  pinMode(MOSI, OUTPUT);
	  pinMode(SS, OUTPUT);

	  digitalWrite(SCK, LOW);
	  digitalWrite(MOSI, LOW);
	  digitalWrite(SS, HIGH);

	  // Warning: if the SS pin ever becomes a LOW INPUT then SPI
	  // automatically switches to Slave, so the data direction of
	  // the SS pin MUST be kept as OUTPUT.
	  SPCR |= _BV(MSTR);
	  SPCR |= _BV(SPE);
}
void SPIEnd() {
	SPCR &= ~_BV(SPE);
}
void SPISetBitOrder(uint8_t bitOrder)
{
  if(bitOrder == LSBFIRST) {
    SPCR |= _BV(DORD);
  } else {
    SPCR &= ~(_BV(DORD));
  }
}

void SPISetDataMode(uint8_t mode)
{
  SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

void SPISetClockDivider(uint8_t rate)
{
  SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
  SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}
