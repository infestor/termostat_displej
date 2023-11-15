/*
SoftwareUart (adapted from SoftwareSerialWithHalfDuplex)

This library is based off SoftwareSerial from Arduino 1.6.5r5, with half-duplex changes
from @nickstedman's SoftwareSerialWithHalfDuplex library.

Modifications by @micooke to reduce codespace, and template it to allow changing the rx buffer.

[SoftwareSerialWithHalfDuplex] (https://github.com/nickstedman/SoftwareSerialWithHalfDuplex)
is a simple modification that should definitely be rolled into the SoftwareSerial core library.

This library is an attempt to add greater transparency and useability to SoftwareSerial, but
it requires the user to setup a few more things, namely an interrupt routine.

In short the differences to SoftwareSerial are/will be:
* templated - change the buffer size without changing the library!
* HalfDuplex - taken from [SoftwareSerialWithHalfDuplex] (https://github.com/nickstedman/SoftwareSerialWithHalfDuplex)
* flush() call to cli() to stop interrupts was removed (cli() inside write() is needed unfortunately)
* Removed the static functions which are there to enable time sharing of the pinchange interrupt
  (used to recv() serial data) - your interrupt routine can handle the time sharing! If you dont
  know what i mean, see how 'active_object' was used. In short the new implementation does not block
  a pinchange interrupt on bank 0 from blocking pinchange interrupts on every other bank. There
  will always be interrupt scheduling conflicts, but now the user can manage the conflicts!
* Removed/exposed the pin change interrupts so that developers can make use of them

I do not claim copyright on the code.
This is adapted from : NewSoftSerial, SoftwareSerial and SoftwareSerialWithHalfDuplex

----

*/

#ifndef SoftwareUart_h
#define SoftwareUart_h

#include <inttypes.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay_basic.h>

//PB0 , PCINT0
#define RXPIN PB0
#define RXPORT_IN PINB
#define RXPORT_OUT PORTB

#define PCICR_BIT PCIE0 //reg = PCICR
#define PCMSK_REG PCMSK0
#define PCMSK_BIT PCINT0

#define TXPIN_HI RXPORT_OUT |= (1 << RXPIN)
#define TXPIN_LO RXPORT_OUT &= ~(1 << RXPIN)

#define pinMode_OUTPUT DDRB |= (1 << RXPIN)
#define pinMode_INPUT DDRB &= ~(1 << RXPIN)


/******************************************************************************
* Definitions
******************************************************************************/

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

template <uint8_t _SU_RX_BUFFER = 16>
//class SoftwareUart : public Stream
class SoftwareUart
{
private:
	// Expressed as 4-cycle delays (must never be 0!)
	uint16_t _rx_delay_centering;
	uint16_t _rx_delay_intrabit;
	uint16_t _rx_delay_stopbit;

	bool _buffer_overflow;

	char _receive_buffer[_SU_RX_BUFFER];
	volatile uint8_t _receive_buffer_tail;
	volatile uint8_t _receive_buffer_head;

	// private methods
	void recv();
	uint8_t rx_pin_read() { return RXPORT_IN & (1 << RXPIN); }
	void setRxIntMsk(bool enable);

	uint16_t _tx_delay;

	// Return num - sub, or 1 if the result would be < 1
	//static uint16_t subtract_cap(uint16_t num, uint16_t sub);
	uint16_t subtract_cap(uint16_t num, uint16_t sub);

	// private static method for timing
	//static inline void tunedDelay(uint16_t delay);
	inline void tunedDelay(uint16_t delay) { _delay_loop_2(delay); }

public:
	// public methods
	SoftwareUart(void);
	~SoftwareUart();
	void begin(long speed);

	bool listen() { _buffer_overflow = 0; _receive_buffer_head = 0; _receive_buffer_tail = 0; setRxIntMsk(true); return false; }
	bool stopListening() { setRxIntMsk(false); return true; }
	bool overflow() { bool ret = _buffer_overflow; if (ret) { _buffer_overflow = false; } return ret; }
	int16_t peek();
	virtual int16_t read();
	virtual int16_t available();
	virtual void flush() { _receive_buffer_head = _receive_buffer_tail = 0; }

	// public only for easy access by interrupt handlers
	inline void handle_interrupt() __attribute__((__always_inline__)) { recv(); };

	virtual size_t write(uint8_t byte);
	//using Print::write;
};

//
// Private methods
//

//
// The receive routine called by the interrupt handler
//
template <uint8_t _SU_RX_BUFFER>
void SoftwareUart<_SU_RX_BUFFER>::recv()
{

	uint8_t d = 0;

	// If RX line is high, then we don't see any start bit
	// so interrupt is probably not for us
	if (!rx_pin_read())
	{
		// Disable further interrupts during reception, this prevents
		// triggering another interrupt directly after we return, which can
		// cause problems at higher baudrates.
		setRxIntMsk(false);

		// Wait approximately 1/2 of a bit width to "center" the sample
		tunedDelay(_rx_delay_centering);

		// Read each of the 8 bits
		for (uint8_t i = 8; i > 0; --i)
		{
			tunedDelay(_rx_delay_intrabit);
			d >>= 1;
			if (rx_pin_read())
				d |= 0x80;
		}

		// if buffer full, set the overflow flag and return
		uint8_t next = (_receive_buffer_tail + 1) % _SU_RX_BUFFER;
		if (next != _receive_buffer_head)
		{

			// save new data in buffer: tail points to where byte goes
			_receive_buffer[_receive_buffer_tail] = d; // save new byte
			_receive_buffer_tail = next;
		}
		else
		{
			_buffer_overflow = true;
		}

		// skip the stop bit
		tunedDelay(_rx_delay_stopbit);

		// Re-enable interrupts when we're sure to be inside the stop bit
		setRxIntMsk(true);
	}

}

template <uint8_t _SU_RX_BUFFER>
uint16_t SoftwareUart<_SU_RX_BUFFER>::subtract_cap(uint16_t num, uint16_t sub)
{
	if (num > sub)
		return num - sub;
	else
		return 1;
}

//
// Public methods
//
//
// Constructor
//
template <uint8_t _SU_RX_BUFFER>
SoftwareUart<_SU_RX_BUFFER>::SoftwareUart(void)
{
	_tx_delay = 0;

	_buffer_overflow = false;

	_rx_delay_centering = 0;
	_rx_delay_intrabit = 0;
	_rx_delay_stopbit = 0;

	pinMode_INPUT; //pinMode(_rxPin, INPUT);
	//digitalWrite(_rxPin, HIGH);
	TXPIN_HI; //HIGH
	// pullup for normal logic
}

//
// Destructor
//
template <uint8_t _SU_RX_BUFFER>
SoftwareUart<_SU_RX_BUFFER>::~SoftwareUart()
{
	stopListening();
}

template <uint8_t _SU_RX_BUFFER>
void SoftwareUart<_SU_RX_BUFFER>::begin(long speed)
{
	_rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = 0;
	// Precalculate the various delays, in number of 4-cycle delays
	uint16_t bit_delay = (F_CPU / speed) / 4;

	// 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
	// 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
	// 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
	// These are all close enough to just use 15 cycles, since the inter-bit
	// timings are the most critical (deviations stack 8 times)
	_tx_delay = subtract_cap(bit_delay, 15 / 4);

	// Only setup rx when we have a valid PCINT for this pin
	// Timings counted from gcc 4.8.2 output. This works up to 115200 on
	// 16Mhz and 57600 on 8Mhz.
	//
	// When the start bit occurs, there are 3 or 4 cycles before the
	// interrupt flag is set, 4 cycles before the PC is set to the right
	// interrupt vector address and the old PC is pushed on the stack,
	// and then 75 cycles of instructions (including the RJMP in the
	// ISR vector table) until the first delay. After the delay, there
	// are 17 more cycles until the pin value is read (excluding the
	// delay in the loop).
	// We want to have a total delay of 1.5 bit time. Inside the loop,
	// we already wait for 1 bit time - 23 cycles, so here we wait for
	// 0.5 bit time - (71 + 18 - 22) cycles.
	_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

	// There are 23 cycles in each loop iteration (excluding the delay)
	_rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

	// There are 37 cycles from the last bit read to the start of
	// stopbit delay and 11 cycles from the delay until the interrupt
	// mask is enabled again (which _must_ happen during the stopbit).
	// This delay aims at 3/4 of a bit time, meaning the end of the
	// delay will be at 1/4th of the stopbit. This allows some extra
	// time for ISR cleanup, which makes 115200 baud at 16Mhz work more
	// reliably
	_rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);


	// Enable the PCINT for the entire port here, but never disable it
	// (others might also need it, so we disable the interrupt by using
	// the per-pin PCMSK register).
	PCICR |= (1 << PCICR_BIT);

	tunedDelay(_tx_delay); // if we were low this establishes the end
	listen();
}

template <uint8_t _SU_RX_BUFFER>
void SoftwareUart<_SU_RX_BUFFER>::setRxIntMsk(bool enable)
{
	if (enable)
		PCMSK_REG |= (1 << PCMSK_BIT);
	else
		PCMSK_REG &= ~(1 << PCMSK_BIT);
}

// Read data from buffer
template <uint8_t _SU_RX_BUFFER>
int16_t SoftwareUart<_SU_RX_BUFFER>::read()
{
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;

	// Read from "head"
	uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
	_receive_buffer_head = (_receive_buffer_head + 1) % _SU_RX_BUFFER;
	return d;
}

template <uint8_t _SU_RX_BUFFER>
int16_t SoftwareUart<_SU_RX_BUFFER>::available()
{
	return (_receive_buffer_tail + _SU_RX_BUFFER - _receive_buffer_head) % _SU_RX_BUFFER;
}

template <uint8_t _SU_RX_BUFFER>
int16_t SoftwareUart<_SU_RX_BUFFER>::peek()
{
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;

	// Read from "head"
	return _receive_buffer[_receive_buffer_head];
}


template <uint8_t _SU_RX_BUFFER>
size_t SoftwareUart<_SU_RX_BUFFER>::write(uint8_t b)
{
	if (_tx_delay == 0)
	{
		//setWriteError();
		return 0;
	}

	uint8_t oldSREG = SREG;
	uint16_t delay = _tx_delay;

	cli();  // turn off interrupts for a clean txmit

	// NS - Set Pin to Output
	pinMode_OUTPUT; //pinMode(_txPin, OUTPUT);

	// Write the start bit
	TXPIN_LO;

	tunedDelay(delay);

	// Write each of the 8 bits
	for (uint8_t i = 8; i > 0; --i)
	{
		if (b & 1) // choose bit
			RXPORT_OUT |= (1 << RXPIN); // send 1
		else
			RXPORT_OUT &= ~(1 << RXPIN); // send 0

		tunedDelay(delay);
		b >>= 1;
	}

	// NS - Set Pin back to Input
	pinMode_INPUT; //pinMode(_txPin, INPUT);

	// restore pin to natural state (pullup ON)
	TXPIN_HI;

	PCIFR |= (1 << PCIF0);
	SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);

	return 1;
}

#endif
