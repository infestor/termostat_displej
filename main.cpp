
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "software_uart.h"

#define NOP __asm__("nop\n\t");
void main() __attribute__ ((noreturn));

#define LED_OVERHEAT PD0
#define LED_TOPENI PD1
#define LED_ON(pin) PORTD &= ~(1 << pin);
#define LED_OFF(pin) PORTD |= (1 << pin);

#define SWITCH_LED(pin) PORTB ^= (1 << pin);

volatile uint8_t timer10msTriggered;
volatile uint16_t longTimer;

SoftwareUart<> uart;
volatile uint8_t uart_buffer[8];
volatile uint8_t uartPos; //pocet prijatych bytu na uartu
volatile uint8_t uartExpected; //ocekavany pocet bajtu - je to hodnota prvniho byte pri zahajeni komunikace
volatile uint8_t uartInProgress;
volatile uint8_t uartOutProgress;
volatile uint8_t uartTimeoutCitac;

volatile uint8_t teplota = 150;



//-----------------------------------------------------------
ISR(TIMER0_COMPA_vect) {
	timer10msTriggered++;
	longTimer++;
}

ISR(BADISR_vect) { //just for case
	NOP
}

ISR(PCINT0_vect)
{
	uart.handle_interrupt();
}

//-----------------------------------------------------------


void init(void)
{
	//timer0 10ms period, interrupt enable
	//prescaler 1024, count to 156
	OCR0A = 156;
	OCR0B = 170;
	TCCR0A = 2;
	TCCR0B = 5;
	TIMSK0 = 2;

	//musime vypnout UART aby se daly ovladat jeho diody
	//!! Ale musi se to vypnout uz tady. kdyz se to udela nize tak to nejak nefunguje...
	UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0));

	//Software onewire UART na PB0
	uart.begin(57600);

	DDRB |= (1 << PB5);

	//diody od UARTu jako signalizace -> OUTPUT
	//cervena = overheat
	//zelena = topeni ON
	DDRD |= (1 << LED_OVERHEAT) | (1 << LED_TOPENI);
	PORTD &= ~((1 << LED_OVERHEAT) | (1 << LED_TOPENI)); //obracena logika - 1 je vypne
}


void main(void)
{
	wdt_disable();
	init();
	sei();

	uint16_t tmout = longTimer + 100; //delay(1000)

	//Hlavni LOOP
	for(;;)
	{
	//======================================================================================================

		// --- TIMER interrupt ------------------------
		if (longTimer == tmout)
		{
			tmout += 100;

			//poslat pres uart neco cyklickeho
			teplota++;
			if (teplota < 150) teplota = 150;

			uart_buffer[0] = 1; //delka dat 1
			uart_buffer[1] = teplota;
			uartExpected = 2; //celkem se posilaji 2 byte
			uartPos = 0; //ukazatel na byte co se ma zrovna posilat
			uartOutProgress = 1;
			//SWITCH_LED(PB5)
			LED_OFF(LED_OVERHEAT)
			LED_ON(LED_TOPENI)

		}

		if (uartOutProgress)
		{
			if (uart.write(uart_buffer[uartPos]))
			{
				uartPos++;
			}
			else //doslo k nejake chybe pri zapisu
			{
				uartOutProgress = 0;
				LED_ON(LED_OVERHEAT)
			}

			if (uartPos == uartExpected) //vsechna data odeslana
			{
				uartOutProgress = 0;
				LED_OFF(LED_TOPENI)
			}
		}

		//------------------------------------------------------------------------
		if (0)
		{
			//uart.available()
			//uart.write();

			uint8_t inp = uart.read();

			if (uartInProgress == 0)
			{
				uartExpected = inp;
				uartInProgress = 1;
				uartPos = 0;
				uartTimeoutCitac = 0;
			}
			else
			{
				if (uartPos == uartExpected) //incoming packet is longer than allowed
				{
					uartInProgress = 0;
				}
				else
				{
					uart_buffer[uartPos] = inp;
					uartPos++;
				}
			}
		}

		if ( (uartInProgress) && (uartPos == uartExpected) ) //whole packet is received
		{
			//zpracujeme prichozi data
		}

		if ((uartInProgress) && (uartTimeoutCitac > 1) ) //timeout receiving whole packet
		{
			uartInProgress = 0; //reset receiving
		}

		//------------------------------------------------------------------------
		if (timer10msTriggered)
		{
			timer10msTriggered = 0;

		}




	//======================================================================================================
	}
}
