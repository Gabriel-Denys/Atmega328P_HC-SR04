/*
* Created: 3/04/2019
* Author : Gabriel Denys
* License: MIT
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define F_CPU 16000000UL //clock

#include <util/delay.h>

#define _BV(bit) (1 << (bit)) //mask representation
#define CHECK_BIT(var,pos) ((var) & (1<<(pos))) //is bit set?
#define BETWEEN_INCL(i, min, max) (i >= min) && (i <= max)?1:0 //inclusive within range
#define BETWEEN(i, min, max) (i > min) && (i < max)?1:0//exclusive within range

volatile unsigned char rov_counter;/*counter for timer 1 overflow*/
volatile unsigned int period_high = 0;// stores the high period
volatile unsigned int got_period = 0;//got a new period - flag
volatile unsigned int timer0count;
volatile unsigned int distance;
volatile unsigned int cdr;

/*Function Declarations*/
void InitTimer0();
void InitTimer1();
void sendmsg (char *s);
void init_USART();
void usartsend(unsigned char data);
unsigned char USART_receive(void);
void init_HCSR04();

int main(void){
	
	//Initialization functions
	InitTimer0();
	InitTimer1();
	init_HCSR04();
	init_USART();
	
	char buffer[50]; //buffer for our sprintf function
	char ch; //character variable for received character
	sei(); //enable global interrupts

	while(1){
		if(cdr && got_period)//Continuous distance reporting & we got a new reading
		{
			sprintf(buffer, "Distance: %umm", distance);
			sendmsg(buffer);
		}
		
		if (UCSR0A & (1<<RXC0)) //check for character received
		{
			ch = USART_receive(); //get character sent from PC
			
			switch (ch)
			{
				case 'M':
				case 'm':
				sprintf(buffer, "Distance: %umm", distance); //display once on press
				sendmsg(buffer);
				break;
				case 'V':
				case 'v':
				sendmsg("Continuous Reporting of Distance: ON");
				cdr = 1; //turn continuous distance reporting on
				break;
				case 'w':
				case 'W':
				sendmsg("Continuous Reporting of Distance: OFF");
				cdr = 0; //turn continuous reporting off
				break;
			}
		}
	}
	return 1;
}

//Initialize Timer0
void InitTimer0(){
	TCCR0B=0b00000101; //Clock source: CLKIO/1024, Outputs disabled
	TIMSK0 |= (1<<TOIE0); // enable interrupt capture & overflow interrupts
	TCNT0 = 100;// 256 - 100 = 156, 156 * 64us = ~ 10ms
}

//Initialise Timer1
void InitTimer1(){
	TIMSK1 = (0<<ICIE1) | (1<<TOIE1); // disable interrupt capture & enable overflow interrupts
	TCCR1B = (0<<ICNC1) | (1<<ICES1) | (1<<CS11); //outputs disabled by default
}

//initialise Ultrasound Sensor
void init_HCSR04()
{
	DDRD = (1<<PORTD4); //enable PD4 as output
}

//Overflow Interrupt
ISR(TIMER0_OVF_vect)
{
	timer0count++;
	if(timer0count == 7) //has 70ms elapsed?
	{
		PORTD ^=(1<<PORTD4);
		_delay_us(10);
		PORTD ^=(1<<PORTD4);
		_delay_us(25); //the 40kHz pulsing lasts 25us, let's not read anything during this time because of echoes/noise
		TIMSK1 |= (1<<ICIE1); //enable interrupts
		TCCR1B |= (1<<ICES1); //trigger on rising edge
		timer0count = 0;
		TCNT0 = 100;
	}
}

//Overflow Interrupt
ISR(TIMER1_OVF_vect)
{
	rov_counter++; /*increment counter when overflow occurs*/
}

//Capture interrupt
ISR(TIMER1_CAPT_vect)
{
	static volatile unsigned int starting_edge,ending_edge;/*storage for times*/
	static volatile int high = 1;
	static volatile int old_distance;
	volatile unsigned long clocks;/*storage for actual clock counts in the pulse*/
	TCCR1B ^= (1<<ICES1); //switch which edge to trigger on

	ending_edge=ICR1;
	clocks=((unsigned long)rov_counter * 65536) + (unsigned long)ending_edge -(unsigned long)starting_edge;
	if(high)period_high = (clocks/2); //dividing by two because each tick is 0.5s @ 16MHz with prescaler of 8

	rov_counter=0; //reset counter
	starting_edge=ending_edge; //save end time to use as starting edge
	distance = (period_high*0.34)/2; // 0.34mm/us is speed of sound
	if(high && old_distance != distance)//don't say we got new data if it's the same as the previous data
	{
		got_period = 1;
	}
	else
	{
		got_period = 0;
	}
	old_distance = distance; //save distance
	if(!high)TIMSK1 ^= (1<<ICIE1); //if we are entering low period we must turn capture interrupt enable off
	
	high = ~high; //
}


// Initialise everything for the functioning of USART
void init_USART()
{
	DDRB |= (1<<PORTB1); // enable port bit 1
	UCSR0A	= 0b10; //changed to double speed mode, because error is too large on normal mode for 115200 baud
	UCSR0B	= (1<<RXEN0) | (1<<TXEN0) | (1<<TXC0);  /*enable receiver, transmitter and transmit interrupt*/
	UBRR0	= 16;  /*baud rate = 11520*/
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);//8 bit mode
}

//Send a character array i.e. 'string' to UDR0
void sendmsg (char *s)
{
	while (*s != 0x00)
	{
		usartsend(*s);//send character
		s++;
		if(*s == 0x00) //if end of array reached
		{
			//Send return and newline
			usartsend(0x0d);
			usartsend(0x0a);
		}
	}
}

//Send a single character to UDR0
void usartsend(unsigned char data)
{
	while(!(UCSR0A & _BV(UDRE0) ) ) //if buffer is busy do nothing
	;
	UDR0 = data;
}

//Simply retrieve a character from serial
unsigned char USART_receive(void){
	while(!(UCSR0A & _BV(UDRE0) ) )//if buffer is busy do nothing
	;
	return UDR0;
}

//USART Transmit Complete ISR, not actually using it but still needed
ISR(USART_TX_vect)
{
	/*
	This is probably the more lazy(quicker) way of enabling interrupt flags
	But it works.
	*/
}
