///Made by- Mihir Bhatia 76149921
// Prashant Wason 81087413  


#define F_CPU 16000000UL
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#include <util/delay.h>


// Flash memory commands
#define WRITE_ENABLE     0x06  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define WRITE_DISABLE    0x04  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define READ_STATUS      0x05  // Address:0 Dummy:0 Num:1 to infinite fMax: 32MHz
#define READ_BYTES       0x03  // Address:3 Dummy:0 Num:1 to infinite fMax: 20MHz
#define READ_SILICON_ID  0xab  // Address:0 Dummy:3 Num:1 to infinite fMax: 32MHz
#define FAST_READ        0x0b  // Address:3 Dummy:1 Num:1 to infinite fMax: 40MHz
#define WRITE_STATUS     0x01  // Address:0 Dummy:0 Num:1 fMax: 25MHz
#define WRITE_BYTES      0x02  // Address:3 Dummy:0 Num:1 to 256 fMax: 25MHz
#define ERASE_ALL        0xc7  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define ERASE_BLOCK      0xd8  // Address:3 Dummy:0 Num:0 fMax: 25MHz
#define READ_DEVICE_ID   0x9f  // Address:0 Dummy:2 Num:1 to infinite fMax: 25MHz


#define SET_CS PORTD |= 0b10000000
#define CLR_CS PORTD &= 0b01111111
#define SET_MOSI PORTB |= 0b00000010
#define CLR_MOSI PORTB &= 0b11111101
#define SET_SCLK PORTB |= 0b00000100
#define CLR_SCLK PORTB &= 0b11111011
#define MISO_SET ((PINB & 0b00000001)==0b00000001)

#define DEF_FREQ 22050L
#define OCR1_RELOAD ((F_CPU/DEF_FREQ)+1)


unsigned int cnt = 0;

volatile unsigned long int playcnt=0;
volatile unsigned char play_flag=0;





// Bitbang SPI.  Surprisingly fast!  SCLK is about 1.25MHz.
unsigned char SPIWrite(unsigned char tx)
{
    unsigned char i, rx, mask;

    mask=0x80;
    rx=0;

    for(i=0; i<8; i++)
    {
        if(tx & mask)
            SET_MOSI;            
        else
        	CLR_MOSI;

        SET_SCLK;        

        if(MISO_SET) rx |= mask;
        mask>>=1;

        CLR_SCLK;                
    }
    return rx;
}

// 'Timer 1 output compare A' Interrupt Service Routine
ISR(TIMER1_COMPA_vect)
{
	OCR1A = OCR1A + OCR1_RELOAD;
	PORTD ^= 0b00100000; // Toggle PD5 (pin 11) to check that we have the right frequency

	if(play_flag!=0)
	{  
		if(playcnt==0)
		{
			SET_CS; // Done playing: Disable 25Q32 SPI flash memory
			play_flag=0;
		}
		else
		{
			OCR0A=SPIWrite(0x00); // Output value to PWM (used as DAC)
			playcnt--;
		}
	}
}



void wait_1ms(void)
{
	unsigned int saved_TCNT1;
	
	saved_TCNT1=TCNT1;
	
	while((TCNT1-saved_TCNT1)<(F_CPU/1000L)); // Wait for 1 ms to pass
}

void waitms(int ms)
{
	while(ms--) wait_1ms();
}

#define PIN_PERIOD (PIND & 0b00100000)

// GetPeriod() seems to work fine for frequencies between 30Hz and 300kHz.
long int GetPeriod (int n)
{	
	int i, overflow;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	overflow=0;
	TIFR1=1; // TOV1 can be cleared by writing a logic one to its bit location.  Check ATmega328P datasheet page 113.
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>5) return 0;}
	}
	overflow=0;
	TIFR1=1;
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>5) return 0;}
	}
	
	overflow=0;
	TIFR1=1;
	saved_TCNT1a=TCNT1;
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>1024) return 0;}
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(TIFR1&1)	{ TIFR1=1; overflow++; if(overflow>1024) return 0;}
		}
	}
	saved_TCNT1b=TCNT1;
	if(saved_TCNT1b<saved_TCNT1a) overflow--; // Added an extra overflow.  Get rid of it.
	
	
	return overflow*0x10000L+(saved_TCNT1b-saved_TCNT1a);
}

void Init_pwm (void)
{
    DDRD |= (1 << DDD6); // PD6 is now an output (pin 12 of DIP28)
    OCR0A = 128; // set PWM for 50% duty cycle
    TCCR0A |= (1 << COM0A1); // set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00); // set fast PWM Mode
    TCCR0B |= (1 << CS00); // set prescaler to none and starts PWM
}

void Init_Timer1 (void)
{
	DDRD|=0b00100000; // PD5 (pin 11) is an output now
	TCCR1B |= _BV(CS10);   // set prescaler to Clock/1
	TIMSK1 |= _BV(OCIE1A); // output compare match interrupt for register A
	
	sei(); // enable global interupt
}

void Setup_BB_SPI (void)
{
	DDRB|=0b00000110; // PB1 and PB2 are outputs.  PB0 is input.
	DDRD|=0b10000000; // PD7 is output.
	PORTB |= 0b00000001; // Activate pull-up in PB0

	SET_CS;
	CLR_MOSI;
	CLR_SCLK;
}



void Start_Playback (unsigned long int address, unsigned long int numb)
{
    CLR_CS; // Select/enable 25Q32 SPI flash memory.
    SPIWrite(READ_BYTES);
    SPIWrite((unsigned char)((address>>16)&0xff));
    SPIWrite((unsigned char)((address>>8)&0xff));
    SPIWrite((unsigned char)(address&0xff));
    playcnt=numb;
    play_flag=1;
}




int main(void)
{


	long int count,countbase;
	float T, f,basef,baseT;
	
	usart_init(); // Configure the usart and baudrate
	
	DDRD  &= 0b11011111; // Configure PD5 as input
	PORTD |= 0b00100000; // Activate pull-up in PD5

	// Turn on timer with no prescaler on the clock.  We use it for delays and to measure period.
	TCCR1B |= _BV(CS10); // Check page 110 of ATmega328P datasheet

	waitms(500); // Wait for putty to start
	countbase=GetPeriod(100);
	if(countbase>0)
	{
	baseT=countbase/(F_CPU*100.0);
	basef=1/baseT;
	printf("Base f=%f",basef);
	}
	while (1)
	{	DDRD  &= 0b11011111; // Configure PD5 as input
		PORTD |= 0b00100000; // Activate pull-up in PD5
		count=GetPeriod(100);
		if(count>0)
		{
			T=count/(F_CPU*100.0);
			f=1/T;
			printf("f=%fHz (count=%lu)     \n\r", f, count);
			if(f>basef+40 && f<basef+150)   //small non ferrous
			{
				Init_pwm();  // Initialize the PWM output used as DAC
				Init_Timer1(); // Timer 1 is used as playback ISR
				Setup_BB_SPI(); // The hardware SPI is not available, so use bitbang SPI instead

				playcnt=0;
				play_flag=0;
	
				Start_Playback(0x06500,0x008000);
				while(play_flag);
				
			}
			
			else if(f>basef+150)     		//large non ferrous
			{
				Init_pwm();  // Initialize the PWM output used as DAC
				Init_Timer1(); // Timer 1 is used as playback ISR
				Setup_BB_SPI(); // The hardware SPI is not available, so use bitbang SPI instead

				playcnt=0;
				play_flag=0;
	
				Start_Playback(0x014000,0x007500);
				while(play_flag);
			}
			
			
			else if(f<basef-250)			//large ferrous
			{
				Init_pwm();  // Initialize the PWM output used as DAC
				Init_Timer1(); // Timer 1 is used as playback ISR
				Setup_BB_SPI(); // The hardware SPI is not available, so use bitbang SPI instead

				playcnt=0;
				play_flag=0;
	
				Start_Playback(0x0010000,0x005000);
				while(play_flag);
			}
			
			
			else if(f<basef-40 && f>basef-230)		//small ferrous
			{
				Init_pwm();  // Initialize the PWM output used as DAC
				Init_Timer1(); // Timer 1 is used as playback ISR
				Setup_BB_SPI(); // The hardware SPI is not available, so use bitbang SPI instead

				playcnt=0;
				play_flag=0;
	
				Start_Playback(0x000000,0x006800);
				while(play_flag);
			}
			
		}
		else
		{
			printf("NO SIGNAL                     \r");
		}
		
		waitms(200);
	}

}