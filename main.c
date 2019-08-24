
/*
*       ATtiny13 AutoNightLights Module
*       using ordinary  led to measure ambient light
*       PIR sensor utilyzed to detect human presence
*       an UART output for easy calibration
*                               by Larry_B 'intenseC'
*                              ver. 001beta
*                          Aug 2k18 - Revised
*/
//*****************************************************************************

//		#define  __USART
		
		/*   timing values   */
		#define GAP   12   // timer compare match
		

		#define M_LIMIT  76500  
                #define NIGHT   1
		#define DAY     0
		#define TWILIGHT   580  // ambient darkness threshold 
		#define C_LIMIT 50000  //    1/2 s
		#define THRESH 4
	
		#define T_DELAY   15         //  minutes
 		#define HW_TMR
 
//*****************************************************************************
 	 #define F_CPU 9600000UL
//*****************************************************************************
          /*   macroses   */  
        #define sbi(var, mask) ((var) |= (1 << mask))  // Set bit function
        #define cbi(var, mask) ((var) &=  ~(1 << mask)) // Clear bit function
//*****************************************************************************
//place includes here
//*****************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <string.h>

	  #ifdef __USART
	  #include "UART.h"
	  #endif

//*****************************************************************************
// define vars here
//*****************************************************************************
   char buf[8];
   uint8_t daytime = DAY; 
   uint8_t active = 0;
   volatile uint8_t proxim = 0;  // bool to mark human presense
   volatile  uint16_t  edge = 0, stage = 0; 
   volatile uint8_t frame = 0;
   volatile  uint8_t  onesec = 0;
   volatile  uint16_t clock = 0;
   uint16_t adcReg = 0;
   volatile   uint8_t  minutes = 0, waitMins = 0;
//*****************************************************************************
// define pins below
//*****************************************************************************


#define PORT_LED_A                   PORTB
#define LED_A	                     PINB                        //  LED Anode
#define PIN_LED_A	             PB0
#define LED_A_DIR	             DDRB

#define PORT_PIR_SENS        	     PORTB
#define PIR_SENS	             PINB                        // INT from an PIR sensor
#define PIN_PIR_SENS	    	     PB1                            //

#define PORT_P_SW                    PORTB
#define P_SW	                     PINB                         // Fet switch
#define PIN_P_SW	             PB3
#define PORT_P_DIR	             DDRB

#define PORT_LED_C                   PORTB
#define LED_C	                     PINB                         
#define PIN_LED_C	             PB2                     //   ADC1     // LED Cathode
#define LED_C_DIR	             DDRB                    //

//*****************************************************************************
       // Led Cathode and  Led Anode as output
       // Led Cathode high / Led Anode low
   #define LED_CHARGE \
	    { \
	   LED_C_DIR |= (1 << PIN_LED_C);	LED_A_DIR |= (1 << PIN_LED_A); \
	   PORT_LED_C |= (1 << PIN_LED_C); PORT_LED_A &= ~(1 << PIN_LED_A); \
	    } while(0)  // while() to swallow last semicolon
		       
				// Led Cathode and  Led Anode as output
       // Led Cathode low / Led Anode high
   #define LED_DISCHARGE \
	    { \
	   LED_C_DIR |= (1 << PIN_LED_C);	LED_A_DIR |= (1 << PIN_LED_A); \
	   PORT_LED_C &= ~(1 << PIN_LED_C); PORT_LED_A |= (1 << PIN_LED_A); \
	    } while(0)

		 // Led Cathode set as input; Led Anode remains as output
         // Led Cathode pullup off,  Led Anode low 
	#define LED_READ \
	     { \
	   LED_C_DIR &= ~(1 << PIN_LED_C); LED_A_DIR |= (1 << PIN_LED_A); \
       PORT_LED_C &= ~(1 << PIN_LED_C); PORT_LED_A &= ~(1 << PIN_LED_A); \
	     } while(0)

//*****************************************************************************
// protos

 void daylight_measure(void);

//*****************************************************************************

void init_io(void) 
{
 PORTB =  0b00000000;   	/* activate pull-ups on  - - - NONE*/
 DDRB   = 0b00000000; 		/*  pins   output  ALL input */  
 }


void timer_init (void)
{   
	 cli();                    // disable global interrupts
     
	#ifdef  HW_TMR

          TCCR0B |= (0 << CS02) | (1 << CS01) | (0 << CS00);         // timer0 with 8 prescaler - 1200 000 Hz.
          TCCR0A |= (1 << WGM01); 	        // CTC
         
         OCR0A   =   GAP;
		 TCNT0 = 0;
		 TIMSK0 |= (1 <<  OCIE0A);  
       #else
     TCCR0B |=  (1 << CS00);  
       #endif

	   #ifdef  INTER 
        GIMSK |= _BV(INT0);
        MCUCR |= (0 << ISC01) | (1 << ISC00);  //  any change
       #endif
        ADMUX |= (0 << MUX1) | (1 << MUX0) | (0 << REFS0);  // adc1 pin / vcc as vref
        ADCSRA |= ( 1 << ADPS1 ) | ( 1 << ADPS0 ) | ( 1 << ADEN ); //  prescaler = clock / 128
      
	  sei();                 // Enable global interrupts
}
//*****************************************************************************
       #ifdef  HW_TMR
											   // 100kHz / 0.01mS / 10uS
	 ISR(TIM0_COMPA_vect)  
  {
    edge++;
    if(++clock > C_LIMIT) {  clock = 0;  if(++onesec > 59) { onesec = 0; minutes++; waitMins++; }     
	                                                         }
      daylight_measure();      stage++;   
  }
       #else  
	   	 ISR(TIM0_COMPA_vect)  
           {
                stage++;    
           }
       #endif
     #ifdef  INTER 
	 ISR(INT0_vect)
  {
        proxim = 1;
  }
       #endif
//*****************************************************************************
               // read adc 
    int adc_read(int ch)
   {
	ch &= 0x07;  
	ADMUX = (ADMUX & 0xF8) | ch; 
	ADCSRA |= (1<<ADSC);
	while( ADCSRA & (1<<ADSC));
	return(ADC);
                          }
//*****************************************************************************



void timebits(void)
{
       #ifndef  HW_TMR
	  edge++;
     if(++clock > M_LIMIT) {  clock = 0;  if(++onesec > 59) { onesec = 0; minutes++; waitMins++; }           }
     daylight_measure();       stage++;
       #endif
}

void compute(void)
{
  static unsigned char presc = 3;
            if(--presc == 0)    {  presc = 3; 
		   		  #ifdef __USART
			      ltoa(adcReg, buf, 10);
              strcat(buf, " " );
					       uart_puts(buf);            //  effective baud 56000
	               #endif
              if(waitMins >= 1) {                  // check daytime once a minute
					  if(adcReg > TWILIGHT) daytime = NIGHT; else daytime = DAY; 	
			    waitMins = 0; }
                                }
}



void daylight_measure(void)
{
	   switch (stage)
           {
            case 0: 
	              LED_DISCHARGE;
			          break;  
            case 5: 
	              LED_CHARGE; 
			          break;  
            case 100: 
	              LED_READ;     
				  break;   
            case 6200:  
                adcReg = adc_read(1);
				  break;
	        }
         if( stage > C_LIMIT )   stage = 0;
}

//****************************************************************************
//*****************************************************************************

void apply(void)
{
       compute();
    	 #ifndef  INTER
	 if(bit_is_set(PIR_SENS, PIN_PIR_SENS)) proxim = 1;
         #endif
	  if(proxim) {
	  minutes = 0;        proxim = 0;
       if(daytime == NIGHT)  {
	 PORT_P_SW |= (1 << PIN_P_SW); // active = 1; 
	      }           
	}             

	if(minutes >= T_DELAY ) { minutes = 0;  PORT_P_SW &= ~(1 << PIN_P_SW);   }
      }

	 void tick(void)
{
           #ifndef  HW_TMR 
           timebits();
           #endif
 if(edge > 49999)  // 1/2 sec frame
		{
        edge = 0;
        apply();
		    wdt_reset();
	                          }
}

//*****************************************************************************

int main(void)      
{ 
  init_io();
  timer_init();
  sei();
  PORT_P_DIR |= (1 << PIN_P_SW);  // output
 	  #ifdef HW_TMR
while(onesec  < 4);
PORT_P_SW |= (1 << PIN_P_SW);
while(onesec  < 10);
	  #endif
PORT_P_SW &= ~(1 << PIN_P_SW);
   wdt_enable(WDTO_2S);

//*****************************************************************************
for(;;)  // eternal loop start
	        {  
	           tick();
        	}  // for() out
    }  // main out
//*****************************************************************************
//*****************************************************************************
