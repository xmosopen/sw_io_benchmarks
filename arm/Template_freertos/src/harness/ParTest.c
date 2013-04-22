/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"

/* Demo application includes. */
#include "partest.h"
#include "LPC17xx.H"                    /* LPC17xx definitions                */

/*
   When FAST_INT is enabled the fast interupts are used. These are ports:
	P2.10
	P2.11
	P2.12
	P2.13
	(There are 4 fast IO ports)
*/
#define FAST_INT


void vParTestInitialise( void )
{
	EINTInit();
}
/*-----------------------------------------------------------*/

/*****************************************************************************
** Function name:		EINTInit
**
** Descriptions:		Initialize external interrupt pin and
**						install interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/


void EINTNormalInit( void )
{
	LPC_GPIO1->FIODIR |= (1UL<<19);      // P1.18
	LPC_GPIO1->FIODIR |= (1UL<<20);      // P1.18
	LPC_GPIO1->FIODIR |= (1UL<<21);      // P1.18
	LPC_GPIO1->FIODIR |= (1UL<<22);      // P1.18


	LPC_GPIO1->FIODIR |= (1UL<<18);      // P1.18
	LPC_GPIO1->FIOPIN |=  (1UL<<18);	  // turn led D7 off
	//LPC_GPIO1->FIOPIN &=  ~(1UL<<18);	  // turn led D7 on

	LPC_GPIO1->FIODIR |= (1UL<<29);      // P1.29
	//LPC_GPIO1->FIOPIN |=  (1UL<<29);	  // turn led D8 off
	LPC_GPIO1->FIOPIN &=  ~(1UL<<29);	  // turn led D8 on

    LPC_PINCON->PINSEL1 &= ~(0x3<<6); 		//set 0.19 as GPIO
	LPC_PINCON->PINSEL1 &= ~(0x3<<8); 		//set 0.20 as GPIO
	LPC_PINCON->PINSEL1 &= ~(0x3<<10); 		//set 0.21 as GPIO 
	LPC_PINCON->PINSEL1 &= ~(0x3<<12); 		//set 0.22 as GPIO	
	   
	LPC_PINCON->PINMODE1 &= ~(0x3<<6); 		//set 0.19 as GPIO
	LPC_PINCON->PINMODE1 &= ~(0x3<<8); 		//set 0.20 as GPIO
	LPC_PINCON->PINMODE1 &= ~(0x3<<10); 	//set 0.21 as GPIO 
	LPC_PINCON->PINMODE1 &= ~(0x3<<12); 	//set 0.22 as GPIO	

	//input ports	  
	LPC_GPIOINT->IO0IntEnF |= (1 << 19);
	LPC_GPIOINT->IO0IntEnF |= (1 << 20);
	LPC_GPIOINT->IO0IntEnF |= (1 << 21);
	LPC_GPIOINT->IO0IntEnF |= (1 << 22);
							  	
	LPC_GPIOINT->IO0IntEnR |= (1 << 19);
	LPC_GPIOINT->IO0IntEnR |= (1 << 20);
	LPC_GPIOINT->IO0IntEnR |= (1 << 21);
	LPC_GPIOINT->IO0IntEnR |= (1 << 22);

	//IO0IntClr	
	//Writing a 1 into a bit in this write-only register 
	//clears any interrupts for the corresponding port 0 pin						  	
	LPC_GPIOINT->IO0IntClr |= (1 << 19);
	LPC_GPIOINT->IO0IntClr |= (1 << 20);
	LPC_GPIOINT->IO0IntClr |= (1 << 21);
	LPC_GPIOINT->IO0IntClr |= (1 << 22);

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriority(EINT3_IRQn, 5);  //levels with 0 being the highest and 31 being the lowest
	NVIC_EnableIRQ(EINT3_IRQn);      
	
	
}

void EINTFastInit( void )
{

    LPC_GPIO1->FIODIR |= (1UL<<19);      // P1.18
	LPC_GPIO1->FIODIR |= (1UL<<20);      // P1.18
	LPC_GPIO1->FIODIR |= (1UL<<21);      // P1.18
	LPC_GPIO1->FIODIR |= (1UL<<22);      // P1.18

	 LPC_GPIO1->FIODIR |= (1UL<<18); // P1.18
	 LPC_GPIO1->FIOPIN |=  (1UL<<18);	  //turn led D7 off
	 	 LPC_GPIO1->FIODIR |= (1UL<<29); // P1.18
	 LPC_GPIO1->FIOPIN |=  (1UL<<29);	  //turn led D7 off
	 LPC_GPIO1->FIOMASK = 0;
	 LPC_GPIO1->FIOCLR = 0xf;

	// set up the inputs as inputs
	 LPC_GPIO2->FIODIR &= ~(1UL<<10); // P2.10
	 LPC_GPIO2->FIODIR &= ~(1UL<<11); // P2.11
	 LPC_GPIO2->FIODIR &= ~(1UL<<12); // P2.12
	 LPC_GPIO2->FIODIR &= ~(1UL<<13); // P2.13
	   
	 LPC_GPIO2->FIOMASK = 0; 
	 LPC_GPIO2->FIOCLR = 0xf;

	// set the pins as interupts
	 LPC_PINCON->PINSEL4 |= (0x01 << 20); //Set P2.10 as EINT0
	 LPC_PINCON->PINSEL4 |= (0x01 << 22); //Set P2.11 as EINT1
	 LPC_PINCON->PINSEL4 |= (0x01 << 24); //Set P2.12 as EINT2
	 LPC_PINCON->PINSEL4 |= (0x01 << 26); //Set P2.13 as EINT3
						   LPC_SC->EXTINT &=0xFFFFFFFF;
	 LPC_SC->EXTMODE |= 0xf; //EINT0-3 are edge sensitive
	// LPC_SC->EXTINT = 0xf; //Clear Interrupts
	 LPC_SC->EXTPOLAR |= 0xf; //EINT0-3 are rising edge sensitive to begin with
 
	 NVIC_EnableIRQ(EINT0_IRQn);
     NVIC_SetPriority(EINT0_IRQn, 5);
	 NVIC_EnableIRQ(EINT1_IRQn);
     NVIC_SetPriority(EINT1_IRQn, 5);
	 NVIC_EnableIRQ(EINT2_IRQn);
     NVIC_SetPriority(EINT2_IRQn, 5);
	 NVIC_EnableIRQ(EINT3_IRQn);
     NVIC_SetPriority(EINT3_IRQn, 5);
	
}		 
  
void EINTInit( void )
{
#ifdef FAST_INT
	EINTFastInit();
#else
	EINTNormalInit();
#endif
	
}

/*-----------------------------------------------------------*/

extern void EINT0_IRQHandler (void) 
{			   LPC_SC->EXTINT &=0xFFFFFFFF;;
	NVIC_ClearPendingIRQ(EINT0_IRQn);	
		LPC_GPIOINT->IO2IntClr |= (1 << 10);
	LPC_GPIO1->FIOPIN ^=  (1<<19); 	
	LPC_GPIO1->FIOPIN ^=  (1<<18); 		  	 
}
extern void EINT1_IRQHandler (void) 
{			  LPC_SC->EXTINT &=0xFFFFFFFF;
	NVIC_ClearPendingIRQ(EINT1_IRQn); 	
		LPC_GPIOINT->IO2IntClr |= (1 << 11);
	LPC_GPIO1->FIOPIN ^=  (1<<20); 	
	LPC_GPIO1->FIOPIN ^=  (1<<29); 	
}
extern void EINT2_IRQHandler (void) 
{			  LPC_SC->EXTINT &=0xFFFFFFFF;
	NVIC_ClearPendingIRQ(EINT2_IRQn); 
		LPC_GPIOINT->IO2IntClr |= (1 << 12);
	LPC_GPIO1->FIOPIN ^=  (1<<21); 		
	LPC_GPIO1->FIOPIN ^=  (1<<29); 
}

void Fast_EINT3_IRQHandler (void) 
{			 LPC_SC->EXTINT &=0xFFFFFFFF;
	NVIC_ClearPendingIRQ(EINT3_IRQn); 	
		LPC_GPIOINT->IO2IntClr |= (1 << 13);
	LPC_GPIO1->FIOPIN ^=  (1<<22); 	
	LPC_GPIO1->FIOPIN ^=  (1<<18); 	
}

void Normal_EINT3_IRQHandler (void) {

	 if (LPC_GPIOINT->IO0IntStatR & (1 << 19))
	 {  
		LPC_GPIO1->FIOPIN |=  (1<<19); 	 
		LPC_GPIOINT->IO0IntClr |= (1 << 19);
		return;
	 }
	 if (LPC_GPIOINT->IO0IntStatF & (1 << 19))
	 {
		LPC_GPIO1->FIOPIN &=  ~(1<<19); 	 
		LPC_GPIOINT->IO0IntClr |= (1 << 19);
		return;
	 }
	 if (LPC_GPIOINT->IO0IntStatR & (1 << 20))
	 {
		LPC_GPIO1->FIOPIN |=  (1<<20); 	 
		LPC_GPIOINT->IO0IntClr |= (1 << 20);
		return;
	 }
	 if (LPC_GPIOINT->IO0IntStatF & (1 << 20))
	 {
		LPC_GPIO1->FIOPIN &=  ~(1<<20);	 
		LPC_GPIOINT->IO0IntClr |= (1 << 20);
		return;
	 }
	 if (LPC_GPIOINT->IO0IntStatR & (1 << 21))
	 {
		LPC_GPIO1->FIOPIN |=  (1<<21); 	 
		LPC_GPIOINT->IO0IntClr |= (1 << 21);
		return;
	 }
	 if (LPC_GPIOINT->IO0IntStatF & (1 << 21))
	 {
		LPC_GPIO1->FIOPIN &=  ~(1<<21);	 
		LPC_GPIOINT->IO0IntClr |= (1 << 21);
		return;
	 }
	 
	 if (LPC_GPIOINT->IO0IntStatR & (1 << 22))
	 {
		LPC_GPIO1->FIOPIN |=  (1<<22); 	 
		LPC_GPIOINT->IO0IntClr |= (1 << 22);
		return;
	 }
	 if (LPC_GPIOINT->IO0IntStatF & (1 << 22))
	 {
		LPC_GPIO1->FIOPIN &=  ~(1<<22);	 
		LPC_GPIOINT->IO0IntClr |= (1 << 22);
		return;
	 }
	 
	
}

extern void EINT3_IRQHandler (void) 
{
#ifdef FAST_INT
	Fast_EINT3_IRQHandler();
#else
	Normal_EINT3_IRQHandler();
#endif	
}


/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned long ulLEDIn, signed long xValue )
{
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned long ulLEDIn )
{
}
/*-----------------------------------------------------------*/

long lParTestGetLEDState( void )
{
	return 0;
}
/*-----------------------------------------------------------*/

void vParTestSetLEDState( long lState )
{
}
/*-----------------------------------------------------------*/

