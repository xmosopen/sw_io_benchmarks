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

/* Scheduler includes. */
#include "FreeRTOS.h"

/* Demo app includes. */
#include "partest.h"

//-------------------- MICROCONTROLLER FUSES -------------------- 
_FGS(GSS_OFF & GWRP_OFF);

//* FOSC = 80MHz -> FCY = 40MHz (40MIPs)
_FOSCSEL(IESO_OFF & FNOSC_FRCPLL);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);

_FWDT(FWDTEN_OFF); 
_FPOR(FPWRT_PWR1); 
_FICD(JTAGEN_ON & ICS_PGD1); //Different from dsPIC33FJ32MC204

#define  NUM_CHS2SCAN			4		// Number of channels enabled for channel scan

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

void vParTestInitialise( void )
{
	// Configure I/O Pins
	PORT_E_Init();
	PORT_F_Init();
	PORT_D_Init();

	// Configure Oscillator
	ConfigOSC();

	// Configure Interrupt
//	ConfigIC3(); //NOTE: Comment xTaskCreate(cCT_InOut, ...) (main.c) if ConfigIC3 is used

	// Configure ADC
//	ConfigADC();
}
/*-----------------------------------------------------------*/

void vParTest_LED_D10( unsigned char xValue )
{
	if(xValue==1)
		PORTEbits.RE0 = 1;
	else
		PORTEbits.RE0 = 0;
}
/*-----------------------------------------------------------*/

void vParTest_LED_D14( unsigned char uxLED )
{
	if(uxLED==1)
		PORTEbits.RE4 = 1;
	else
		PORTEbits.RE4 = 0;
}
/*-----------------------------------------------------------*/

void PORT_D_Init( void )
{
	//Initialize PORT D
	LATD = 0;			//Initial state
	TRISD = 0xFF;		//All inputs
}
/*-----------------------------------------------------------*/

void PORT_E_Init( void )
{
	//Initialize PORT E
	LATE = 0;			//Initial state
	TRISE = 0;			//All outputs
}
/*-----------------------------------------------------------*/

void PORT_F_Init( void )
{
	//Initialize PORT F
	LATF = 0;			//Initial state
	TRISF = 0xFF;		//All inputs
}
/*-----------------------------------------------------------*/

void ConfigOSC( void )
{
	// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 43; // M = 43 for CF = 7.37MHz / M = 40 for CF = 8MHz
	CLKDIVbits.PLLPOST=0; // N2 = 2
	CLKDIVbits.PLLPRE=0; // N1 = 2

	//FRC
	OSCTUN = 0b000000; //Center Frequency = 7.37MHz nominal
	//OSCTUN = 0b011001; //Center Frequency = 8MHz;

	// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);
	__builtin_write_OSCCONL(0x01);

	// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b001);

	// Wait for PLL to lock
	while(OSCCONbits.LOCK != 1){};
}
/*-----------------------------------------------------------*/

void ConfigIC3( void )
{
	// Initialize Capture Module
	IC3CONbits.ICM = 0b00; // Input capture module turned off
	IC3CONbits.ICI = 0b00; // Interrupt on every capture event
	IC3CONbits.ICM = 0b01; // Capture mode, every edge (rising and falling)
	// 0b011 - Rising

	IPC9bits.IC3IP = 1; // Setup IC3 interrupt priority level
	IFS2bits.IC3IF = 0; // Clear IC3 Interrupt Status Flag
	IEC2bits.IC3IE = 1; // Enable IC3 interrupt
}
/*-----------------------------------------------------------*/

void ConfigADC( void )
{
	TRISBbits.TRISB8 = 1; //Input

	AD1CON1bits.FORM   = 0;		// Data Output Format: Signed Fraction (Q15 format)
	AD1CON1bits.SSRC   = 7;		// Sample Clock Source: GP Timer starts conversion
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 0;		// 10-bit ADC operation

	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
//	AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
									// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

	AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);	// 4 ADC Channel is scanned

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
	AD1CSSLbits.CSS8=1;		// Enable AN8 for channel scan
 
 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;
 	AD1PCFGLbits.PCFG8 = 0;	// AN8 as Analog Input

    IFS0bits.AD1IF = 0;			// Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;			// Enable A/D interrupt 
    AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}
/*-----------------------------------------------------------*/
