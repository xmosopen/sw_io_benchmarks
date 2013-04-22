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

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the 
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt 
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt 
 * timing.  The maximum measured jitter time is latched in the usMaxJitter 
 * variable, and displayed on the LCD by the 'Check' as described below.  
 * The fast interrupt is configured and handled in the timer_test.c source 
 * file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the LCD directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of 
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting 
 * for messages - waking and displaying the messages as they arrive.  The LCD
 * task is defined in lcd.c.  
 * 
 * "Check" task -  This only executes every three seconds but has the highest 
 * priority so is guaranteed to get processor time.  Its main function is to 
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write "FAIL #n" to the LCD (via the LCD task).  If all the demo tasks are 
 * executing with their expected behaviour then the check task writes the max
 * jitter time to the LCD (again via the LCD task), as described above.
 */

/* Standard includes. */
#include <stdio.h>
#include "p33FJ256MC710A.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo application includes. */
#include "partest.h"

/*
 * The check task as described at the top of this file.
 */
void vTestTask( void *pvParameters );
void cCT_InOut( void *pvParameters );
void vDummyTask( void *pvParameters );
void vADCTask( void *pvParameters );

/*
 * Setup the processor ready for the demo.
 */
void prvSetupHardware( void );

void __attribute__((interrupt, auto_psv)) _IC3Interrupt(void);
void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void);

int ADC1Result;

/*-----------------------------------------------------------*/

/*
 * Create the demo tasks then start the scheduler.
 */
int main( void )
{
	char i, numDummyTasks=0;

	/* Configure any hardware required */
	prvSetupHardware();

	/* Create a Test Task */
	xTaskCreate( vTestTask, (signed char *) "Test1", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
//	xTaskCreate( vADCTask, (signed char *) "ADC", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	//NOTE: Comment ConfigIC3 (ParTest.c) if xTaskCreate(cCT_InOut, ...) is used
	xTaskCreate( cCT_InOut, (signed char *) "ctIO", configMINIMAL_STACK_SIZE, NULL, 1, NULL ); //3

	for(i=0; i<numDummyTasks; i++)
		xTaskCreate( vDummyTask, (signed char *) "Dummy", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	/* Finally start the scheduler. */
	vTaskStartScheduler();
	//NOTE: To disable the timer for the scheduler go to prvSetupTimerInterrupt (port.c)

//	while(1); //NOTE: If vTaskStartScheduler is not used then uncomment while(1)

	/* Will only reach here if there is insufficient heap available to start
	the scheduler. */
	return 0;
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vTestTask( void *pvParameters )
{
	char flag = 0;

	/* Used to wake the task at the correct frequency. */
	portTickType xLastExecutionTime;

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount(); 

	while(1)
	{
		/* Wait until it is time for the next cycle. */
		vTaskDelayUntil( &xLastExecutionTime, ((portTickType)1000 / portTICK_RATE_MS) ); // 1second

		if(!flag)
		{
			vParTest_LED_D10(1);
			flag = 1;
		}
		else
		{
			vParTest_LED_D10(0);
			flag = 0;
		}
	}
}
/*-----------------------------------------------------------*/

void cCT_InOut( void *pvParameters )
{
	/* Used to wake the task at the correct frequency. */
	portTickType xLastExecutionTime;

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount(); 

	while(1)
	{
		/* Wait until it is time for the next cycle. */
//		vTaskDelayUntil( &xLastExecutionTime, ((portTickType) 1 / portTICK_RATE_MS) ); //1ms

		if(RX)
			TX = 1;
		else
			TX = 0;

//		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

void vDummyTask( void *pvParameters )
{
	int i, value=0;

	while(1)
	{
		for(i=0; i<1000; i++)
			value++;
	}
}
/*-----------------------------------------------------------*/

void vADCTask( void *pvParameters )
{
	/* Used to wake the task at the correct frequency. */
	portTickType xLastExecutionTime;

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount(); 

	while(1)
	{
		/* Wait until it is time for the next cycle. */
		vTaskDelayUntil( &xLastExecutionTime, ((portTickType)10 / portTICK_RATE_MS) ); // 10ms

		if(ADC1Result > 0x200)
			ADC_LED = 1;
		else
			ADC_LED = 0;
	}
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	while(1);
}
/*-----------------------------------------------------------*/

// Capture Interrupt Service Routine
void __attribute__((interrupt, auto_psv)) _IC3Interrupt(void)
{

	TXi = RXi;

	IFS2bits.IC3IF = 0;
}
/*-----------------------------------------------------------*/

// ADC Interrupt Service Routine
void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void)
{

	ADC1Result = ADC1BUF0;

    IFS0bits.AD1IF = 0;		// Clear the ADC1 Interrupt Flag
}
/*-----------------------------------------------------------*/
