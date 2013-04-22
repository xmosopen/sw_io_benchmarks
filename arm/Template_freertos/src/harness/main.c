/**
 * @file		main.c
/**********/

#include "lpc17xx_libcfg.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

const unsigned long LED8 = 1<<29;
const unsigned long LED7 = 1<<18;

//extern volatile uint32_t ADC0IntDone;

/** Characteristic Tasks **/
void cCT_InOut( void *pvParameters );

/** WorkLoad Tasks **/
void WT_Task( void *pvParameters );

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

int main (void)
{

	/* Configure the hardware for use by this demo. */
	prvSetupHardware();

	//NOTE: Comment EINT1Init (ParTest.c) if xTaskCreate(cCT_InOut, ...) is used
//	xTaskCreate( cCT_InOut, ( signed char * ) "ctIO", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY, NULL );

	xTaskCreate( WT_Task, ( signed char * ) "Dummy", configMINIMAL_STACK_SIZE, ( void * ) NULL, tskIDLE_PRIORITY, NULL );
	
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	while(1);

	return 0;
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void cCT_InOut( void *pvParameters )
{

}
/*-----------------------------------------------------------*/

void WT_Task( void *pvParameters )
{
	int i;
	int array[16];

	while(1)
	{
		array[i]++;
		array[i] &= 0x1f;
	//	LPC_GPIO1->FIOPIN ^=  LED7;
		i++;
		i &= 0xf;
	}
}
/*-----------------------------------------------------------*/

/*
	SCHEDULER SETUP
*/
void vConfigureTimerForRunTimeStats( void )
{
const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;

	/* This function configures a timer that is used as the time base when
	collecting run time statistical information - basically the percentage
	of CPU time that each task is utilising.  It is called automatically when
	the scheduler is started (assuming configGENERATE_RUN_TIME_STATS is set
	to 1). */

	/* Power up and feed the timer. */
	LPC_SC->PCONP |= 0x02UL;
	LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & (~(0x3<<2))) | (0x01 << 2);

	/* Reset Timer 0 */
	LPC_TIM0->TCR = TCR_COUNT_RESET;

	/* Just count up. */
	LPC_TIM0->CTCR = CTCR_CTM_TIMER;

	/* Prescale to a frequency that is good enough to get a decent resolution,
	but not too fast so as to overflow all the time. */
	LPC_TIM0->PR =  ( configCPU_CLOCK_HZ / 10000UL ) - 1UL;

	/* Start the counter. */
	LPC_TIM0->TCR = TCR_COUNT_ENABLE;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	while(1);
}
/*-----------------------------------------------------------*/

#ifdef  DEBUG
/*******************************************************************************
* @brief		Reports the name of the source file and the source line number
* 				where the CHECK_PARAM error has occurred.
* @param[in]	file Pointer to the source file name
* @param[in]    line assert_param error line source number
* @return		None
*******************************************************************************/
void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
#endif

