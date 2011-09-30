


/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
/* Standard demo includes. */
#include "ParTest.h"
#include "flash.h"

//#include "msp430x26x.h"
//#include "io430.h"


/* Demo task priorities. */
//#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )
//#define mainCOM_TEST_PRIORITY			( tskIDLE_PRIORITY + 2 )
//#define mainQUEUE_POLL_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainLED_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )


/* The constants used in the calculation. */
#define intgCONST1				( ( portLONG ) 123 )
#define intgCONST2				( ( portLONG ) 234567 )
#define intgCONST3				( ( portLONG ) -3 )
#define intgCONST4				( ( portLONG ) 7 )
#define intgEXPECTED_ANSWER		( ( ( intgCONST1 + intgCONST2 ) * intgCONST3 ) / intgCONST4 )


portBASE_TYPE xLocalError = pdFALSE;
volatile unsigned portLONG ulIdleLoops = 0UL;

///BEGIN DCO Clock
/* From: msp430x261x_fll_01.c (slac151d)
Set DCO clock to (Delta)*(4096) using software FLL. DCO clock
is output on P1.4 as SMCLK.  DCO clock, which is the selected SMCLK source
for timer_A is integrated over LFXT1/8 (4096) until SMCLK is equal to Delta
CCR2 captures ACLK.  To use Set_DCO Timer_A must be operating in continous
mode.  Watch crystal for ACLK is required for this example.  Delta must be
kept in a range that allows possible DCO speeds.  Minimum Delta must ensure
that Set_DCO loop can complete within capture interval. Maximum delta can be 
calculated by f(DCOx7) / 4096.  f(DCOx7) can be found in device specific
datasheet. ACLK = LFXT1/8 = 32768/8, MCLK = SMCLK = target DCO
External watch crystal installed on XIN XOUT is required for ACLK */
//formula is DELTA*ACLK = frequency you want!!
#define DELTA 2930                      // target DCO = DELTA*(4096) = 12MHz
//#define DELTA 2445                        // target DCO = DELTA*(4096) = 10MHz
//#define DELTA 900                       // target DCO = DELTA*(4096) = 3686400
//#define DELTA 256                       // target DCO = DELTA*(4096) = 1048576
//#define DELTA 70                        // target DCO = DELTA*(4096) = 286720

volatile unsigned int i;
void Set_DCO(void);
///END DCO Clock

// Uncomment the following to test the LEDs

/*
 * Perform the hardware setup required by the ES449 in order to run the demo
 * application.
 */
static void prvSetupHardware( void );

#define mainON_BOARD_LED_BIT	( ( unsigned portCHAR ) 0x04 )

static void prvToggleOnBoardLED( void );

static void prvBeep( void );

static void prvBeep (void) {
#define COUNTERLEN 500 // ~440 Hz
#define NUMCYCLES 250
#define P36 0x40 // 0b01000000, pin 3.6
    unsigned i, cyc;

    P3DIR |= P36; // P3.6 is an output pin; all others are inputs

    for (cyc = 0; cyc != NUMCYCLES; ++cyc) {
        P3OUT ^= P36; // toggle P3.6
        i = COUNTERLEN;
        do { --i; } while (i != 0);
    }

    printf("Beep\n");                       // SW Delay

}


static void prvToggleOnBoardLED( void )
{
  /* Toggle the state of the single genuine on board LED. 
        static unsigned portSHORT sState = pdFALSE;
      
	
	if( sState )	
	{
		P4OUT |= mainON_BOARD_LED_BIT;
	}
	else
	{
		P4OUT &= ~mainON_BOARD_LED_BIT;
	}
        sState = !sState;
  */
    
    P4OUT ^= 0x04;
  
    printf("Blink\n");                       // Delay
    
}
/*-----------------------------------------------------------*/


static void prvSetupHardware( void )
{
	/* Stop the watchdog. */
	WDTCTL = WDTPW + WDTHOLD;
         
        
        for (i = 0; i < 0xfffe; i++);             // Delay for XTAL stabilization
        P1DIR |= 0x10;                            // P1.4 output
        P1SEL |= 0x10;                            // P1.4 SMCLK output
        Set_DCO();   

	/* Setup DCO+ for ( xtal * D * (N + 1) ) operation. */
	//FLL_CTL0 |= DCOPLUS + XCAP18PF;

	/* X2 DCO frequency, 8MHz nominal DCO */
	//SCFI0 |= FN_4;

	/* (121+1) x 32768 x 2 = 7.99 Mhz */
	//SCFQCTL = mainMAX_FREQUENCY;

	/* Setup the IO.  This is just copied from the demo supplied by SoftBaugh
	 for the ES449 demo board. */
	//P1SEL = 0x32;
	P2SEL = 0x00;
	P3SEL = 0x00;
	P4SEL = 0xFC;
	P5SEL = 0xFF;
}

int main( void )
    {
      
        /* Setup the microcontroller hardware for the demo. */
        //prvSetupHardware(); ---> this for some reason bloks the LED task but not other tasks
	vParTestInitialise();

        /* Start the standard demo application tasks. */
	//vStartLEDFlashTasks( mainLED_TASK_PRIORITY );
        
        prvToggleOnBoardLED();
        prvBeep();

        /* All other functions that create tasks are commented out.
        
            vCreatePollQTasks();
            vCreateComTestTasks();
            Etc.

            xTaskCreate( vCheckTask, "check", STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        */

        //Start the scheduler. 
        vTaskStartScheduler();

        /* Should never get here! */
        return 0;
    }

/* The idle hook is just a copy of the standard integer maths tasks.  See
Demo/Common/integer.c for rationale. */

void vApplicationIdleHook( void )
{
/* These variables are all effectively set to constants so they are volatile to
ensure the compiler does not just get rid of them. */
volatile portLONG lValue;

	/* Keep performing a calculation and checking the result against a constant. */
	for( ;; )
	{
		/* Perform the calculation.  This will store partial value in
		registers, resulting in a good test of the context switch mechanism. */
		lValue = intgCONST1;
		lValue += intgCONST2;

		/* Yield in case cooperative scheduling is being used. */
		#if configUSE_PREEMPTION == 0
		{
			taskYIELD();
		}
		#endif

		/* Finish off the calculation. */
		lValue *= intgCONST3;
		lValue /= intgCONST4;

		/* If the calculation is found to be incorrect we stop setting the
		TaskHasExecuted variable so the check task can see an error has
		occurred. */
		if( lValue != intgEXPECTED_ANSWER ) /*lint !e774 volatile used to prevent this being optimised out. */
		{
			/* Don't bother with mutual exclusion - it is only read from the
			check task and never written. */
			xLocalError = pdTRUE;
		}
		/* Yield in case cooperative scheduling is being used. */
		#if configUSE_PREEMPTION == 0
		{
			taskYIELD();
		}
		#endif

        ulIdleLoops++;

        /* Place the processor into low power mode. */
        LPM3;
	}
}

/* The MSP430X port uses this callback function to configure its tick interrupt.
This allows the application to choose the tick interrupt source.
configTICK_VECTOR must also be set in FreeRTOSConfig.h to the correct
interrupt vector for the chosen tick interrupt source.  This implementation of
vApplicationSetupTimerInterrupt() generates the tick from timer A0, so in this
case configTICK_VECTOR is set to TIMER0_A0_VECTOR. */
void vApplicationSetupTimerInterrupt( void )
{
const unsigned short usACLK_Frequency_Hz = 32768;

	/* Ensure the timer is stopped. */
	TA0CTL = 0;

	/* Run the timer from the ACLK. */
	TA0CTL = TASSEL_1;

	/* Clear everything to start with. */
	TA0CTL |= TACLR;

	/* Set the compare match value according to the tick rate we want. */
	TA0CCR0 = usACLK_Frequency_Hz / configTICK_RATE_HZ;

	/* Enable the interrupts. */
	TA0CCTL0 = CCIE;

	/* Start up clean. */
	TA0CTL |= TACLR;

	/* Up mode. */
	TA0CTL |= MC_1;
}

void Set_DCO(void)                          // Set DCO to selected frequency
{
  unsigned int Compare, Oldcapture = 0;

  BCSCTL1 |= DIVA_3;                        // ACLK= LFXT1CLK/8 = 4096Hz
  TACCTL2 = CM_1 + CCIS_1 + CAP;            // CAP, ACLK
  TACTL = TASSEL_2 + MC_2 + TACLR;          // SMCLK, cont-mode, clear

  while (1)
  {
    while (!(CCIFG & TACCTL2));             // Wait until capture occured
    TACCTL2 &= ~CCIFG;                      // Capture occured, clear flag
    Compare = TACCR2;                       // Get current captured SMCLK
    Compare = Compare - Oldcapture;         // SMCLK difference
    Oldcapture = TACCR2;                    // Save current captured SMCLK

    if (DELTA == Compare)
      break;                                // If equal, leave "while(1)"
    else if (DELTA < Compare)
    {
      DCOCTL--;                             // DCO is too fast, slow it down
      if (DCOCTL == 0xFF)                   // Did DCO roll under?
        if (BCSCTL1 & 0x0f)
          BCSCTL1--;                        // Select lower RSEL
    }
    else
    {
      DCOCTL++;                             // DCO is too slow, speed it up
      if (DCOCTL == 0x00)                   // Did DCO roll over?
        if ((BCSCTL1 & 0x0f) != 0x0f)
          BCSCTL1++;                        // Sel higher RSEL
    }
  }
  TACCTL2 = 0;                              // Stop TACCR2
  TACTL = 0;                                // Stop Timer_A
  BCSCTL1 &= ~DIVA_3;                       // ACLK = LFXT1CLK = 32.768KHz
}
