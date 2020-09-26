/*----------------------------------------------------------------------------
    Code for Project D: DC Motor Control 
   
	 	This project 
		- Controls the speed and direction of DC motor.
		- Measures the speed and show in debugger. 
		- Senses the direction of the motor. 
		- Shows the speed in bands using RGB LED.

This project file includes
    TPM_PWM.c/.h  configuration code for TPM Input Capture and PWM
    GPIO.c/.h     configuration code for GPIO
    main.c        this file
      - ISR for TPM1
      - Queue for interval messages
      - button polling task 
      - frequency calc task
 *---------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <cmsis_os2.h>
#include <MKL25Z4.H>
#include "../include/gpio.h"
#include "../include/tpmPwm.h"

osEventFlagsId_t evtFlags ; // event flags

osThreadId_t t_freq;            /*  task id of task to calculate freq */
osThreadId_t t_button;          /*  task id of task to poll button */
osThreadId_t t_button2;         /*  task id of task to poll button */
osThreadId_t t_directionMotor;  /*  task id of task to control direction of motor */ 

/* -------------------------------------
    Message queue
   ------------------------------------- */

osMessageQueueId_t interval_q_id ;           // Declare an ID for the message queue

/* -------------------------------------
    TPM1 interrupt handler

    Check each channel to see if caused interrupt
      - Only channel 0 in use
    Also check overflow flag
          - Both may be set
    Reset by writing 1 to the flag register
   ------------------------------------- */

// Data updated by ISR
uint32_t count = 0 ; // last count (or zero)
uint32_t cumul = 0 ; // cumul count since message
uint32_t interval[8] = {0, 0, 0, 0, 0, 0, 0, 0} ;
                         // array of intervals


int nextIdx = 0 ;        // index to next spaces
int newCount ;

/*  Update Interval

    Called to update the interval and, periodically,
    send a message to the frequency task
*/
void updateInterval() {
    // if no previous count has been recorded no interval
    // can be calculated
    if (!count) {
        count = TPM1->CONTROLS[0].CnV ;
        return ;
    }
    // calculate new interval
    newCount = TPM1->CONTROLS[0].CnV ;

    // handle case where TPM counter has wrapped
    if (newCount < count) {
        interval[nextIdx] = 0x10000 + newCount - count ;
    } else {
        interval[nextIdx] = newCount - count ;
    }
    count = newCount ;

    // update the interval since last message
    cumul = cumul + interval[nextIdx] ;
    
    // next index
    nextIdx = (nextIdx + 1) % 8 ;

    // send message if cumul time greater than threashold and
    // array is full
    if (cumul > MESSINTERVAL) {
        // check 8 non zero values
        int c = 0 ;        // count of non-zero intervals
        uint32_t sum = 0 ; // no need to divide by 8 here
        for (int i = 0; i < 8; i++) {
            if (interval[i] > 0) c++ ;
            sum += interval[i] ;
        }
        if (c == 8) {
            // send interval message
            osMessageQueuePut(interval_q_id, &sum, 0, NULL);  // Send Message
            // may fail and should be checked
        }
    }
}

/*----------------------------------------------------------- 
The following four functions are for controlling the DC motor
-----------------------------------------------------------*/

void forwardMotor() {
	PTB->PSOR |= MASK(IP1); 
	PTB->PCOR |= MASK(IP2); 
}

void backwardMotor() {
	PTB->PCOR |= MASK(IP1); 
	PTB->PSOR |= MASK(IP2); 
}

void stopMotorFreeWheel() {
	PTB->PCOR |= MASK(IP1); 
	PTB->PCOR |= MASK(IP2); 
}

void stopMotorFast() {
	PTB->PSOR |= MASK(IP1); 
	PTB->PSOR |= MASK(IP2); 
}


int compare = 0; 
// Handler for TPM1 interrupts
//   - only channel 0 channel interrupt used
void TPM1_IRQHandler(void) {
    
    // clear pending interrupts
    NVIC_ClearPendingIRQ(TPM1_IRQn);

    // Channel 0 flag
    if (TPM_CnSC_REG(TPM1,0) & TPM_CnSC_CHF_MASK) {
        // clear it
        TPM_CnSC_REG(TPM1,0) |= TPM_CnSC_CHF(1); 
        // Sensing the direction
				if (!isPressed3()) directionLEDOnOff(LED_ON); 
				else if (isPressed3()) directionLEDOnOff(LED_OFF); 
				updateInterval() ;
    }

    // Channel 1 flag
    if (TPM_CnSC_REG(TPM1,1) & TPM_CnSC_CHF_MASK) {
        // clear it
        TPM_CnSC_REG(TPM1,1) |= TPM_CnSC_CHF(1) ;
        // CPU doesn't take any action
    }

    // Overflow flag
    if (TPM1->SC & TPM_SC_TOF_MASK) {
        // clear it
        TPM1->SC |= TPM_SC_TOF(1) ;
        // CPU doesn't take any action
    }
}


/*----------------------------------------------------------------------------
  Polling the Button

  The button is polled using a task with a delay.
    The polling rate is 25Hz
 *----------------------------------------------------------------------------*/
#define BUTTONUP (0)
#define BUTTONDOWN (1)
#define BUTTONBOUNCE (2)
#define BOUNCEDELAY (3)


double PWMdutyCycle[] = {0, 0.55, 0.70, 0.80, 1}; 
uint16_t PWM_index=0; 

void buttonTask(void *arg) {
    int buttonState = BUTTONUP ; // current state of the button
    int bounceCounter = BOUNCEDELAY ; // counter for debounce
	forwardMotor(); 
    while (1) {
        if (bounceCounter > 0) bounceCounter-- ;
        switch (buttonState) {
            case BUTTONUP:
                if (isPressed()) {
                    buttonState = BUTTONDOWN ;
										PWM_index = PWM_index + 1; 
										if (PWM_index == 5) PWM_index = 0;
										setPWMDuty(PWM_DUTY_MAX*PWMdutyCycle[PWM_index]);
                }
                break ;
            case BUTTONDOWN:
                if (!isPressed()) {
                    buttonState = BUTTONBOUNCE ;
                    bounceCounter = BOUNCEDELAY ;
                }
                break ;
            case BUTTONBOUNCE:
                if (isPressed()) {
                    buttonState = BUTTONDOWN ;
                }
                else if (bounceCounter == 0) {
                    buttonState = BUTTONUP ;
                }
                break ;
        }
        osDelay(40) ; // delay
    }
}

void button2Task(void *arg) {
    int buttonState2 = BUTTONUP ; // current state of the button
    int bounceCounter2 = BOUNCEDELAY ; // counter for debounce
    
   
    while (1) {
        if (bounceCounter2 > 0) bounceCounter2-- ;
        switch (buttonState2) {
            case BUTTONUP:
                if (isPressed2()) {
                    buttonState2 = BUTTONDOWN ; 
                    osEventFlagsSet(evtFlags, MASK(PRESS_EVT));
                }
                break ;
            case BUTTONDOWN:
                if (!isPressed2()) {
                    buttonState2 = BUTTONBOUNCE ;
                    bounceCounter2 = BOUNCEDELAY ;
                }
                break ;
            case BUTTONBOUNCE:
                if (isPressed2()) {
                    buttonState2 = BUTTONDOWN ;
                }
                else if (bounceCounter2 == 0) {
                    buttonState2 = BUTTONUP ;
                }
                break ;
        }
        osDelay(40) ; // delay
    }
}

/*--------------------------------------------------------------
 *     Task t_freq - calculate the frequency
 *--------------------------------------------------------------*/

int RPM = 0; 
uint32_t RPM_int = 0, frequency = 0 ; // frequency in mHz

void freqTask (void *arg) {
    uint32_t interval ; // interval x 8

    // start the PIT
    setTimer(0, FASTCOUNT) ;   // initially fast
    startTimer(0) ;            // running

    while (1) {
        osStatus_t status = osMessageQueueGet(interval_q_id, &interval, NULL, osWaitForever);
        if (status == osOK) {
            
			RPM = (8*1000000*60*2) / (interval*6*30*3.05); 
			//Displaying speed bands through RGB on-board LED
			if (RPM <60) 									redLEDOnOff(LED_ON), greenLEDOnOff(LED_OFF), blueLEDOnOff(LED_OFF); 			
			else if (RPM>=60 && RPM<80) 	redLEDOnOff(LED_ON), greenLEDOnOff(LED_ON), blueLEDOnOff(LED_OFF); 
			else if (RPM>=80 && RPM<110) 	redLEDOnOff(LED_ON), greenLEDOnOff(LED_ON), blueLEDOnOff(LED_ON); 
			else if (RPM>=110 && RPM<180) redLEDOnOff(LED_OFF), greenLEDOnOff(LED_ON), blueLEDOnOff(LED_ON); 
			else if (RPM>=180) 						redLEDOnOff(LED_ON), greenLEDOnOff(LED_OFF), blueLEDOnOff(LED_ON); 
        }
    }
}

void switchDirectionTask (void *arg) {
	int motorDirection = FORWARD; 
	while(1) {
		osEventFlagsWait (evtFlags, MASK(PRESS_EVT), osFlagsWaitAny, osWaitForever);
        if      (motorDirection == FORWARD) motorDirection = REVERSE, backwardMotor();
        else if (motorDirection == REVERSE) motorDirection = FORWARD, forwardMotor(); 
	}
}


/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/

int main (void) {
    configureGPIOinput() ;       // Initialise button
    configureGPIOoutput() ;      // Initialise output

    configurePIT(0) ;

    configureTPMClock() ; 
    configureTPM0forPWM() ;
    configureTPM1forCapture() ;

    SystemCoreClockUpdate() ;
    osKernelInitialize ();

    //setPWMDuty(PWM_DUTY_MAX * 1) ; // motor should run with 12v
	redLEDOnOff(LED_ON); 
    
    // Create message queue
    interval_q_id = osMessageQueueNew(2, sizeof(uint32_t), NULL) ;

    // Create event flags
    evtFlags = osEventFlagsNew(NULL);
    
    // Create tasks
    
    t_button = osThreadNew(buttonTask, NULL, NULL) ;
		t_button2 = osThreadNew(button2Task, NULL, NULL) ;
		t_directionMotor = osThreadNew(switchDirectionTask, NULL, NULL);
		t_freq = osThreadNew(freqTask, NULL, NULL) ;
		osKernelStart ();

    // end of initialisation
    for (;;) ;
}
