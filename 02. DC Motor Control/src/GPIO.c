#include <MKL25Z4.H>
#include <stdbool.h>
#include "../include/gpio.h"

/*----------------------------------------------------------------------------
  GPIO Input Configuration

  Initialse a Port D pin as an input, with no interrupt
  Bit number given by BUTTON_POS
 *----------------------------------------------------------------------------*/
void configureGPIOinput(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;  /* enable clock for port B and D */

	/* Select GPIO and enable pull-up resistors and no interrupts */
    PORTD->PCR[BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
	PORTD->PCR[BUTTON_POS_2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
	PORTD->PCR[SENSORB] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);

    /* Set port D switch bit to inputs */
    PTD->PDDR &= ~MASK(BUTTON_POS);
	PTD->PDDR &= ~MASK(BUTTON_POS_2);
	PTD->PDDR &= ~MASK(SENSORB);
}

/* ----------------------------------------
    Configure two GPIO outputs on port A
       1. Enable clock to GPIO port
       2. Enable GPIO port
       3. Set GPIO direction to output
       4. Ensure output low
     Pin number given by PULSE_POS
 * ---------------------------------------- */
void configureGPIOoutput(void) {
    // Enable clock to port A and B
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK;

	
	// Make 2 pins GPIO
    PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
    PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);
	PORTB->PCR[DIRECTION_LED] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[DIRECTION_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);

	//Make PTB0 and PTB1 GPIO [Connected to IP1 and IP2 respectively] 
	PORTB->PCR[IP1] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[IP1] |= PORT_PCR_MUX(1);          
    PORTB->PCR[IP2] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[IP2] |= PORT_PCR_MUX(1);
	
	// Make pin GPIO
    PORTA->PCR[PULSE_POS] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[PULSE_POS] |= PORT_PCR_MUX(1);

    // Set ports to outputs
    PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS) | MASK(IP1) | MASK(IP2) | MASK(DIRECTION_LED);
	PTD->PDDR |= MASK(BLUE_LED_POS);
    
	// Set pins to output
    PTA->PDDR |= MASK(PULSE_POS) ;

    // Turn off output
    PTA->PCOR = MASK(PULSE_POS) ;
	
	// Turn off LEDs and motor control PINs
	PTB->PSOR = MASK(RED_LED_POS) | MASK(GREEN_LED_POS) | MASK(IP1) | MASK(IP2) | MASK(DIRECTION_LED);
	PTD->PSOR = MASK (BLUE_LED_POS);
} ;

/*----------------------------------------------------------------------------
  isPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isPressed(void) {
    if (PTD->PDIR & MASK(BUTTON_POS)) {
            return false ;
    }
    return true ;
}

bool isPressed2(void) {
    if (PTD->PDIR & MASK(BUTTON_POS_2)) {
            return false ;
    }
    return true ;
}

bool isPressed3(void) {
    if (PTD->PDIR & MASK(SENSORB)) {
            return false ;
    }
    return true ;
}

/*----------------------------------------------------------------------------
  Function that turns Red LED on or off
 *----------------------------------------------------------------------------*/
void redLEDOnOff (int onOff) {
    if (onOff == LED_ON) 	 	PTB->PCOR = MASK(RED_LED_POS) ;
    else if (onOff == LED_OFF)  PTB->PSOR = MASK(RED_LED_POS) ;
    
}

/*----------------------------------------------------------------------------
  Function that turns Green LED on or off
 *----------------------------------------------------------------------------*/
void greenLEDOnOff (int onOff) {
    if (onOff == LED_ON) {
        PTB->PCOR = MASK(GREEN_LED_POS) ;
    } else {
        PTB->PSOR = MASK(GREEN_LED_POS) ;
    }
}

void directionLEDOnOff (int onOff) {
    if (onOff == LED_ON) {
        PTB->PSOR = MASK(DIRECTION_LED) ;
    } else {
        PTB->PCOR = MASK(DIRECTION_LED) ;
    }
}

void blueLEDOnOff (int onOff) {
    if (onOff == LED_ON) {
        PTD->PCOR = MASK(BLUE_LED_POS) ;
    } else {
        PTD->PSOR = MASK(BLUE_LED_POS) ;
    }
}

