#include <stdbool.h>

// Definitions for GPIO
#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H


#define MASK(x) (1UL << (x))

// Buttons are on port D, pin 6 and 7
#define BUTTON_POS (6)
#define BUTTON_POS_2 (7)
#define SENSORB (5)

#define RED_LED_POS (18)
#define GREEN_LED_POS (19)
#define BLUE_LED_POS (1)
#define DIRECTION_LED (2)

#define IP1 (0)
#define IP2 (1)

// LED states
#define LED_ON (1)
#define LED_OFF (0)

// GPIO output used for the pulses, port A pin 2
#define PULSE_POS (2)

//Signal Number
#define PRESS_EVT (0) 

//Motor Direction
#define FORWARD (0)
#define REVERSE (1)



// Function prototypes
void configureGPIOinput(void) ;       // Initialise button
void configureGPIOoutput(void) ;      // Initialise output
bool isPressed(void) ;
bool isPressed2(void);
bool isPressed3(void);
void greenLEDOnOff(int) ;
void redLEDOnOff(int) ;
void blueLEDOnOff(int);
void directionLEDOnOff(int); 

#endif
