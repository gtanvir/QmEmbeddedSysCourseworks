/* ------------------------------------------
   Motor Demo
     Show green     
     When button pressed, run motor slowly clockwise; show red
     Stop on button press
     Run motor quickly backwards to starting position 

   Stepping is done by counting 10ms cycles
     Fast: every cycle - 1 Rev in 480ms - 100pps, 125 RPM
     Slow: every 5 cycles - 20 pps, 25 RPM
     
     The following GPIO pins are used for the motor
        Motor Cnnctn   Port E Pin
     -----------------------------
         IN1           pin 30       (phase A+)
         IN2           pin 29       (phase A-)
         IN3           pin 23       (phase B+)
         IN4           pin 22       (phase B-)

     -------------------------------------------- */

#include <MKL25Z4.H>
#include "../include/gpio_defs.h"
#include "../include/stepperMotor.h"
#include "../include/SysTick.h"
#include "../include/pit.h"
#include <stdbool.h>

#define STATESTART (0)
#define STATERUNNING (1)
#define STATERETURN (2)
#define STATEIDLE (3)

#define BUTTONOPEN (0)
#define BUTTONCLOSED (1)
#define BUTTONBOUNCE (2)

#define POSITION_0 (0)
#define POSITION_1 (1)
#define POSITION_2 (2)
#define POSITION_3 (3)
#define POSITION_4 (4)
#define POSITION_5 (5)





/*----------------------------------------------------------------------------
  GPIO Configuration

  Configure the port B pin for the on-board red & green leds as an output
 *----------------------------------------------------------------------------*/
void configureGPIOoutput() {
        // Configuration steps
    //   1. Enable clock to GPIO ports
    //   2. Enable GPIO ports
    //   3. Set GPIO direction to output
    //   4. Ensure LEDs are off

    // Enable clock to ports B 
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK ;
    
    // Make the pin GPIO
    PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);          
    PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;          
    PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);          
    
    // Set ports to outputs
    PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;

    // Turn off the LED
    PTB->PSOR = MASK(RED_LED_POS) | MASK(GREEN_LED_POS) ;
}

/*----------------------------------------------------------------------------
  GPIO Input Configuration

  Initialse a Port D pin as an input, with no interrupt
  Bit number given by BUTTON_POS
 *----------------------------------------------------------------------------*/ 
void configureGPIOinput(void) {
    SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */

    /* Select GPIO and enable pull-up resistors and no interrupts for PTD6 and PTD7*/
    PORTD->PCR[BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    PORTD->PCR[BUTTON_POS2] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
    
    /* Set port D switch PTD6 and PTD7 bits to inputs */
    PTD->PDDR &= ~MASK(BUTTON_POS);
    PTD->PDDR &= ~MASK(BUTTON_POS2);
}

/*----------------------------------------------------------------------------
  Motor Configuration

 *----------------------------------------------------------------------------*/
motorType mcb ;   // motor control block
MotorId m1 ;      // motor id
void configureMotor() {
    m1 = & mcb ;
    m1->port = PTE ;
    m1->bitAp = MOTOR_IN1 ;
    m1->bitAm = MOTOR_IN2 ;
    m1->bitBp = MOTOR_IN3 ;
    m1->bitBm = MOTOR_IN4 ;

    // Enable clock to port E
    SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK; /* enable clock for port E */
    
    // Initialise motor data and set to state 1
    initMotor(m1) ; // motor initially stopped, with step 1 powered
}

/*----------------------------------------------------------------------------
  ledOn: Set led LED on, assumes port B

  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/    
void ledOn(int pos)
{
       // set led on without changing anything else
       // LED is actve low
         PTB->PCOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  ledOff: Set LED off, assumes port B

  Note use of the clear register (c.f. the data output)
 *----------------------------------------------------------------------------*/
void ledOff(int pos)
{
       // set led off with changing anything else
       // LED is actve low
         PTB->PSOR |= MASK(pos) ;
}

/*----------------------------------------------------------------------------
  isPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.

  mgtz: A Similar Function [isPressed2] has been created to test the switch/button 2
 *----------------------------------------------------------------------------*/
bool RunIsPressed(void) {
    if (PTD->PDIR & MASK(BUTTON_POS)) {
            return false ;
    }
    return true ;
}

bool StopIsPressed(void) {
    if (PTD->PDIR & MASK(BUTTON_POS2)) {
            return false ;
    }
    return true ;
}


/*----------------------------------------------------------------------------
  Poll the input

 Detect changes in the switch state.
    isPressed and not closed --> new press; 
     ~isPressed and closed -> not closed
*----------------------------------------------------------------------------*/
int b_state = BUTTONOPEN, b_state2 = BUTTONOPEN ;
int RunPressed = 0, StopPressed = 0 ;      //checking the position of Button 1 and Button 2
int bounceCounter = 0, bounceCounter2 = 0 ;

void task1PollInput(void)
{
    if (bounceCounter > 0) bounceCounter -- ;
    
    switch (b_state) {
        case BUTTONOPEN :
            if (RunIsPressed()) {
                RunPressed = 1 ;  // create a 'pressed' event
                b_state = BUTTONCLOSED ;
            }
            break ;
        case BUTTONCLOSED :
            if (!RunIsPressed()) {
                b_state = BUTTONBOUNCE ;
                bounceCounter = 50 ;
            }
            break ;
        case BUTTONBOUNCE :
            if (RunIsPressed()) {
                b_state = BUTTONCLOSED ;
            }
            if (bounceCounter == 0) {
                b_state = BUTTONOPEN ;
            }
            break ;
    }
}

void task12PollInput(void)
{
    if (bounceCounter2 > 0) bounceCounter2 -- ;
    
    switch (b_state2) {
        case BUTTONOPEN :
            if (StopIsPressed()) {
                StopPressed = 1 ;  // create a 'pressed' event
                b_state2 = BUTTONCLOSED ;
            }
            break ;
        case BUTTONCLOSED :
            if (!StopIsPressed()) {
                b_state2 = BUTTONBOUNCE ;
                bounceCounter2 = 50 ;
            }
            break ;
        case BUTTONBOUNCE :
            if (StopIsPressed()) {
                b_state2 = BUTTONCLOSED ;
            }
            if (bounceCounter2 == 0) {
                b_state2 = BUTTONOPEN ;
            }
            break ;
    }
}

/*-----------------------------------------------------------------
  task 2 - control the LEDs

  State      LED
  -----      ---
  start      green
  running    red
  stopped    red
  return     none
 *------------------------------------------------------------------ */
int sys_state = STATESTART ;
int sub_state = POSITION_0; 

void task2ControlLight(void)
{
    switch (sys_state) {
    case STATESTART :
          ledOn(GREEN_LED_POS) ;
          ledOff(RED_LED_POS) ;
          break ;
    case STATERUNNING:
			    ledOff(GREEN_LED_POS);
		      ledOn(RED_LED_POS);
    case STATEIDLE:
          ledOff(GREEN_LED_POS) ;
          ledOff(RED_LED_POS) ;
          break ;
    case STATERETURN :
          ledOn(RED_LED_POS) ;
          ledOn(GREEN_LED_POS) ;
          break;
    }
}


/*----------------------------------------------------------------------------
   task3 Motor Control
       initially stopped
       STATESTART && pressed --> 
         run at speed 1, clockwise; new state STATERUNNING
       STATERUNNING  && pressed -->
         stop; get steps and command steps back; new state STATERETURN
       STATERETURN   off
         when motor stopped --> next state
       STATESTOPPED  red on
*----------------------------------------------------------------------------*/
#define FASTDELAY (2) 
#define SLOWDELAY (8) 

int counter = 0;
int delay = 0;
int net_steps = 0; 
int move_state = 0;
int steps = 0;
bool motorRunning = false ;
int move_info [6][3] = { {64, true, 0},
                         {272, true, 0},
                         {736, false, 0},
                         {512, false, 0},
                         {960, false, 0},
                         {1472, true, 0},
                        };       
                         

void task3ControlMotor(void) {
     
    switch (sys_state) {
        case STATESTART :
            if (RunPressed) {
                RunPressed = false ; // acknowledge
				moveSteps(m1, 64, true);
                move_state = 1; ; 							
//                
                delay = FASTDELAY ;
                counter = delay ;
                sys_state = STATERUNNING; 
            }
            break ;
        case STATERUNNING:
            if (StopPressed) {
                StopPressed = false ; // acknowledge
                stopMotor(m1) ;
                steps = getSteps(m1) ;
				net_steps = steps; 
				//moveSteps(m1, steps, false) ; // move back same number of steps
                //delay = FASTDELAY ;
                //counter = delay ;
                sys_state = STATERETURN ;
            }
            else if (move_state == 0 && isMoving(m1) == false){     //Checking why the motor is not running, is it because the move is complete or it has returned to START position?
                     sys_state = STATESTART; 
					}
						
			else if (isMoving(m1) == false){                        
                     sys_state = STATEIDLE; 
						}
						
			break;
		case STATEIDLE:
			if (RunPressed){
				RunPressed = false;
//				switch(sub_state) {
//                    case POSITION_0:
//                                    moveSteps(m1, move_info[0][0], move_info[0][1]);
//                                    sub_state = POSITION_1; 
//                                    break;
//                    case POSITION_1:moveSteps(m1, move_info[1][0], move_info[1][1]);
//                                    sub_state = POSITION_2; 
//                                    break;
//                    case POSITION_2:moveSteps(m1, move_info[2][0], move_info[2][1]);
//                                    sub_state = POSITION_3; 
//                                    break;
//                    case POSITION_3:moveSteps(m1, move_info[3][0], move_info[3][1]);
//                                    sub_state = POSITION_4; 
//                                    break;
//                    case POSITION_4:moveSteps(m1, move_info[4][0], move_info[4][1]);
//                                    sub_state = POSITION_5; 
//                                    break;
//                    case POSITION_5:moveSteps(m1, move_info[5][0], move_info[5][1]);
//                                    sub_state = POSITION_0; 
//                                    break;  
//                }               
                
                if      (move_state == 0)   moveSteps(m1, move_info[0][0], move_info[0][1]),    move_state = 1, stopTimer(0), setTimer(0, 3276800), startTimer(0);                                        
                else if (move_state == 1)   moveSteps(m1, move_info[1][0], move_info[1][1]),    move_state = 2, stopTimer(0), setTimer(0, 771011), startTimer(0);                       
                else if (move_state == 2)   moveSteps(m1, move_info[2][0], move_info[2][1]),    move_state = 3, stopTimer(0), setTimer(0, 284939), startTimer(0);                       
                else if (move_state == 3)   moveSteps(m1, move_info[3][0], move_info[3][1]),    move_state = 4, stopTimer(0), setTimer(0, 204800), startTimer(0);                       
                else if (move_state == 4)   moveSteps(m1, move_info[4][0], move_info[4][1]),    move_state = 5, stopTimer(0), setTimer(0, 109226), startTimer(0);                       
                else if (move_state == 5)   moveSteps(m1, move_info[5][0], move_info[5][1]),    move_state = 0, stopTimer(0), setTimer(0, 71234), startTimer(0);        
								
				delay = FASTDELAY ;
                counter = delay ;
                sys_state = STATERUNNING;
				}
			else if(StopPressed) {
                StopPressed = false; 
                stopMotor(m1) ;
                returnToStart();
								move_state = 0;
//                net_steps = net_steps%48;
//                if(net_steps>24) net_steps = 48-net_steps, moveSteps (m1, net_steps, true);
//                else moveSteps(m1, net_steps, false) ; // move back same number of steps
//                delay = SLOWDELAY ;
//                counter = delay ;
                sys_state = STATESTART; 
            }
            break;
				
		case STATERETURN :
            if (StopPressed) {
								StopPressed = false; 
                stopMotor(m1);
							  returnToStart();
                move_state = 0;
				delay = SLOWDELAY ;
                counter = delay ;
				sys_state = STATERUNNING; 
			}		         
            break;
            
    }
}

/* --------------------------------------
     Update motor state - take steps - at appropriate intervals
   -------------------------------------- */
void task4UpdateMotor() {
    if (counter == 0) {
        counter = delay ;
        updateMotor(m1) ;
        motorRunning = isMoving(m1) ;
    } else {
        if (counter > 0) counter-- ;
    }
}
    

/* -------------------------------------
    Configure a timer channel with interrupts
   ------------------------------------- */
void configurePIT(int channel) {
    // enable clock to PIT
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK ;
    
    // Set MDIS = 0 module enabled
    //     FRZ = 0, timers run in debug
    PIT->MCR &= ~PIT_MCR_MDIS_MASK ;
    
    // Clear any outstanding interrupts
    // Set TIE = 1 to enable interupts
    // Do not start timer
    PIT->CHANNEL[channel].TFLG = PIT_TFLG_TIF_MASK ;
    PIT->CHANNEL[channel].TCTRL |= PIT_TCTRL_TIE_MASK ;    
    
    // Enable Interrupts 
    NVIC_SetPriority(PIT_IRQn, 128); // 0, 64, 128 or 192
    NVIC_ClearPendingIRQ(PIT_IRQn);  // clear any pending interrupts
    NVIC_EnableIRQ(PIT_IRQn);
}

/* -------------------------------------
    Start the timer on the given channel
   ------------------------------------- */
void startTimer(int channel) {
    PIT->CHANNEL[channel].TCTRL |= PIT_TCTRL_TEN_MASK ;    
}

/* -------------------------------------
    Stop the timer on the given channel
   ------------------------------------- */
void stopTimer(int channel) {
    PIT->CHANNEL[channel].TCTRL &= ~PIT_TCTRL_TEN_MASK ;    
}

/* -------------------------------------
   Set the timer value
      If the timer is running, the new value is used
      after the next timeout
   Unit are number of cycle of bus clock
   ------------------------------------- */
void setTimer(int channel, uint32_t timeout) {
    PIT->CHANNEL[channel].LDVAL = timeout ;
}

/* -------------------------------------
    Timer interrupt handler
    Check each channel to see if caused interrupt
    Write 1 to TIF to reset interrupt flag
   ------------------------------------- */
void PIT_IRQHandler(void) {
    // clear pending interrupts
    NVIC_ClearPendingIRQ(PIT_IRQn);

    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        // clear TIF
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK ;
 
        updateMotor(m1) ;
        // add code here for channel 0 interrupt
        // -- start of demo code
        // Toggle the tone pos
        //audioToggle();
        // -- end of demo code
    }
}

void returnToStart() {
								steps = getSteps(m1) ;
                net_steps = steps;
								if(net_steps>0) { 
                    net_steps = net_steps%48;
											if(net_steps>=24 && net_steps!=0) 			net_steps = 48-net_steps, moveSteps (m1, net_steps, true);
											else if (net_steps<24  && net_steps!=0)	moveSteps(m1, net_steps, false) ; // move back same number of steps
                }
                else if (net_steps<0) {
                    net_steps = net_steps * (-1);
                    net_steps = net_steps%48; 
											if(net_steps>=24 && net_steps!=0) net_steps = 48-net_steps, moveSteps (m1, net_steps, false);
											else if (net_steps<24  && net_steps!=0) moveSteps(m1, net_steps, true) ; // move back same number of steps
                
				}
			}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int main (void) {
    configureGPIOoutput() ;
    configureGPIOinput() ;
    configureMotor() ;
    configurePIT(0) ;   
    setTimer(0,3276800);
    Init_SysTick(1000) ; // SysTick every ms
    waitSysTickCounter(10) ; // initialise counter
    
    while (1) {        
        startTimer(0);
        task1PollInput() ;
        task12PollInput();
        task2ControlLight() ;
        task3ControlMotor() ;
        //task4UpdateMotor() ;
        
        
        waitSysTickCounter(10) ; // cycle every 10ms
    }
}

