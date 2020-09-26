#include <MKL25Z4.H>
#include "../include/tpmPwm.h"

/* ---------------------------------------------------------------------------
   Configure TPM Clock

     The KL25Z has 3 TPM modules but they share the clock source
     This configures the system clock as the source (20.97 MHz default)
   --------------------------------------------------------------------------*/
void configureTPMClock() {
    // Choose TPM clock - system clock (section 12.2.3, p195)
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1) ;
}

/*----------------------------------------------------------------------------
  Configure TPM0 for PWM

     Controlled by the following macro definitions:
         PWM_CHAN the channel on TPM0; note this determines the pin
         PWM_DUTY_MAX  the maximum duty = modulo + 1
         PWM_PRESCALE the pre-scale for the clock (3 bit value):
            000 Divide by 1        001 Divide by 2
            010 Divide by 4        011 Divide by 8
            100 Divide by 16       101 Divide by 32
            110 Divide by 64       111 Divide by 128
         PWM_PORT the port, such as PORTA, PORTB etc
         PWM_PIN  the pin, within the port
         ALT_TPM  the alternative number for muxiplexing the pin
      These values must be consistent with the KL25Z pinout

      Note that this this could be split to separate module and channel
      configurations
 *----------------------------------------------------------------------------*/
void configureTPM0forPWM() {
    // Enable clock to port A
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // Enable clock to TPM0
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

    // Set the Pin Control Register
    // Set Pin on PWM_PORT as TPM o/p
    PWM_PORT->PCR[PWM_PIN] &= ~PORT_PCR_MUX_MASK;
    PWM_PORT->PCR[PWM_PIN] |= PORT_PCR_MUX(ALT_TPM);

    // Count modulo PWM_DUTY_MAX - 1 = 127.
    // Note: PWM_DUTY_MAX is used for 100% duty cycle
    TPM0->MOD = TPM_MOD_MOD(PWM_DUTY_MAX-1) ;

    // Disable the TPM0
    TPM0->SC = TPM_SC_CMOD(0) ;
    while ((TPM0->SC & TPM_SC_CMOD(3))) ;

    // Prescale (e.g. by 8 with 0x3)
    TPM0->SC |= TPM_SC_PS(PWM_PRESCALE) ;

    // Configure channel TPM_CHAN
    //   Edge pulse: MSB:A = 1:0
    //   True pulse high: ELSB:A = 1:0
    TPM0->CONTROLS[PWM_CHAN].CnSC = 0x28 ;

    // Set the channel value - off
    TPM0->CONTROLS[PWM_CHAN].CnV = TPM_CnV_VAL(0) ;

    // Set countr to continue when debugging
    TPM0->CONF |= TPM_CONF_DBGMODE(0x3) ;

    // Enable using internal clock
    TPM0->SC |= TPM_SC_CMOD(0x01) ;
    while (!(TPM0->SC & TPM_SC_CMOD(3))) ;
}

/*----------------------------------------------------------------------------
  Set PWM duty cycle

    Set the duty cycle.
    duty value       duty %
       0               0%
     PWM_DUTY_MAX    100%
 *----------------------------------------------------------------------------*/
void setPWMDuty(unsigned int duty) {

    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX ;

    // Set the channel variable
    TPM0->CONTROLS[PWM_CHAN].CnV = TPM_CnV_VAL(duty) ;
}

/*----------------------------------------------------------------------------
  Configure TPM1 for Capture

    This module only has two channels: channel 0 is configured

    Controlled by the following macro definitions:
        TPM_PRESCALE the pre-scale for the clock (3 bit value):
            000 Divide by 1        001 Divide by 2
            010 Divide by 4        011 Divide by 8
            100 Divide by 16       101 Divide by 32
            110 Divide by 64       111 Divide by 128
         TPM1_CHAN0_PORT the port, such as PORTA, PORTB etc
         TPM1_CHAN0_PIN  the pin, within the port
         TPM1_CHAN0_ALT  the alternative number for muxiplexing the pin
    These values must be consistent with the KL25Z pinout

    The capture is on the rising edge. Interrupts are generated.

    Note that this this could be split to separate module and channel
    configurations
 *----------------------------------------------------------------------------*/

 void configureTPM1forCapture() {
    
     // Enable clock to port A
     SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

     // Enable clock to TPM1
     SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

     // Set the Pin Control Register
     // Set Pin ..... ASSUMES CLOCK TO PORT
     TPM1_CHAN0_PORT->PCR[TPM1_CHAN0_PIN] &= ~PORT_PCR_MUX_MASK;
     TPM1_CHAN0_PORT->PCR[TPM1_CHAN0_PIN] |= PORT_PCR_MUX(TPM1_CHAN0_ALT);
     
     // Set the Pin Control Register
     // Set Pin ..... ASSUMES CLOCK TO PORT
     TPM1_CHAN0_PORT->PCR[TPM1_CHAN1_PIN] &= ~PORT_PCR_MUX_MASK;
     TPM1_CHAN0_PORT->PCR[TPM1_CHAN1_PIN] |= PORT_PCR_MUX(TPM1_CHAN1_ALT);

     // Count modulo max value 0xffff
     TPM1->MOD = TPM_MOD_MOD(0xFFFF) ;

     // Set countr to continue when debugging
     TPM1->CONF |= TPM_CONF_DBGMODE(0x3) ;

     // Disable the TPM0
     TPM1->SC = TPM_SC_CMOD(0) ;
     while ((TPM1->SC & TPM_SC_CMOD(3))) ;

     // Prescale (e.g. by 8 with 0x3)
     TPM1->SC |= TPM_SC_PS(TPM_PRESCALE) ;

     // Enable the counter overflow interrupt
     TPM1->SC |= TPM_SC_TOIE(1) ;

     // Configure the channel
     //   Interrupt           =   1
     //   Capture: MSB:A      = 0:0
     //   Rising edge: ELSB:A = 0:1
     //                           0
     //   DMA                     0
     TPM_CnSC_REG(TPM1,0) = 0x44 ; // 100 0100
     TPM_CnSC_REG(TPM1,1) = 0x44 ; // 100 0100

     /* Enable Interrupts */
     NVIC_SetPriority(TPM1_IRQn, 128); // 0, 64, 128 or 192
     NVIC_ClearPendingIRQ(TPM1_IRQn);  // clear any pending interrupts
     NVIC_EnableIRQ(TPM1_IRQn);

     // Enable using internal clock
     TPM1->SC |= TPM_SC_CMOD(0x01) ;
     while (!(TPM1->SC & TPM_SC_CMOD(3))) ;
 }
