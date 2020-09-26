#ifndef TPM_PWM_H
#define TPM_PWM_H
#include <MKL25Z4.H>

// TPM0 PWM
#define PWM_CHAN (1)
#define PWM_PORT (PORTA)
#define PWM_PIN (4)
#define ALT_TPM (3)

#define PWM_PRESCALE (2)  // divide by 4: 20.97 MHz --> 5.23 MHz
#define PWM_DUTY_MAX (1049) // modulo 1048 -> 5.002 KHz

// TPM1 Input Capture
#define TPM_PRESCALE (6)  // divide by 64: 20.97 MHz --> 327.7 KHz
#define TPM1_CHAN0_PORT (PORTA)
#define TPM1_CHAN0_PIN  (12)
#define TPM1_CHAN1_PIN  (13)
#define TPM1_CHAN0_ALT  (3)
#define TPM1_CHAN1_ALT  (3)
#define MESSINTERVAL (10000)

// prototypes
void configureTPMClock(void) ;
void configureTPM0forPWM(void) ;
void setPWMDuty(unsigned int duty) ;
void configureTPM1forCapture(void) ;

#endif
