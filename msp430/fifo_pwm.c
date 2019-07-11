//*******************************************************************************
//  OLD CODE. Using main.c, this file has been kept for posterity
//  Modified PWM code, based on MSP430F55xx_ta0_16.c
//
//  Description: This program generates three PWM outputs on P1.2,P1.3,P1.4 using
//  Timer1_A configured for up mode. The value in CCR0, 21100, defines the PWM
//  period at 20ms (measured on oscilloscope) which is in the allowable servo period range
//  for the Jaguar motor controller (p5 of datasheet).
//
//  The values in CCR1 and CCR2 are the PWM duty cycles. Using ~1.045MHz
//  SMCLK as TACLK, the timer period is ~ (21100/1045k) ~ 20ms.
//  The Jaguar expects 0.67 and 2.33ms as the min/max PWM signals with 1.5ms as neutral
//  ACLK = n/a, SMCLK = MCLK = TACLK = default DCO ~1.045MHz.
//
// 	The input code for the Jaguar Steering Motor is #
// 	The input code for the Jaguar Brake Actuator is $
// 	The input code for the Turnigy Shimano Motor is %
//
//                MSP430F552x
//            -------------------
//        /|\|                   |
//         | |                   |
//         --|RST                |
//           |                   |
//           |         P1.2/TA0.1|--> CCR1 - PWM (Jaguar Steering Motor)
//     RX<-- |P3.4     P1.3/TA0.2|--> CCR2 - PWM (Jaguar Brake Actuator)
//     TX<-- |P3.3     P1.4/TA0.3|--> CCR3 - PWM (Turnigy Shimano Drive Motor)
//
//  TA0CCR1 = 710; // corresponds to 0.67ms (Jaguar full reverse)
//  TA0CCR1 = 1580; // corresponds to 1.5ms (Jaguar neutral)
//  TA0CCR1 = 2450; // corresponds to 2.33ms (Jaguar full forward)
//  TA0CCR3 = 1050; // corresponds to 1.0ms (turnigy neutral)
//  TA0CCR3 = 2100; // corresponds to 2.0ms (turnigy neutral)
//******************************************************************************

#include <msp430.h>
#include <stdlib.h>

#define PWM_NEUTRAL_JAGUAR (1580)		// 1.5 ms
#define PWM_NEUTRAL_SHIMANO (1050)		// 1.0 ms

int main(void)
{

// PWM Setup
    WDTCTL = WDTPW + WDTHOLD;			// Stop WDT

    P1DIR |= BIT0;                      // LED1 P1.0 as output
    P1OUT = 0;                          // Turn LED OFF
	
	P1DIR |= BIT2+BIT3+BIT4;			// P1.2, P1.3 and P1.4 output
    P1SEL |= BIT2+BIT3+BIT4;			// P1.2, P1.3 and P1.4 options select
    TA0CCR0 = 21100-1;					// PWM Period
    TA0CCTL1 = OUTMOD_7;				// CCR1 reset/set
    TA0CCTL2 = OUTMOD_7;				// CCR2 reset/set
	TA0CCTL3 = OUTMOD_7;				// CCR3 reset/set
    TA0CTL = TASSEL_2 + MC_1 + TACLR;	// SMCLK, up mode, clear TAR

// set default output to neutral PWM
    TA0CCR1 = PWM_NEUTRAL_JAGUAR;
    TA0CCR2 = PWM_NEUTRAL_JAGUAR;
    TA0CCR3 = PWM_NEUTRAL_SHIMANO;

// UART Setup
    P3SEL |= BIT3+BIT4;					// P3.3,4 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSWRST;				// **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;				// SMCLK
    UCA0BR0 = 9;						// 1MHz 115200 (see User's Guide)
    UCA0BR1 = 0;						// 1MHz 115200
    UCA0MCTL |= UCBRS_1 + UCBRF_0;		// Modulation UCBRSx=1, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;				// **Initialize USCI state machine**
    UCA0IE |= UCRXIE;					// Enable USCI_A0 RX interrupt

    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0
    __no_operation();                         // For debugger


	
    WDTCTL = WDTPW | WDT_ARST_250;			// Activate watchdog timer, code will restart if timer is not cleared within 1 second.

    for(;;){
        P1OUT = 0;
    };  // Infinite loop
}

// Assume UART input is formatted as "XXXXY", where X is a zero-padded ASCII-coded base-16 number and Y is one of {#,$,%} to select the right output
char uart_buffer[5];

/** @brief Macro for returning the length of an array */
#define NUMEL(x) (sizeof(x)/sizeof(x[0]))

void process_char(char input){
    if(input >= '0' && input <= '9'){
        // Add new input to buffer, shift old contents one element to the right
        int i;
        for(i = 0; i < NUMEL(uart_buffer)-2; i++){
            uart_buffer[i] = uart_buffer[i+1];
        }
        // Place new input in second last position
        uart_buffer[NUMEL(uart_buffer) - 2] = input;
    }
	// Steering Motor
    else if(input == '#'){
        // Ensure uart_buffer is null-terminated
        uart_buffer[NUMEL(uart_buffer) -1 ] = '\0';

        //Check to ensure a non-empty string has been detected
        long int duty = PWM_NEUTRAL_JAGUAR;
        if(uart_buffer[0] != '\0'){
            // Got end-of-input symbol, convert buffer content to int
            char * endptr = &uart_buffer[0];
            long int duty = strtol(uart_buffer, &endptr, 10);

            if(endptr != &uart_buffer[4]){
                //Something bad happened, set duty to safe default
                duty = PWM_NEUTRAL_JAGUAR;
            }

            if(duty > 2450){
                duty = 2330;
            }
            else if(duty < 710){
                duty = 710;
            }

            // Output the duty cycle on P1.2
            TA0CCR1 = duty;
        }
        // Completely wipe buffer
        int i;
        for(i = 0; i < NUMEL(uart_buffer); i++){
            uart_buffer[i] = '\0';
        }
    }
	// Brake Actuator
    else if(input == '$'){
        // Ensure uart_buffer is null-terminated
        uart_buffer[NUMEL(uart_buffer) -1 ] = '\0';

        //Check to ensure a non-empty string has been detected
        long int duty = PWM_NEUTRAL_JAGUAR;
        if(uart_buffer[0] != '\0'){
            // Got end-of-input symbol, convert buffer content to int
            char * endptr = &uart_buffer[0];
            long int duty = strtol(uart_buffer, &endptr, 10);

            if(endptr != &uart_buffer[4]){
                //Something bad happened, set duty to safe default
                duty = PWM_NEUTRAL_JAGUAR;
            }

            if(duty > 2450){
                duty = 2330;
            }
            else if(duty < 710){
                duty = 710;
            }

            // Output the duty cycle on P1.3
			TA0CCR2 = duty;
        }
        // Completely wipe buffer
        int i;
        for(i = 0; i < NUMEL(uart_buffer); i++){
            uart_buffer[i] = '\0';
        }
    }
	// Drive Motor
    else if(input == '%'){
        // Ensure uart_buffer is null-terminated
        uart_buffer[NUMEL(uart_buffer) -1 ] = '\0';

        //Check to ensure a non-empty string has been detected
        long int duty = PWM_NEUTRAL_SHIMANO;
        if(uart_buffer[0] != '\0'){
            // Got end-of-input symbol, convert buffer content to int
            char * endptr = &uart_buffer[0];
            long int duty = strtol(uart_buffer, &endptr, 10);

            if(endptr != &uart_buffer[4]){
                //Something bad happened, set duty to safe default
                duty = PWM_NEUTRAL_SHIMANO;
            }

            if(duty > 2100){
                duty = 2100;
            }
            else if(duty < 1050){
                duty = 1050;
            }

            // Output the duty cycle on P1.4
			TA0CCR3 = duty;
        }
        // Completely wipe buffer
        int i;
        for(i = 0; i < NUMEL(uart_buffer); i++){
            uart_buffer[i] = '\0';
        }
    }
    // Emergency Stop
    else if(input == '@'){
        TA0CCR1 = PWM_NEUTRAL_JAGUAR;
        TA0CCR2 = PWM_NEUTRAL_JAGUAR;
        TA0CCR3 = PWM_NEUTRAL_SHIMANO;
    }
}


// Set PWM value to match RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{

    switch(__even_in_range(UCA0IV,4))
    {
    case 0:break;                             // Vector 0 - no interrupt
    
    case 2:                                   // Vector 2 - RXIFG
        WDTCTL = WDTPW | WDT_ARST_250; // Reset watchdog timer
        P1OUT = 1;              // Turn LED ON
        while (!(UCA0IFG & UCTXIFG));             // USCI_A0 TX buffer ready?
        // Set TA0CCR1 to match received data using letters to represent states
        // UCA0RXBUF = UCA0TXBUF; //echo received char
            process_char((char) UCA0RXBUF);

        break;
        
    case 4:break;                             // Vector 4 - TXIFG

    default: break;
    }
}
