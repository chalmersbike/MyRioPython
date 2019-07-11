//*******************************************************************************
//
//  Modified PWM code, based on MSP430F55xx_ta0_16.c
//
//  Description: This program generates two PWM outputs on P1.2,P1.3 using
//  Timer1_A configured for up mode. The value in CCR0, 21100, defines the PWM
//  period at 20ms (measured on oscilloscope) which is in the allowable servo period range
//  for the Jaguar motor controller (p5 of datasheet).
//
//  The values in CCR1 and CCR2 are the PWM duty cycles. Using ~1.045MHz
//  SMCLK as TACLK, the timer period is ~ (21100/1045k) ~ 20ms.
//  The Jaguar expects 0.67 and 2.33ms as the min/max PWM signals with 1.5ms as neutral
//  ACLK = n/a, SMCLK = MCLK = TACLK = default DCO ~1.045MHz.
//
//                MSP430F552x
//            -------------------
//        /|\|                   |
//         | |                   |
//         --|RST                |
//           |                   |
//     RX<-- |P3.4     P1.2/TA0.1|--> CCR1 - PWM
//     TX<-- |P3.3     P1.3/TA0.2|--> CCR2 - PWM
//
//  TA0CCR1 = 710; // corresponds to 0.67ms (full reverse)
//  TA0CCR1 = 1580; // corresponds to 1.5ms (neutral)
//  TA0CCR1 = 2450; // corresponds to 2.33ms (full forward)
//******************************************************************************

#include <msp430.h>

int main(void)
{
// PWM Setup
    WDTCTL = WDTPW + WDTHOLD;             // Stop WDT
    P1DIR |= BIT2+BIT3;                   // P1.2 and P1.3 output
    P1SEL |= BIT2+BIT3;                   // P1.2 and P1.3 options select
    TA0CCR0 = 21100-1;                    // PWM Period
    TA0CCTL1 = OUTMOD_7;                  // CCR1 reset/set
    TA0CCR1 = 1580;                       // CCR1 initial PWM (neutral)
    TA0CCTL2 = OUTMOD_7;                  // CCR2 reset/set
    TA0CCR2 = 1580;                        // CCR2 initial PWM (neutral)
    TA0CTL = TASSEL_2 + MC_1 + TACLR;     // SMCLK, up mode, clear TAR

// UART Setup
    P3SEL |= BIT3+BIT4;                       // P3.3,4 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 9;                              // 1MHz 115200 (see User's Guide)
    UCA0BR1 = 0;                              // 1MHz 115200
    UCA0MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0
    __no_operation();                         // For debugger
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
        while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
        // Set TA0CCR1 to match received data using letters to represent states
        UCA0RXBUF = UCA0TXBUF; //echo received char

        if(UCA0RXBUF == 'a'){
        TA0CCR1 = 710;
        TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'b'){
            TA0CCR1 = 738;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'c'){
            TA0CCR1 = 766;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'd'){
            TA0CCR1 = 794;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'e'){
            TA0CCR1 = 822;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'f'){
            TA0CCR1 = 850;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'g'){
            TA0CCR1 = 878;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'h'){
            TA0CCR1 = 906;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'i'){
            TA0CCR1 = 934;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'j'){
            TA0CCR1 = 962;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'k'){
            TA0CCR1 = 990;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'l'){
            TA0CCR1 = 1018;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'm'){
            TA0CCR1 = 1046;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'n'){
            TA0CCR1 = 1074;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'o'){
            TA0CCR1 = 1102;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'p'){
            TA0CCR1 = 1130;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'q'){
            TA0CCR1 = 1159;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'r'){
            TA0CCR1 = 1187;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 's'){
            TA0CCR1 = 1215;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 't'){
            TA0CCR1 = 1243;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'u'){
            TA0CCR1 = 1271;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'v'){
            TA0CCR1 = 1299;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'w'){
            TA0CCR1 = 1327;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'x'){
            TA0CCR1 = 1355;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'y'){
            TA0CCR1 = 1383;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'z'){
            TA0CCR1 = 1411;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'A'){
            TA0CCR1 = 1439;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'B'){
            TA0CCR1 = 1467;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'C'){
            TA0CCR1 = 1495;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'D'){
            TA0CCR1 = 1523;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'E'){
            TA0CCR1 = 1551;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'F'){
            TA0CCR1 = 1580;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'G'){
            TA0CCR1 = 1608;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'H'){
            TA0CCR1 = 1636;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'I'){
            TA0CCR1 = 1664;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'J'){
            TA0CCR1 = 1692;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'K'){
            TA0CCR1 = 1720;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'L'){
            TA0CCR1 = 1748;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'M'){
            TA0CCR1 = 1776;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'N'){
            TA0CCR1 = 1804;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'O'){
            TA0CCR1 = 1832;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'P'){
            TA0CCR1 = 1860;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'Q'){
            TA0CCR1 = 1888;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'R'){
            TA0CCR1 = 1916;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'S'){
            TA0CCR1 = 1944;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'T'){
            TA0CCR1 = 1972;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'U'){
            TA0CCR1 = 2000;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'V'){
            TA0CCR1 = 2029;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'W'){
            TA0CCR1 = 2057;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'X'){
            TA0CCR1 = 2085;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'Y'){
            TA0CCR1 = 2113;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == 'Z'){
            TA0CCR1 = 2141;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == ' '){
            TA0CCR1 = 2169;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '0'){
            TA0CCR1 = 2197;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '1'){
            TA0CCR1 = 2225;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '2'){
            TA0CCR1 = 2253;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '3'){
            TA0CCR1 = 2281;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '4'){
            TA0CCR1 = 2309;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '5'){
            TA0CCR1 = 2337;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '6'){
            TA0CCR1 = 2365;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '7'){
            TA0CCR1 = 2393;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '8'){
            TA0CCR1 = 2421;
            TA0CCR2 = TA0CCR1;
        }
        else if(UCA0RXBUF == '9'){
            TA0CCR1 = 2450;
            TA0CCR2 = TA0CCR1;
        }
        else {
        // NEUTRAL
        TA0CCR1 = 1580; // corresponds to 1.51ms (stop)
        TA0CCR2 = TA0CCR1;
        }

    break;

    case 4:break;                             // Vector 4 - TXIFG

    default: break;
    }
}
