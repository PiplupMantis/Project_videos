/**
 * Lab 6
 */

#include <msp430.h>
#include <stdint.h>
#include "lcd.h"
#include <stdio.h>


unsigned int    distance = 0;

unsigned int    mode = 0;
unsigned int    dbounce=61;

void msp_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //LEDS pins with PWM
    P2DIR   |=   BIT6 | BIT7;      // P2.6 and P2.7 set as output
    P2OUT   &= ~(BIT6 | BIT7);
    P3DIR   |=   BIT6 | BIT7;      // P3.6 and P3.7 set as output
    P3OUT   &= ~(BIT6 | BIT7);

    //Button S1
    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;
    P1IES |=  BIT1;
    P1IFG &= ~BIT1;
    P1IE  |=  BIT1;

    // Button S2
    P1DIR &= ~BIT2;


    //Trigger Pin
    P1DIR |= BIT3;
    P1SEL0 |= BIT3;
    P1SEL1 &= ~BIT3;

    //Echo pin
    P3DIR &= ~BIT3;
    P3SEL0 &= ~BIT3;
    P3SEL1 |= BIT3;
    //P3OUT &= ~BIT3;

    //Buzzer Pin
    P1DIR  |= BIT5;
    P1SEL0 |= BIT5;
    P1SEL1 |= BIT5;

    PM5CTL0 &= ~LOCKLPM5;

    // TIMER BUZZER
    TA0CTL = MC_0 | ID_0 | TASSEL_2 | TACLR;
    TA0CCTL0 = OUTMOD_4;
    TA0CCR0 = 851;


    //TimerA1 for Echo Pin
    TA1CTL = TASSEL__SMCLK | MC__UP | ID__1 | TACLR;

    //TA1CCR0 = 0x0032; // PERIOD = 50us
    TA1CCR0 = 0xFFFF; // 65ms
    TA1CCR2 = 0x000A;  // 10us      // for trigger pin
    TA1CCTL2 = OUTMOD_7;

    TA1CCTL1 = CM_3 | SCS | CCIS_0 | CAP | CCIE;

    __enable_interrupt();
}


uint16_t last_cap = 0;       // Time when the last capture happened

#pragma vector = TIMER1_A1_VECTOR   //Timer vector capture Echo pin
__interrupt void button_timer(void) {
    TA1IV=0;                        // must read or write TA1IV to reset interrupt flags
                                    // Normally this is done with switch((TA0IV)
    uint16_t new_cap = TA1CCR1;     // Get the captured time
    uint16_t cap_diff = new_cap - last_cap; // Calculate the time pressed
    last_cap = new_cap;             // Store the captured time for next press

    if (!(TA1CCTL1 & CCI)) {         // capture when falling edge of the echo pin
       distance = cap_diff;   // 75% duty cycle if pressed > than 1 sec
       __low_power_mode_off_on_exit();
   }
}

#pragma vector = PORT1_VECTOR
__interrupt void p1_ISR(){
    if(P1IFG & BIT1){           // pressing button P1.1 to switch between displaying in and cm
        if (mode == 0)
            mode = 1;
        else
            mode = 0;
    }
    P1IFG &= ~BIT1;
}

void turnOn_LEDs(int num){
    switch(num){
    case 1:
        P2OUT &= ~BIT6;
        P2OUT &= ~BIT7;
        P3OUT &= ~BIT6;
        P3OUT |= BIT7;
        break;
    case 2:
        P2OUT &= ~BIT6;
        P2OUT &= ~BIT7;
        P3OUT |= BIT6;
        P3OUT |= BIT7;
        break;
    case 3:
        P2OUT &= ~BIT6;
        P2OUT |= BIT7;
        P3OUT |= BIT6;
        P3OUT |= BIT7;
        break;
    case 4:
        P2OUT |= BIT6;
        P2OUT |= BIT7;
        P3OUT |= BIT6;
        P3OUT |= BIT7;
        break;
    default: break;
    }
}

void turnOff()
{
    P2OUT &= ~BIT6;
    P2OUT &= ~BIT7;
    P3OUT &= ~BIT6;
    P3OUT &= ~BIT7;
    P1OUT &= ~BIT5;

}

void delay(int ticks){

    TB0CTL = MC_1 | ID_3| TASSEL_2 | TAIE | TACLR;

    TB0CCR0 = ticks;

    __low_power_mode_2();

    TB0CTL &= ~(BIT4|BIT5);

}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMERB0_ISR(void)
{
  switch(__even_in_range(TB0IV,14))
  {
    case  0: break;                          // No interrupt
    case  2: break;                          // CCR1 not used
    case  4: break;                          // CCR2
    case  6: break;
    case  8: break;
    case 10: break;                          // set TACCR5 to end of next period
    case 12: break;                          //
    case 14:
        __low_power_mode_off_on_exit();
             break;
    default: break;
  }
}

void main(void){
    msp_init();
    lcd_init();
    for(;;){
        __low_power_mode_3();
        lcd_clear();
        int digits = 0;
        if (mode == 0){     // display distance in cm
            distance = distance/58; // calculate distance in cm
            LCDM15 = 0x9C;
            LCDM8  = 0xEC;
            LCDM9  = 0x50;
            if (distance < 11)      // closest, turn on 1 LED only
                turnOn_LEDs(1);
            else if (distance >= 11 && distance <20)
                turnOn_LEDs(2);
            else if (distance >= 20 && distance <30)
                turnOn_LEDs(3);
            else                    // furthest, turn on all LEDs
                turnOn_LEDs(4);
        }
        else{           // mode == 1, display distance in cm
            distance = distance/148; // calculate distance in inches
            LCDM15 = 0x60;
            LCDM8  = 0xEC;
            if (distance < 4)
                turnOn_LEDs(1);
            else if (distance >= 4 && distance <8)
                turnOn_LEDs(2);
            else if (distance >= 8 && distance <12)
                turnOn_LEDs(3);
            else
                turnOn_LEDs(4);
        }
        int temp_distance = distance;
        while (temp_distance != 0){
            temp_distance /= 10;
            digits++;
        }
        displayNum(distance,digits);    // display distance on LCD
        TA0CTL |= MC_1;
        delay(distance*320);
        TA0CTL &= ~MC_1;
        delay(distance*320);

        if (P1IN & BIT2)
        {
            turnOff();

        }



    }
}

