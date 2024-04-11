#include <p30fxxxx.h>
#include "timers.h"

void Init_T1(void) //1us
{
    TMR1 = 0;
    PR1 = TMR1_period;
    //PR1 = 0xFFFF;
    T1CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
    //T1CONbits.TCKPS0=1;
    //T1CONbits.TCKPS1=1;
    //T1CONbits.TSYNC=0;
    //IPC1bits. = 0; // T1 interrupt pririty (0-7)
    //SRbits.IPL = 3; // CPU interrupt priority is 3(11)
    IFS0bits.T1IF = 0; // clear interrupt flag
    IEC0bits.T1IE = 1; // enable interrupt

    T1CONbits.TON = 0; // T1 on 
}

void Init_T4(void) {
    TMR4 = 0;
    PR4 = TMR4_period;
    T4CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
    IFS1bits.T4IF = 0; // clear timer2 interrupt flag
    IEC1bits.T4IE = 1; // enable timer2 interrupt
    T4CONbits.TON = 0; // Timer2 off
}

void Init_T5(void) {
    TMR5 = 0;
    PR5 = TMR5_period;
    T5CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
    IFS1bits.T5IF = 0; // clear timer2 interrupt flag
    IEC1bits.T5IE = 1; // enable timer2 interrupt
    T5CONbits.TON = 0; // Timer2 off
}