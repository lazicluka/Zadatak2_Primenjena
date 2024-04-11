#include <stdio.h>
#include <stdlib.h>
#include <p30fxxxx.h>
#include <outcompare.h>
#include <string.h>
#include "adc.h"
#include "timers.h"
#include "timer_pmw.h"
#include "UART.h"
#include "fotootpornik.h"
#include <stdint.h>
#define BUFFER_SIZE 6

#define TRIG_FORWARD LATDbits.LATD8
#define ECHO_FORWARD PORTFbits.RF6

#define TRIG_BACK LATBbits.LATB9
#define ECHO_BACK PORTBbits.RB2

#define SPEED_OF_SOUND (0.0343)        // centimeters per microsecond
#define INSTRUCTION_CLOCK_PERIOD (0.1) // microseconds

_FOSC(CSW_FSCM_OFF &XT_PLL4); // instruction takt je isti kao i kristal 10MHz
_FWDT(WDT_OFF);

unsigned char tempRX2, tempRX1_bluetooth;
unsigned int us_counter;
unsigned int sirovi0, sirovi1, sirovi2, sirovi3;
unsigned int FR_front, FR_back, FR_left, FR_right;
int startFlag = 0;
int stopFlag = 0;

char buffer[BUFFER_SIZE];
unsigned int bufferIndex = 0;

static unsigned char time_overflow_back = 0;
static unsigned char time_overflow_forward = 0;
static float measured_distance_back = 0;
static float measured_distance_forward = 0;

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) // svakih 1us
{
    TMR1 = 0;

    us_counter++; // brojac za funkciju delay_1us()

    IFS0bits.T1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) // za PMW
{
    TMR2 = 0;
    IFS0bits.T2IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void) // za dig senzor nazad
{
    TMR4 = 0;
    ECHO_BACK = 0;
    time_overflow_back = 1;
    IFS1bits.T4IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void) // za dig senzor napred
{
    TMR5 = 0;
    ECHO_FORWARD = 0;
    time_overflow_forward = 1;
    IFS1bits.T5IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;

    tempRX1_bluetooth = U1RXREG;

    // Check if the received character is a newline or carriage return because app does it by default sends one of two after word send(ends it with)
    if (tempRX1_bluetooth == '\n' || tempRX1_bluetooth == '\r') {
        // Null-terminate it
        buffer[bufferIndex] = '\0';

        // Check if received data is "START"
        if (strcmp(buffer, "START") == 0) {
            // Set startFlag to 1
            startFlag = 1;

        }// Check if received data is "STOP"
        else if (strcmp(buffer, "STOP") == 0) {
            // Set stopFlag to 1
            stopFlag = 1;
        }
        // Reset buffer index for the next reception
        bufferIndex = 0;
    }// Check if the buffer is not full
    else if (bufferIndex < BUFFER_SIZE - 1) {
        // Add the received character to the buffer
        buffer[bufferIndex] = tempRX1_bluetooth;
        bufferIndex++;
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    IFS1bits.U2RXIF = 0;
    tempRX2 = U2RXREG;
}

void __attribute__((__interrupt__, no_auto_psv)) _ADCInterrupt(void) {

    sirovi0 = ADCBUF0;
    sirovi1 = ADCBUF1;
    sirovi2 = ADCBUF2;
    sirovi3 = ADCBUF3;

    IFS0bits.ADIF = 0;
}

void initPins() {

    // FOTOOTPORNICI analogni
    ADPCFGbits.PCFG0 = 0; // back-fotoR na RB0
    ADPCFGbits.PCFG1 = 0; // right-fotoR na RB1
    ADPCFGbits.PCFG8 = 0; // front-fotoR na RB8
    ADPCFGbits.PCFG10 = 0; // left-fotoR na RB10

    TRISBbits.TRISB0 = 1; // ulazni
    TRISBbits.TRISB1 = 1; // ulazni
    TRISBbits.TRISB8 = 1; // ulazni
    TRISBbits.TRISB10 = 1; // ulazni

    // ULTRAZVUCNI NAPRED
    TRISDbits.TRISD8 = 0; // trigger
    TRISFbits.TRISF6 = 1; // echo

    // ULTRAZVUCNI NAZAD
    ADPCFGbits.PCFG9 = 1; // digitalni pin
    TRISBbits.TRISB9 = 0; // trigger

    ADPCFGbits.PCFG2 = 1; // digitalni pin
    TRISBbits.TRISB2 = 1; // echo

    // pinovi za kretanje
    TRISFbits.TRISF0 = 0; // in1
    TRISFbits.TRISF1 = 0; // in2
    TRISFbits.TRISF2 = 0; // in4
    TRISFbits.TRISF3 = 0; // in3

    // pwm
    TRISDbits.TRISD1 = 0; // enB
    TRISDbits.TRISD0 = 0; // enA
}

void delay_1us(int vreme) {
    us_counter = 0;
    T1CONbits.TON = 1;
    while (us_counter < vreme)
        ;
    T1CONbits.TON = 0;
}

void delay_for(uint32_t num) // unsigned int ide do 65535
{
    uint32_t broj;
    for (broj = 0; broj < num; broj++)
        ;
}

static void MeasureBackDistance() {
    // logical one lasts for 10us
    TRIG_BACK = 1;
    delay_1us(3); //  3 instead of 10 to make logical one lasts for 10us
    TRIG_BACK = 0;
    delay_1us(3);
    while (!ECHO_BACK)
        ; //  the value of the echo pin becomes 1 (the rising edge detected)
    TMR4 = 0; // reset T4
    IFS1bits.T4IF = 0;
    T4CONbits.TON = 1; // turn on T4, time measurement begins
    while (ECHO_BACK)
        ; // the value of the echo pin becomes 0 (the falling edge detected)
    T4CONbits.TON = 0; // turn off T4, time measurement stops
    unsigned int measured_time_back;
    if (time_overflow_back == 1) // time overflow happens
    {
        measured_time_back = TMR4_period;
        time_overflow_back = 0;
    } else // the signal sent has returned
    {
        measured_time_back = TMR4;
    }
    TMR4 = 0;
    // operation /2 is used because the ultrasonic pulse travels to the obstacle and back
    // operation *INSTRUCTION_CLOCK_PERIOD is used to get the time in microseconds
    measured_distance_back = (measured_time_back * INSTRUCTION_CLOCK_PERIOD) / 2 * SPEED_OF_SOUND;
}

static void MeasureForwardDistance() {
    // logical one lasts for 10us
    TRIG_FORWARD = 1;
    delay_1us(3); //  3 instead of 10 to make logical one lasts for 10us
    TRIG_FORWARD = 0;
    delay_1us(3);
    while (!ECHO_FORWARD)
        ; //  the value of the echo pin becomes 1 (the rising edge detected)
    TMR5 = 0; // reset T4
    IFS1bits.T5IF = 0;
    T5CONbits.TON = 1; // turn on T4, time measurement begins
    while (ECHO_FORWARD)
        ; // the value of the echo pin becomes 0 (the falling edge detected)
    T5CONbits.TON = 0; // turn off T4, time measurement stops
    unsigned int measured_time_forward;
    if (time_overflow_forward == 1) // time overflow happens
    {
        measured_time_forward = TMR5_period;
        time_overflow_forward = 0;
    } else // the signal sent has returned
    {
        measured_time_forward = TMR5;
    }
    TMR5 = 0;
    // operation /2 is used because the ultrasonic pulse travels to the obstacle and back
    // operation *INSTRUCTION_CLOCK_PERIOD is used to get the time in microseconds
    measured_distance_forward = (measured_time_forward * INSTRUCTION_CLOCK_PERIOD) / 2 * SPEED_OF_SOUND;
}

void idiNapred() {

    //   OC1RS=250;
    //  OC2RS=250;
    LATFbits.LATF0 = 0; // in1
    LATFbits.LATF1 = 1; // in2
    LATFbits.LATF2 = 1; // in3
    LATFbits.LATF3 = 0; // in4
}

void idiNazad() {

    // OC1RS=385;
    // OC2RS=400;
    LATFbits.LATF0 = 1; // in1
    LATFbits.LATF1 = 0; // in2
    LATFbits.LATF2 = 0; // in3
    LATFbits.LATF3 = 1; // in4
}

void idiLevo() {

    //    OC1RS=250;
    //   OC2RS=250;
    LATFbits.LATF0 = 1; // in1
    LATFbits.LATF1 = 0; // in2
    LATFbits.LATF2 = 1; // in3
    LATFbits.LATF3 = 0; // in4
}

void idiDesno() {

    //  OC1RS=250;
    // OC2RS=250;
    LATFbits.LATF0 = 0; // in1
    LATFbits.LATF1 = 1; // in2
    LATFbits.LATF2 = 0; // in3
    LATFbits.LATF3 = 1; // in4
}

void zaustaviSve() {

    //  OC1RS = 0;
    // OC2RS = 0;
    LATFbits.LATF0 = 0; // in1
    LATFbits.LATF1 = 0; // in2
    LATFbits.LATF2 = 0; // in3
    LATFbits.LATF3 = 0; // in4
}
int start_poruka = 0;
int stop_poruka = 0;

int main(int argc, char **argv) {
    initPins();
    ADCinit();
    initUART1(); // bluetooth
    initUART2(); // za serijsku i debugging
    Init_T1();
    Init_T4();
    Init_T5();
    initPWM();

    RS232_putst("Write START");
    WriteUART1(13);

    while (1) {

        FR_back = fotootpornik(sirovi0); // promenjive za svaki fotootpornik pokazuju da li je on osvjetljen
        FR_right = fotootpornik(sirovi1);
        FR_front = fotootpornik(sirovi2);
        FR_left = fotootpornik(sirovi3);

        if (startFlag == 1 && stopFlag == 0) {

            if (start_poruka == 0) {
                RS232_putst("Tenk upaljen");
                WriteUART1(13);
                start_poruka = 1;
                stop_poruka = 0;
            }

            if (FR_back == 1 && stopFlag == 0) {

                MeasureBackDistance(); // Izmeri udaljenost unazad
                if (measured_distance_back > 13) {
                    idiNazad(); // Ako je udaljenost unazad veca od 13, kreni nazad
                    while (measured_distance_back >= 13 && stopFlag == 0) {
                        MeasureBackDistance(); // Ponovo izmeri udaljenost unazad
                        if (measured_distance_back < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unazad manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavite tenk ako je udaljenost unazad vec manja od 13
                }
            } else if (FR_right == 1 && stopFlag == 0) {

                idiDesno();
                delay_for(705000); // delay realizovan ovako da se tenk okrene za 90 stepeni
                MeasureForwardDistance(); // Izmeri udaljenost unapred
                if (measured_distance_forward >= 13) {
                    idiNapred(); // Ako je udaljenost unapred veca od 13, kreni napred
                    while (measured_distance_forward >= 13 && stopFlag == 0) {
                        MeasureForwardDistance(); // Ponovo izmeri udaljenost unapred
                        if (measured_distance_forward < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred vec manja od 13
                }
            } else if (FR_front == 1 && stopFlag == 0) {

                MeasureForwardDistance(); // Izmeri udaljenost unapred
                if (measured_distance_forward > 13) {
                    idiNapred(); // Ako je udaljenost unapred veca od 13, kreni napred
                    while (measured_distance_forward >= 13 && stopFlag == 0) {
                        MeasureForwardDistance(); // Ponovo izmeri udaljenost unapred
                        if (measured_distance_forward < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred vec manja od 13
                }
            } else if (FR_left == 1 && stopFlag == 0) {

                idiLevo();
                delay_for(705000); // delay realizovan ovako da se tenk okrene za 90 stepeni
                MeasureForwardDistance();
                if (measured_distance_forward >= 13) {
                    idiNapred(); // Ako je udaljenost unapred veca od 13, kreni napred
                    while (measured_distance_forward >= 13 && stopFlag == 0) {
                        MeasureForwardDistance(); // Ponovo izmeri udaljenost unapred
                        if (measured_distance_forward < 13) {
                            zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred manja od 13
                        }
                    }
                } else {
                    zaustaviSve(); // Zaustavi tenk ako je udaljenost unapred vec manja od 13
                }
            } else {
                zaustaviSve(); // Zaustavi tenk ako nijedan fotootpornik nije osvetljen
            }
        } else if (stopFlag == 1) {

            if (stop_poruka == 0) {
                RS232_putst("Tenk ugasen");
                WriteUART1(13);
                stop_poruka = 1;
                start_poruka = 0;
            }
            zaustaviSve();
            stopFlag = 0;
            startFlag = 0;
        }

    } // od while

    return 0;
}
