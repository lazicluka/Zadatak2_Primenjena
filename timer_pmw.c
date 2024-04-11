#include "timer_pmw.h"
#include<p30fxxxx.h>
#include <outcompare.h>
#ifdef _T2IF


unsigned int set_pwm1,set_pwm2;

void initPWM() {

    PR2 = 499; // odredjuje frekvenciju po formuli
    OC1RS = 20; // postavimo pwm
    OC1R = 200; // inicijalni pwm pri paljenju samo
    OC1CON = OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE & T2_PS_1_256; // konfiguracija pwma

    
    OC2RS = 20; // postavimo pwm
    OC2R = 200; // inicijalni pwm pri paljenju samo
    OC2CON = OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE & T2_PS_1_256; // konfiguracija pwma
    T2CONbits.TON = 1; // ukljucujemo timer koji koristi(timer 2)
 
    set_pwm1 = 380; // LEVA GUSENICA   // da bi tenk isao sto pravije jel gusenice ne rade bas isto 372
    set_pwm2 = 380; // DESNA GUSENICA   
    OC1RS = set_pwm1; // ovim postavljamo faktor ispune
    OC2RS = set_pwm2;
}



#else 
#warning "Does not build on this target"
#endif

