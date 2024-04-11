
#ifndef TIMER1_H
#define	TIMER1_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <p30fxxxx.h>
    
#define TMR1_period 10     //1/Fosc * TMR_PERIOD = Interrupt = 1us
#define TMR4_period  25000  //2.5ms == sa ovom vrednoscu do koje T4,T5 broje HC-04 ce videti do 42 cm 
#define TMR5_period  25000  // i sve vise od toga vidi kao 42 cm(ovako radi najbolje otprilike) // testirano odvojeno 
                 
    
void Init_T1(void);
void Init_T4(void);
void Init_T5(void);


#ifdef	__cplusplus
}
#endif

#endif	/* TIMER1_H */

