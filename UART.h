
#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

#include<p30fxxxx.h>
    
void initUART1(void);
void initUART2(void);

void WriteUART1(unsigned int data);
void WriteUART2(unsigned int data);

void WriteUART1dec2string(unsigned int data);
void WriteUART2dec2string(unsigned int data);

void RS232_putst(register const char *str1);
void RS232_putst2(register const char *str2);


#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

