#include "UART.h"

void initUART1(void) // za bluetooth
{
    U1BRG = 0x0040; // ovim odredjujemo baudrate

    U1MODEbits.ALTIO = 1; // biramo koje pinove koristimo za komunikaciju osnovne ili alternativne,koristimo alternativne

    IEC0bits.U1RXIE = 1; // omogucavamo rx1 interupt

    U1STA &= 0xfffc;

    U1MODEbits.UARTEN = 1; // ukljucujemo ovaj modul

    U1STAbits.UTXEN = 1; // ukljucujemo predaju
}

void initUART2(void) // za serijsku
{
    U2BRG = 0x0040; // baud rate 9600

    IEC1bits.U2RXIE = 1; // omogucavamo rx2 interupt

    U2STA &= 0xfffc;

    U2MODEbits.UARTEN = 1; // ukljucujemo ovaj modul

    U2STAbits.UTXEN = 1; // ukljucujemo predaju
}

void WriteUART1(unsigned int data)
{
    while (!U1STAbits.TRMT)
        ;

    if (U1MODEbits.PDSEL == 3)
        U1TXREG = data;
    else
        U1TXREG = data & 0xFF;
}

void WriteUART2(unsigned int data)
{
    while (!U2STAbits.TRMT)
        ;

    if (U2MODEbits.PDSEL == 3)
        U2TXREG = data;
    else
        U2TXREG = data & 0xFF;
}

void RS232_putst(register const char *str1) // funkcija za ispis na terminal
{
    while ((*str1) != 0)
    {
        WriteUART1(*str1++);
        // str1++;
        // WriteUART1(13);
    }
}

void RS232_putst2(register const char *str2)
{
    while ((*str2) != 0)
    {
        WriteUART2(*str2++);
        //  str2++;
        // WriteUART2(13);
    }
}

void WriteUART1dec2string(unsigned int data)
{
    unsigned char temp;

    temp = data / 1000;
    WriteUART1(temp + '0');
    data = data - temp * 1000;
    temp = data / 100;
    WriteUART1(temp + '0');
    data = data - temp * 100;
    temp = data / 10;
    WriteUART1(temp + '0');
    data = data - temp * 10;
    WriteUART1(data + '0');
    WriteUART1(13); // novi red
}

void WriteUART2dec2string(unsigned int data)
{
    unsigned char temp;

    temp = data / 1000;
    WriteUART2(temp + '0');
    data = data - temp * 1000;
    temp = data / 100;
    WriteUART2(temp + '0');
    data = data - temp * 100;
    temp = data / 10;
    WriteUART2(temp + '0');
    data = data - temp * 10;
    WriteUART2(data + '0');
    WriteUART2(13);
}
