#include<stdio.h>

#include "UART.h"

void uart_init(void)
{
    SYSCTL_RCGCUART_R |= (1<<0);
    SYSCTL_RCGCGPIO_R |= (1<<0);
    GPIO_PORTA_AFSEL_R |= (1<<1)|(1<<0);
    GPIO_PORTA_PCTL_R |= (1<<0)|(1<<4);
    GPIO_PORTA_DEN_R |= (1<<0)|(1<<1);
    UART0_CTL_R =0x0;
    UART0_IBRD_R = 104;
    UART0_FBRD_R = 11;
    //UART0_LCRH_R = 0x60;
    UART0_LCRH_R = (0x03<<5)|(1<<4);
    UART0_CC_R = 0x00;
    //UART0_CTL_R = 0x301;
    UART0_CTL_R = (1<<0)|(1<<8)|(1<<9);
}

char readChar(void)
{
    char c;
    while((UART0_FR_R & (1<<4)) != 0);
    c = UART0_DR_R;
    return c;
}

void printChar(char c)
{
    while((UART0_FR_R & (1<<5)) != 0);
    UART0_DR_R =c;
}

void printString(char *string)
{
    while(*string!='\0')
    {
        printChar(*(string++));
    }
}
