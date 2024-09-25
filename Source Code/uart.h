#ifndef UART_H_
#define UART_H_


#include "inc/tm4c123gh6pm.h"

void uart_init(void);

char readChar(void);

void printChar(char c);

void printString(char *string);

#endif /* UART_H_ */
