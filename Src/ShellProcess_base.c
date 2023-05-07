/*
 * ShellProcess.c
 *
 *  Created on: Jul 18, 2021
 *      Author: jjc
 */

#include "main.h"
#include <stdio.h>
#include "UartControl.h"

void ShellProcess()
{
	printf("===========================\r\n");
	printf("Welcome to Elec Academy\r\n");
	printf("===========================\r\n");
	printf(" 1. func 1 test            \r\n");
	printf(" 2. func 2 test            \r\n");
	printf(" 3. func 3 test            \r\n");
	printf(" Press function Key        \r\n");
	while(getFillBufferSize()==0){};
	switch(getBufferChar())
	{
		case '1':	printf(" func 1  \r\n");break;
		case '2':	printf(" func 2  \r\n");break;
		case '3':	printf(" func 3  \r\n");break;
		default: 	printf(" Not support function \r\n");break;
	}
}
