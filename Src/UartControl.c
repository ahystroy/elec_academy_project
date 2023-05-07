/*
 * UartControl.c
 *
 *  Created on: Jul 18, 2021
 *      Author: jjc
 */

#include "main.h"

#define UART_MAX_BUFFER_SIZE		16

extern UART_HandleTypeDef huart2;
uint8_t iUartBuffer;
uint8_t UartBuffer[UART_MAX_BUFFER_SIZE];
uint16_t UartHead;
uint16_t UartTail;

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 200);
	return 1;
}

void UartReceiveStart(UART_HandleTypeDef *huart)
{
	UartHead = 0;
	UartTail = 0;
	HAL_UART_Receive_IT(huart, &iUartBuffer, 1);
}

uint16_t getEmptyBufferSize()
{
	if(UartHead >= UartTail)
	{
		return (UART_MAX_BUFFER_SIZE-UartHead) + UartTail;
	}
	else
	{
		return (UartTail - UartHead);
	}
}

uint16_t getFillBufferSize()
{
	if(UartHead >= UartTail)
	{
		return (UartHead- UartTail);
	}
	else
	{
		return (UART_MAX_BUFFER_SIZE-UartTail) + UartHead;
	}
}

uint8_t getBufferChar()
{
	uint8_t retChar = UartBuffer[UartTail];
	UartTail++;
	if(UartTail>=UART_MAX_BUFFER_SIZE)
	{
		UartTail = 0;
	}
	return retChar;
}

// Uart interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(getEmptyBufferSize()>1)		/* Check overflow */
	{
		UartBuffer[UartHead] = iUartBuffer;
		UartHead++;
		if(UartHead >= UART_MAX_BUFFER_SIZE)
		{
			UartHead = 0;
		}
		HAL_UART_Receive_IT(huart, &iUartBuffer, 1);
	}
}
