/*
* FreeRTOS Kernel V10.0.1
* Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* http://www.FreeRTOS.org
* http://aws.amazon.com/freertos
*
* 1 tab == 4 spaces!
*/


/* BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR IAR AVR PORT. */


#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "serial.h"
#include "usart.h"

#define serBAUD_DIV_CONSTANT			( ( unsigned long ) 16 )

/* Constants for writing to UCSRB. */
#define serRX_INT_ENABLE				( ( unsigned char ) 1 << RXCIEn )
#define serRX_ENABLE					( ( unsigned char ) 1 << RXENn )
#define serTX_ENABLE					( ( unsigned char ) 1 << TXENn )
#define serTX_INT_ENABLE				( ( unsigned char ) 1 << TXCIEn)

/* Constants for writing to UCSRC. */
//#define serUCSRC_SELECT					( ( unsigned char ) 0x80 )
#define serEIGHT_DATA_BITS				( ( unsigned char ) ((1 << UCSZn1) | (1 << UCSZn0)) )

static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

#define vInterruptOn(n)										\
{															\
	unsigned char ucByte;								\
        \
            ucByte = UCSR##n##B;											\
                ucByte |= serTX_INT_ENABLE;								\
                    outb( UCSR##n##B, ucByte );									\
}																				
/*-----------------------------------------------------------*/
//#define test(a)  hello(#a)ds

#define vInterruptOff(n)										\
{															\
	unsigned char ucByte;								\
        \
            ucByte = UCSR##n##B;											\
                ucByte &= ~serTX_INT_ENABLE;							\
                    outb( UCSR##n##B, ucByte );									\
}
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength , uint8_t uComNum)
{
    unsigned long ulBaudRateCounter;
//    unsigned char ucByte;
    
	portENTER_CRITICAL();
	{
        
		/* Create the queues used by the com test task. */
		xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
		xCharsForTx = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
        
		/* Calculate the baud rate register value from the equation in the
		data sheet. */
		ulBaudRateCounter = ( configCPU_CLOCK_HZ / ( serBAUD_DIV_CONSTANT * ulWantedBaud ) ) - ( unsigned long ) 1;
        
        switch(uComNum){
          case 0:
            /* Set the baud rate. */	
            UBRR0 = ulBaudRateCounter ;
            /* Enable the Rx interrupt.  The Tx interrupt will get enabled
            later. Also enable the Rx and Tx. */
            UCSR0B |= ((1 << TXCIEn) | (1 << RXCIEn) | (1 << RXENn) | ( 1 << TXENn));
            /* Set the data bits to 8. */
            UCSR0C |= ((1 << UCSZn1) | (1 << UCSZn0));
            break;
          case 1:
            /* Set the baud rate. */	
            UBRR1 = ulBaudRateCounter ;
            /* Enable the Rx interrupt.  The Tx interrupt will get enabled
            later. Also enable the Rx and Tx. */
            UCSR1B |= ((1 << TXCIEn) | (1 << RXCIEn) | (1 << RXENn) | ( 1 << TXENn));
            /* Set the data bits to 8. */
            UCSR1C |= ((1 << UCSZn1) | (1 << UCSZn0));
            break;
          case 2:
            /* Set the baud rate. */	
            UBRR2 = ulBaudRateCounter ;
            /* Enable the Rx interrupt.  The Tx interrupt will get enabled
            later. Also enable the Rx and Tx. */
            UCSR2B |= ((1 << TXCIEn) | (1 << RXCIEn) | (1 << RXENn) | ( 1 << TXENn));
            /* Set the data bits to 8. */
            UCSR2C |= ((1 << UCSZn1) | (1 << UCSZn0));
            
            break;
          case 3:
            /* Set the baud rate. */	
            UBRR3 = ulBaudRateCounter ;
            /* Enable the Rx interrupt.  The Tx interrupt will get enabled
            later. Also enable the Rx and Tx. */
            UCSR3B |= ((1 << TXCIEn) | (1 << RXCIEn) | (1 << RXENn) | ( 1 << TXENn));
            /* Set the data bits to 8. */
            UCSR3C |= ((1 << UCSZn1) | (1 << UCSZn0));
            break;
        }
		
	}
	portEXIT_CRITICAL();
	
	/* Unlike other ports, this serial code does not allow for more than one
	com port.  We therefore don't return a pointer to a port structure and can
	instead just return NULL. */
	return NULL;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime, uint8_t uComNum )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime , uint8_t uComNum)
{
	/* Return false if after the block time there is no room on the Tx queue. */
	if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
	{
		return pdFAIL;
	}
    
    vInterruptOn(0);
    
	return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort , uint8_t uComNum)
{    unsigned char ucByte;
    
	/* Turn off the interrupts.  We may also want to delete the queues and/or
	re-install the original ISR. */
    
    portENTER_CRITICAL();
    {
        
        switch(uComNum){
            case 0:
              vInterruptOff(0);
              ucByte = UCSR0B;
              ucByte &= ~serRX_INT_ENABLE;
              outb( UCSR0B, ucByte );
              break;
          case 1:
            vInterruptOff(1);
            ucByte = UCSR1B;
            ucByte &= ~serRX_INT_ENABLE;
            outb( UCSR1B, ucByte );
            break;
          case 2:
            vInterruptOff(2);
            ucByte = UCSR2B;
            ucByte &= ~serRX_INT_ENABLE;
            outb( UCSR2B, ucByte );
            break;
          case 3:
            vInterruptOff(3);
            ucByte = UCSR3B;
            ucByte &= ~serRX_INT_ENABLE;
            outb( UCSR3B, ucByte );
            break;
        }
        

    }
    portEXIT_CRITICAL();
}
/*************************************************************
                 UART 0   __interrupt  
 ************************************************************/
/*-----------------------------------------------------------*/
__interrupt void SIG_UART0_RECV( void )
{
    signed char ucChar, xHigherPriorityTaskWoken = pdFALSE;
    
	/* Get the character and post it on the queue of Rxed characters.
	If the post causes a task to wake force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
    ucChar = UDR0;
    
	xQueueSendFromISR( xRxedChars, &ucChar, &xHigherPriorityTaskWoken );
    
	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

__interrupt void SIG_UART0_DATA( void )
{
    signed char cChar, cTaskWoken = pdFALSE;
    
    if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
    {
        /* Send the next character queued for Tx. */
        outb( UDR0, cChar );
    }
    else
    {
        /* Queue empty, nothing to send. */
        //        vInterruptOff();
        unsigned char ucByte;					
        
        ucByte = UCSR0B;											
        ucByte &= ~serTX_INT_ENABLE;							
        outb( UCSR0B, ucByte );			
    }
}

/*************************************************************
                 UART 1   __interrupt  
 ************************************************************/
/*-----------------------------------------------------------*/
__interrupt void SIG_UART1_RECV( void )
{
    signed char ucChar, xHigherPriorityTaskWoken = pdFALSE;
    
	/* Get the character and post it on the queue of Rxed characters.
	If the post causes a task to wake force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
    ucChar = UDR1;
    
	xQueueSendFromISR( xRxedChars, &ucChar, &xHigherPriorityTaskWoken );
    
	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

__interrupt void SIG_UART1_DATA( void )
{
    signed char cChar, cTaskWoken = pdFALSE;
    
    if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
    {
        /* Send the next character queued for Tx. */
        outb( UDR1, cChar );
    }
    else
    {
        /* Queue empty, nothing to send. */
        //        vInterruptOff();
        unsigned char ucByte;					
        
        ucByte = UCSR1B;											
        ucByte &= ~serTX_INT_ENABLE;							
        outb( UCSR1B, ucByte );			
    }
}
/*************************************************************
                 UART 2   __interrupt  
 ************************************************************/
/*-----------------------------------------------------------*/
__interrupt void SIG_UART2_RECV( void )
{
    signed char ucChar, xHigherPriorityTaskWoken = pdFALSE;
    
	/* Get the character and post it on the queue of Rxed characters.
	If the post causes a task to wake force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
    ucChar = UDR2;
    
	xQueueSendFromISR( xRxedChars, &ucChar, &xHigherPriorityTaskWoken );
    
	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

__interrupt void SIG_UART2_DATA( void )
{
    signed char cChar, cTaskWoken = pdFALSE;
    
    if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
    {
        /* Send the next character queued for Tx. */
        outb( UDR2, cChar );
    }
    else
    {
        /* Queue empty, nothing to send. */
        //        vInterruptOff();
        unsigned char ucByte;					
        
        ucByte = UCSR2B;											
        ucByte &= ~serTX_INT_ENABLE;							
        outb( UCSR2B, ucByte );			
    }
}
/*************************************************************
                 UART 3   __interrupt  
 ************************************************************/
/*-----------------------------------------------------------*/
__interrupt void SIG_UART3_RECV( void )
{
    signed char ucChar, xHigherPriorityTaskWoken = pdFALSE;
    
	/* Get the character and post it on the queue of Rxed characters.
	If the post causes a task to wake force a context switch as the woken task
	may have a higher priority than the task we have interrupted. */
    ucChar = UDR3;
    
	xQueueSendFromISR( xRxedChars, &ucChar, &xHigherPriorityTaskWoken );
    
	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

__interrupt void SIG_UART3_DATA( void )
{
    signed char cChar, cTaskWoken = pdFALSE;
    
    if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
    {
        /* Send the next character queued for Tx. */
        outb( UDR3, cChar );
    }
    else
    {
        /* Queue empty, nothing to send. */
        //        vInterruptOff();
        unsigned char ucByte;					
        
        ucByte = UCSR3B;											
        ucByte &= ~serTX_INT_ENABLE;							
        outb( UCSR3B, ucByte );			
    }
}