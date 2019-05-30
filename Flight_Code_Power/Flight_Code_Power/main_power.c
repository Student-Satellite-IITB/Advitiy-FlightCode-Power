/*
 * Flight_Code_Power.c
 *
 * Created: 29-05-2019 12:33:45
 * Author : PRASHANT KURREY
 */ 

#include <avr/io.h>
#include "common.h"
#include "avr_compiler.h"
#include "usart.h"

USART_data_t USART_datac0;
uint8_t usartc0_receive;
uint8_t op_mode;
	

int main(void)
{  
	USART_ENABLE_C();
    /* Replace with your application code */
    while (1) 
    {   USART_TXBuffer_PutByte(&USART_datac0,'Z');
		_delay_ms(500) ;
		switch(op_mode){
		case preflight_mode: USART_TXBuffer_PutByte(&USART_datac0,'A');
		break; 
		case launch_mode:    USART_TXBuffer_PutByte(&USART_datac0,'B'); 
		break;
		case detumb_pre_mode:USART_TXBuffer_PutByte(&USART_datac0,'C');
		break; 
		case detumb_post_mode:USART_TXBuffer_PutByte(&USART_datac0,'D');
		break; 
		case nom_idle_mode: USART_TXBuffer_PutByte(&USART_datac0,'E'); 
		break;
		case nom_transmit_mode:USART_TXBuffer_PutByte(&USART_datac0,'F');
		break; 
		case nom_uplink_mode:USART_TXBuffer_PutByte(&USART_datac0,'G'); 
		break;
		case low_power_mode:USART_TXBuffer_PutByte(&USART_datac0,'H'); 
		break;
		case extreme_low_power_mode:USART_TXBuffer_PutByte(&USART_datac0,'I'); 
		break;
		case restart_mode:USART_TXBuffer_PutByte(&USART_datac0,'J');
		break; 
		case kill_mode:USART_TXBuffer_PutByte(&USART_datac0,'K'); 
		break;
		}	
		if(usartc0_receive=='A'){
			op_mode=preflight_mode;
		}
		if(usartc0_receive=='D'){
			op_mode=kill_mode;
		}
		if(usartc0_receive=='B'){
			op_mode=restart_mode;
		}
	
    }
}

ISR(USARTC0_RXC_vect)
{ 
	USART_RXComplete(&USART_datac0);
	if (USART_RXBufferData_Available(&USART_datac0)) {                                               // modified by  me
		usartc0_receive = USART_RXBuffer_GetByte(&USART_datac0);}                  // receive the data      // modified
	UART_TXBuffer_PutByte(&USART_datac0, usartc0_receive);	                     // send data 
	
}
/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_datac0);
}


