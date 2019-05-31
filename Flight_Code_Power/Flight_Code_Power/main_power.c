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
#include "twi_master_driver.h"

/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
USART_data_t USART_datac0;
uint8_t usartc0_receive=0x01;
uint8_t op_mode=preflight_mode;
uint16_t Battery_volatge=0x09;
uint16_t v_threshold=0x0A;
uint8_t uplink_command=0xAA;
uint8_t emergency=0xFF;
uint8_t w;
uint8_t image_upload;
uint8_t RBF ;  
uint8_t SNAP;
uint8_t ads_deploy; 
uint8_t HM_Data;
uint8_t SendBuffer[3] ={0b00000000,0b00000001,0x01};
uint8_t sat_pos;  //defined to complete the flow chart
uint8_t range_India; // defined to complete the flow chart



int main(void)
{   /* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	&TWIC,
	TWI_MASTER_INTLVL_LO_gc,
	TWI_BAUDSETTING);
	USART_ENABLE_C();
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	sei();
    /* Replace with your application code */
	while (1) 
    {   RBF=0X00;
		SNAP=0X00;
		
		if ( RBF & 0x01)
		{	
			op_mode=preflight_mode;
		}
		else if (SNAP & 0x02)
		{
			op_mode=launch_mode;
		}
		else if (uplink_command==KILL)
		{
			op_mode=kill_mode;
		}
		else if (uplink_command==RESTART)
		{
			op_mode=restart_mode;
		}
		else if (emergency==0x11)  //defined randomly
		{
			op_mode=emergency_mode;
		}
		else if (Battery_volatge>v_threshold)
		{
			v_threshold=v_3E3v;
			if (w>w_threshold)
			{
				if (ads_deploy==0xA0) //defined randomly
				{
					op_mode=detumb_post_mode;
				}
				else 
				{
					op_mode=detumb_pre_mode;
				}
			}
			else 
			{
				if(sat_pos>range_India)
				{
					if(image_upload==0XFF)      //defined randomly
					{
						op_mode=nom_uplink_mode;
					}
					else
					{
						op_mode=nom_transmit_mode;
					}
				}
				else
				{
					op_mode=nom_idle_mode;
				}
			}
		}
		else
		{
			if (op_mode==extreme_low_power_mode)
			{
				op_mode=extreme_low_power_mode;
				v_threshold=v_4v;
			}
			else if (Battery_volatge>v_threshold_2)
			{
				op_mode=low_power_mode;
				v_threshold=v_3E5v;
			}
			else 
			{
				op_mode=extreme_low_power_mode;
				v_threshold=v_4v;
			}
		}
		USART_TXBuffer_PutByte(&USART_datac0,op_mode);
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
		case emergency_mode: USART_TXBuffer_PutByte(&USART_datac0,'P'); 
		break;
		case restart_mode:USART_TXBuffer_PutByte(&USART_datac0,'J');
		break; 
		case kill_mode:USART_TXBuffer_PutByte(&USART_datac0,'K'); 
		break;
		}
		SendBuffer[2]=usartc0_receive;
		TWI_MasterWriteRead(&twiMaster,                                      //added by me
		EEPROM_ADDRESS,
		&SendBuffer[0],
		3,
		0);
		while (twiMaster.status != TWIM_STATUS_READY)
		{
		}
		UART_TXBuffer_PutByte(&USART_datac0, 'P');
		_delay_ms(500) ;
		TWI_MasterWriteRead(&twiMaster,                                      //added by me
		EEPROM_ADDRESS,
		&SendBuffer[0],
		2,
		1);
		while (twiMaster.status != TWIM_STATUS_READY)
		{
		}
		uplink_command=twiMaster.readData[0];
		UART_TXBuffer_PutByte(&USART_datac0, uplink_command);	                     // send data 
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

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

