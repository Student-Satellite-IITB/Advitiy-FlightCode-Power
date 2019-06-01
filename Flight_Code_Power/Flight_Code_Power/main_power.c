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
#include "wdt_driver.h"
#include "adc_driver.h"
#include "hm_data.h"

/* Global variables */
TWI_Master_t twiMaster;           /*!< TWI master module. */
USART_data_t USART_datac0;
uint8_t usartc0_receive=0x01;
uint8_t op_mode=preflight_mode;
uint16_t Battery_volatge=0x09;
uint16_t Battery_current;
uint16_t Battery_temp;
uint16_t temp_power_amp;
uint16_t temp_vol_reg_amp;
uint16_t v_threshold=0x0A;
uint8_t uplink_command=0xAA;
uint8_t emergency=0xFF;
uint8_t w;
uint8_t image_upload;
uint8_t RBF;  
uint8_t SNAP;
uint8_t ads_deploy; 
uint8_t HM_Data[20]={0};
uint8_t SendBuffer[3] ={0b00000000,0b00000001,0x01};
uint8_t sat_pos;  //defined to complete the flow chart
uint8_t range_India; // defined to complete the flow chart





int main(void)
{   WDT_EnableAndSetTimeout(WDT_PER_2KCLK_gc); // timer set for 2s
	/* Initialize TWI master. */
	//TWI_MasterInit(&twiMaster,
	//&TWIC,
	//TWI_MASTER_INTLVL_LO_gc,
	//TWI_BAUDSETTING);
	USART_ENABLE_C();
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	sei();
	PORTD_DIR=0x00;
	PORTE_DIR=0x00;
	PORTF_DIR=0xFF;
    /* Replace with your application code */
	while (1) 
    {   RBF=0X00;
		if ( RBF & 0x01)
		{	
			op_mode=preflight_mode;
		}
		else if (HM_Data[0] & 0xC0)
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
		
		ADC_INIT_ADC(&ADCB, &ADCB.CH0 );
		Battery_volatge= ADC_DATA(&ADCB.CH0, ADC_CH_MUXPOS_PIN8_gc);
		Battery_current= ADC_DATA(&ADCB.CH0, ADC_CH_MUXPOS_PIN9_gc);
		Battery_temp= ADC_DATA(&ADCB.CH0, ADC_CH_MUXPOS_PIN10_gc);
		temp_vol_reg_amp= 0x1234;//ADC_DATA(&ADCA.CH0, ADC_CH_MUXPOS_PIN3_gc);
		temp_power_amp= 0x5678;//ADC_DATA(&ADCA.CH0, ADC_CH_MUXPOS_PIN4_gc);
		HM_Data[0]=fourinput_8bit_HM( (PORTC.IN >>4), HM_Data[0]);
		HM_Data[1]=eightinput_8bit_HM(PORTD.IN,HM_Data[1]);
		HM_Data[2]=eightinput_8bit_HM(PORTE.IN,HM_Data[2]);
		HM_Data[3]=(Battery_volatge>>8); //MSB
		HM_Data[4]=Battery_volatge;      //LSB
		HM_Data[5]=(Battery_current>>8); //MSB
		HM_Data[6]=Battery_current;      //LSB
		HM_Data[7]=(Battery_temp>>8); //MSB
		HM_Data[8]=Battery_temp;      //LSB
		HM_Data[9]=(temp_vol_reg_amp>>8); //MSB
		HM_Data[10]=temp_vol_reg_amp;      //LSB
		HM_Data[11]=(temp_power_amp>>8); //MSB
		HM_Data[12]=temp_power_amp;      //LSB
		
		for (int i=0; i<=12;i++)
		{
			UART_TXBuffer_PutByte(&USART_datac0, HM_Data[i]);
			_delay_ms(50);
		}
		PORTF_OUTSET=0x0F;
		//SendBuffer[2]=usartc0_receive;
		//TWI_MasterWriteRead(&twiMaster,                                      //added by me
		//EEPROM_ADDRESS,
		//&SendBuffer[0],
		//3,
		//0);
		//while (twiMaster.status != TWIM_STATUS_READY)
		//{
		//}
		//UART_TXBuffer_PutByte(&USART_datac0, 'P');
		//_delay_ms(500) ;
		//TWI_MasterWriteRead(&twiMaster,                                      //added by me
		//EEPROM_ADDRESS,
		//&SendBuffer[0],
		//2,
		//1);
		//while (twiMaster.status != TWIM_STATUS_READY)
		//{
		//}
		//uplink_command=twiMaster.readData[0];
		//UART_TXBuffer_PutByte(&USART_datac0, uplink_command);	                     // send data 
		//
		
		
		WDT_Reset();  //watchdog timer will reset, if not then system will reset
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

