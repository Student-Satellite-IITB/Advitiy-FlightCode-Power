/*
 * Flight_Code_Power.c
 *
 * Created: 29-05-2019 12:33:45
 * Author : PRASHANT KURREY
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
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
USART_data_t USART_datac1;
USART_data_t USART_dataf0;
uint8_t usartc0_receive=0x01;
uint8_t op_mode=PREFLIGHT_MODE;       //next state of oprational mode
uint8_t pre_op_mode=PREFLIGHT_MODE;  //previous state of operation mode
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
uint8_t SendBuffer[3] ={0b00000000,0b00000001,0x43};
uint8_t sat_pos;  //defined to complete the flow chart
uint8_t range_India; // defined to complete the flow chart





int main(void)
{   
	//POWER_SAVE_ENABLE();
	EXTERNAL_INT_PINDO();
	//USART_ENABLE_C();
	USART0_INIT(&PORTC,USART_datac0, USARTC0);            //for usart connected to preflight
	USART1_INIT(&PORTC,USART_datac1, USARTC1);            //for usart connected to OBC
	USART0_INIT(&PORTF,USART_dataf0, USARTF0);			  //for usart connected to comm board
	
	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	&TWIC,
	TWI_MASTER_INTLVL_LO_gc,
	TWI_BAUDSETTING);
	
	/* INitialize ADC */
	ADC_INIT_ADC(&ADCB, &ADCB.CH0 );              // for all the pins channel will be 0
	
	/* Global interrupt*/
	PMIC.CTRL |= PMIC_MEDLVLEN_bm|PMIC_LOLVLEN_bm;
	sei();
	
	
	/* take special data from EEPROM */
	TWI_MasterWriteRead(&twiMaster,                                      //added by me
	EEPROM_ADDRESS,
	&SendBuffer[0],
	2,
	1);
	while (twiMaster.status != TWIM_STATUS_READY);
	
	SNAP= twiMaster.readData[0];								//if SNAP=0x41 then SNAP has been detected so no need to go sleep mode after reset
	

	if (SNAP!=0x43)
	{
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		POWER_SAVE_CLK_ENABLE();
		sleep_mode();
	}
	
	WDT_EnableAndSetTimeout(WDT_PER_2KCLK_gc); // timer set for 2s
	
    /* Replace with your application code */
	while (1) 
    {
			if ( RBF & 0x01)
			{
				op_mode=PREFLIGHT_MODE;
			}
			else if (SNAP & 0xC0)
			{
				op_mode=LAUNCH_MODE;
			}
			else if (uplink_command==KILL)
			{
				op_mode=KILL_MODE;
			}
			else if (uplink_command==RESTART)
			{
				op_mode=RESTART_MODE;
			}
			else if (emergency==0x11)  //defined randomly
			{
				op_mode=EMERGENCY_MODE;
			}
			else if (Battery_volatge>v_threshold)
			{
				v_threshold=V_3E3V;
				if (w>W_THRESHOLD)
				{
					if (ads_deploy==0xA0) //defined randomly
					{
						op_mode=DETUMB_POST_MODE;
					}
					else
					{
						op_mode=DETUMB_PRE_MODE;
					}
				}
				else
				{
					if(sat_pos>range_India)
					{
						if(image_upload==0XFF)      //defined randomly
						{
							op_mode=NOM_UPLINK_MODE;
						}
						else
						{
							op_mode=NOM_TRANSMIT_MODE;
						}
					}
					else
					{
						op_mode=NOM_IDLE_MODE;
					}
				}
			}
			else
			{
				if (op_mode==EXTREME_LOW_POWER_MODE)
				{
					op_mode=EXTREME_LOW_POWER_MODE;
					v_threshold=V_4V;
				}
				else if (Battery_volatge>V_THRESHOLD_2)
				{
					op_mode=LOW_POWER_MODE;
					v_threshold=V_3E5V;
				}
				else
				{
					op_mode=EXTREME_LOW_POWER_MODE;
					v_threshold=V_4V;
				}
			}
			
			if(op_mode==pre_op_mode)                    
			{
				
				pre_op_mode=op_mode;
				USART_TXBuffer_PutByte(&USART_datac0,'X');
				_delay_ms(10);
				
			}
			else
			{
				
				switch(op_mode)
				{
					case PREFLIGHT_MODE: USART_TXBuffer_PutByte(&USART_datac0,'A');
					break;
					case LAUNCH_MODE:   //op_mode=DETUMB_PRE_MODE; 
										USART_TXBuffer_PutByte(&USART_datac0,'B');
					break;
					
					case DETUMB_PRE_MODE:USART_TXBuffer_PutByte(&USART_datac0,'C');
					break;
					
					case DETUMB_POST_MODE:USART_TXBuffer_PutByte(&USART_datac0,'D');
					break;
					
					case NOM_IDLE_MODE: 
							if (pre_op_mode==EXTREME_LOW_POWER_MODE)
							{
								PORTE.OUTSET=0x55;
								PORTF.OUTSET=0xE2;
								PORTD.OUTSET=0x40;
								PORTC.OUTSET=0x18;
							}
							else if (pre_op_mode==LOW_POWER_MODE)
							{
								PORTE.OUTSET=0x55;
								PORTF.OUTSET=0xE2;
							}
							USART_TXBuffer_PutByte(&USART_datac0,'E');
					break;
					
					case NOM_TRANSMIT_MODE:
							if (pre_op_mode==EXTREME_LOW_POWER_MODE)
							{
								PORTE.OUTSET=0x55;
								PORTF.OUTSET=0xE2;
								PORTD.OUTSET=0x40;
								PORTC.OUTSET=0x18;
							}
							else if (pre_op_mode==LOW_POWER_MODE)
							{
								PORTE.OUTSET=0x55;
								PORTF.OUTSET=0xE2;
							}
							USART_TXBuffer_PutByte(&USART_datac0,'F');
					break;
					
					case NOM_UPLINK_MODE:
							if (pre_op_mode==EXTREME_LOW_POWER_MODE)
							{
								PORTE.OUTSET=0x55;
								PORTF.OUTSET=0xE2;
								PORTD.OUTSET=0x40;
								PORTC.OUTSET=0x18;
							}
							else if (pre_op_mode==LOW_POWER_MODE)
							{
								PORTE.OUTSET=0x55;
								PORTF.OUTSET=0xE2;
							}
							USART_TXBuffer_PutByte(&USART_datac0,'G');
					break;
					
					case LOW_POWER_MODE:
							PORTE.OUTCLR=0x55;
							PORTF.OUTCLR=0xE2;
							USART_TXBuffer_PutByte(&USART_datac0,'H');     //
					break;
					
					case EXTREME_LOW_POWER_MODE:
							// Switch off all the components
							PORTE.OUTCLR=0x55;
							PORTF.OUTCLR=0xE2;
							PORTD.OUTCLR=0x40;
							PORTC.OUTCLR=0x18;						
							USART_TXBuffer_PutByte(&USART_datac0,'I');
					break;
					case EMERGENCY_MODE: USART_TXBuffer_PutByte(&USART_datac0,'P');
					break;
					
					case RESTART_MODE:
							// Switch off all the components
							PORTE.OUTCLR=0x55;
							PORTF.OUTCLR=0xE2;
							PORTD.OUTCLR=0x40;
							PORTC.OUTCLR=0x18;
							// Switch off for some time
							_delay_ms(10);
							// Power on all
							PORTE.OUTSET=0x55;
							PORTF.OUTSET=0xE2;
							PORTD.OUTSET=0x40;
							PORTC.OUTSET=0x18;
							USART_TXBuffer_PutByte(&USART_datac0,'J');
					break;
					
					case KILL_MODE:
							// Switch off all the components
							PORTE.OUTCLR=0x55;
							PORTF.OUTCLR=0xE2;
							PORTD.OUTCLR=0x40;
							PORTC.OUTCLR=0x18;
							while(1);           
							USART_TXBuffer_PutByte(&USART_datac0,'K');
					break;
					
				}
				
				pre_op_mode=op_mode; //update previous operation mode
			}
			
			
			
			Battery_volatge= ADC_DATA(&ADCB.CH0, ADC_CH_MUXPOS_PIN8_gc);
			//Battery_current= ADC_DATA(&ADCB.CH0, ADC_CH_MUXPOS_PIN9_gc);
			//Battery_temp= ADC_DATA(&ADCB.CH0, ADC_CH_MUXPOS_PIN10_gc);
			//temp_vol_reg_amp= 0x1234;//ADC_DATA(&ADCA.CH0, ADC_CH_MUXPOS_PIN3_gc);
			//temp_power_amp= 0x5678;//ADC_DATA(&ADCA.CH0, ADC_CH_MUXPOS_PIN4_gc);
			//HM_Data[0]=fourinput_8bit_HM( (PORTC.IN >>4), HM_Data[0]);
			//HM_Data[1]=eightinput_8bit_HM(PORTD.IN,HM_Data[1]);
			//HM_Data[2]=eightinput_8bit_HM(PORTE.IN,HM_Data[2]);
			//HM_Data[3]=(Battery_volatge>>8); //MSB
			//HM_Data[4]=Battery_volatge;      //LSB
			//HM_Data[5]=(Battery_current>>8); //MSB
			//HM_Data[6]=Battery_current;      //LSB
			//HM_Data[7]=(Battery_temp>>8); //MSB
			//HM_Data[8]=Battery_temp;      //LSB
			//HM_Data[9]=(temp_vol_reg_amp>>8); //MSB
			//HM_Data[10]=temp_vol_reg_amp;      //LSB
			//HM_Data[11]=(temp_power_amp>>8); //MSB
			//HM_Data[12]=temp_power_amp;      //LSB
			//
			//for (int i=0; i<=12;i++)
			//{
			//UART_TXBuffer_PutByte(&USART_datac0, HM_Data[i]);
			//_delay_ms(20);
			//}
			
		
		
		
		

		
		UART_TXBuffer_PutByte(&USART_datac0, Battery_volatge);
		_delay_ms(100) ;
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
		//
		
		
		WDT_Reset();  //watchdog timer will reset, if not then system will reset
    }
	
	
}

ISR(PORTD_INT0_vect)
{
	POWER_SAVE_CLK_DISABLE();
	_delay_ms(100);
	UART_TXBuffer_PutByte(&USART_datac0, 'Z');
	TWI_MasterWriteRead(&twiMaster,                                      //added by me
	EEPROM_ADDRESS,
	&SendBuffer[0],
	3,
	0);
}

ISR(USARTC0_RXC_vect)
{ 
	USART_RXComplete(&USART_datac0);
	if (USART_RXBufferData_Available(&USART_datac0)) 
	{                                               // modified by  me
		usartc0_receive = USART_RXBuffer_GetByte(&USART_datac0); // receive the data      // modified
	}                  
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

