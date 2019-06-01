/*
 * hm_data.c
 *
 * Created: 01-06-2019 18:33:44
 *  Author: PRASHANT KURREY
 */ 
#include "hm_data.h"

uint8_t fourinput_8bit_HM(uint8_t port_value, uint8_t result)            //to find the over current status and on/off
{
	if (port_value&0x01)
	{
		result=result|0x03;                 
	};
	if (port_value&0x02)
	{
		result=result|0x0C;					
	};
	if (port_value&0x30)
	{
		result=result|0x04;					
	};
	if (port_value&0x08)
	{
		result=result|0xC0;					
	};
	if (port_value&0x00)
	{
		result=result|0x00;
	};
	
	
	return result;
}


uint8_t eightinput_8bit_HM(uint8_t port_value, uint8_t result)            //to find the over current status and on/off 
{
	if (port_value&0x01)
	{
		result=result|0x01;                 
	};
	if (port_value&0x02)
	{
		result=result|0x02;				
	};
	if (port_value&0x04)
	{
		result=result|0x04;					
	};
	if (port_value&0x08)
	{
		result=result|0x08;					
	};
	if (port_value&0x10)
	{
		result=result|0x10;				
	};
	if (port_value&0x20)
	{
		result=result|0x20;									
	};
	if (port_value&0x40)
	{
		result=result|0x40;					
	};
	if (port_value&0x80)
	{
		result=result|0x80;					
	};
	if (port_value==0x00)
	{
		result=result|0x00;
	};
	
	return result;
}

