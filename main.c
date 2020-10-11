/*
 * main.c
 */

#include "stdio.h"
#include "i2c_master.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "USART.h"
#include "bme280.h"

#define MAX_DIGITS_IN_INT_32 13 //this value contains the number of characters of the 32 bit number, 
								//the end of line character, dot and number sign

void number_to_string(int32_t number, char string[], uint8_t dec_point)
{
	if (number < 0)
	{
		number = -number;
		string[0] = '-';
		string++;
	}
	
	uint8_t length = 0;
	while (number)
	{
		if (dec_point && length == dec_point)
		{
			string[length++] = '.';
		}
		string[length++] = number%10 + '0';
		number /= 10;
	}
	
	for (uint8_t i = 0; i<(length>>1); i++)
	{
		char value = string[i] ;
		string[i] = string[length-1-i];
		string[length-1-i] = value;
	}
	
	string[length] = '\0';
}

void send_measurments(void)
{
	char string[MAX_DIGITS_IN_INT_32];
	int32_t t_fine; //temperature in special format needed in humidity and pressure compensation
    bme280_measure();
        
	//temperature
	int32_t temp_C = bme280_temp_get(&t_fine);
	number_to_string(temp_C, string, 2);
	printString("Temperature: ");
	printString(string);
	printString((" deg C\n"));
		
	//pressure
	uint32_t press_hPa = bme280_press_get(&t_fine);
	number_to_string(press_hPa, string, 2);
	printString("Absolute pressure: ");
	printString(string);
	printString((" hPa\n"));
		
	//humidity
	int32_t hum_RH = bme280_hum_get(&t_fine);
	number_to_string(hum_RH, string, 0);
	printString("Humidity: ");
	printString(string);
	printString((" %RH\n"));	

}

int main(void)
{
    initUSART();
	bme280_init();
	
    while (1)
	{
		send_measurments();
    }
}
