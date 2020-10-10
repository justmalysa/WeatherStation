/*
 * bme280.c
 Author: Justyna Malysa
 */

#include "bme280.h"
#include "i2c_master.h"
#include <util/delay.h>

#define BME280_WRITE 0xEC
#define BME280_READ 0xED
#define BME280_CTRL_MEAS 0xF4 // register for BME280 configuration
#define BME280_MODE_AND_OSRS 0x25 // oversampling x1 and forced mode
#define BME280_CTRL_HUM 0xF2
#define BME280_OSRS_H 0x01
#define BME280_STATUS 0xF3
#define BME280_STATUS_BUSY (1 << 3)
#define BME280_TEMP 0xFA
#define BME280_PRESS 0xF7
#define BME280_HUM 0xFD
#define BME280_CALIB_VALUE_START 0x88
#define BME280_CALIB_VALUE_COUNT 26
#define BME280_CALIB_H_START 0xE1
#define BME280_CALIB_H_COUNT 7
#define BME280_RESET_REG 0xE0
#define BME280_RESET_VALUE 0xB6

typedef struct
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
} bme280_calib_temp_t;

typedef struct
{
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} bme280_calib_press_t;

typedef struct
{
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t  dig_H6;
} bme280_calib_hum_t;

static bme280_calib_temp_t m_calib_temp;
static bme280_calib_press_t m_calib_press;
static bme280_calib_hum_t m_calib_hum;

// Returns temperature in DegC, resolution is 0.01 DegC.
// Equation taken from official documentation.
static int32_t BME280_compensate_T(int32_t adc_T, bme280_calib_temp_t const * p_calib, int32_t * p_t_fine)
{
	int32_t var1, var2, T, t_fine;
	var1 = ((((adc_T>>3) - ((int32_t)p_calib->dig_T1<<1))) * ((int32_t)p_calib->dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)p_calib->dig_T1)) * ((adc_T>>4) - ((int32_t)p_calib->dig_T1)))
	>> 12) *
	((int32_t)p_calib->dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	if (p_t_fine)
	{
		*p_t_fine = t_fine;
	}
	return T;
}

// Returns absolute pressure in hPa.
// Equation taken from official documentation.
static uint32_t BME280_compensate_P(uint32_t adc_P, bme280_calib_press_t const * p_calib, int32_t const * p_t_fine)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)*p_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)p_calib->dig_P6;
	var2 = var2 + ((var1*(int64_t)p_calib->dig_P5)<<17);
	var2 = var2 + (((int64_t)p_calib->dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)p_calib->dig_P3)>>8) + ((var1 * (int64_t)p_calib->dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)p_calib->dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)p_calib->dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)p_calib->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)p_calib->dig_P7)<<4);
	return ((uint32_t)p >> 8);
}

// Returns humidity in %RH.
// Equation taken from official documentation.
static uint32_t BME280_compensate_H(int32_t adc_H,  bme280_calib_hum_t const * p_calib, int32_t const * p_t_fine)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (*p_t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)p_calib->dig_H4) << 20) - (((int32_t)p_calib->dig_H5) *
	v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
	((int32_t)p_calib->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)p_calib->dig_H3)) >> 11) +
	((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)p_calib->dig_H2) +
	8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	((int32_t)p_calib->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return ((uint32_t)(v_x1_u32r>>12) >> 10);
}

void bme280_init(void)
{
	i2c_init();
	
	// sensor reset
	_delay_ms(3);
	i2c_start(BME280_WRITE);
	i2c_write(BME280_RESET_REG);
	i2c_write(BME280_RESET_VALUE);
	i2c_stop();
	_delay_ms(20);
	
	// calibration
	i2c_start(BME280_WRITE);
	i2c_write(BME280_CALIB_VALUE_START);
	i2c_start(BME280_READ);
	
	// read calibration values
	
	uint8_t calib_values[BME280_CALIB_VALUE_COUNT + BME280_CALIB_H_COUNT];
	
	uint8_t i;
	for (i = 0; i < BME280_CALIB_VALUE_COUNT - 1; i++)
	{
		calib_values[i] = i2c_read_ack();
	}
	calib_values[i++] = i2c_read_nack();
	i2c_stop();
	
	i2c_start(BME280_WRITE);
	i2c_write(BME280_CALIB_H_START);
	i2c_start(BME280_READ);
	for (uint8_t j = 0; j < BME280_CALIB_H_COUNT - 1; j++)
	{
		calib_values[i++] = i2c_read_ack();
	}
	calib_values[i] = i2c_read_nack();
	i2c_stop();
	
	m_calib_temp.dig_T1 = (uint16_t)calib_values[1] << 8 | calib_values[0];
	m_calib_temp.dig_T2 = (uint16_t)calib_values[3] << 8 | calib_values[2];
	m_calib_temp.dig_T3 = (uint16_t)calib_values[5] << 8 | calib_values[4];
	
	m_calib_press.dig_P1 = (uint16_t)calib_values[7] << 8 | calib_values[6];
	m_calib_press.dig_P2 = (uint16_t)calib_values[9] << 8 | calib_values[8];
	m_calib_press.dig_P3 = (uint16_t)calib_values[11] << 8 | calib_values[10];
	m_calib_press.dig_P4 = (uint16_t)calib_values[13] << 8 | calib_values[12];
	m_calib_press.dig_P5 = (uint16_t)calib_values[15] << 8 | calib_values[14];
	m_calib_press.dig_P6 = (uint16_t)calib_values[17] << 8 | calib_values[16];
	m_calib_press.dig_P7 = (uint16_t)calib_values[19] << 8 | calib_values[18];
	m_calib_press.dig_P8 = (uint16_t)calib_values[21] << 8 | calib_values[20];
	m_calib_press.dig_P9 = (uint16_t)calib_values[23] << 8 | calib_values[22];
	
	m_calib_hum.dig_H1 = calib_values[25];
	m_calib_hum.dig_H2 = (uint16_t)calib_values[27] << 8 | calib_values[26];
	m_calib_hum.dig_H3 = calib_values[28];
	m_calib_hum.dig_H4 = (uint16_t)calib_values[29] << 4 | (calib_values[30] & 0x0F);
	m_calib_hum.dig_H5 = (uint16_t)calib_values[31] << 4 | (calib_values[30] >> 4);
	m_calib_hum.dig_H6 = (uint8_t)calib_values[32];
	
	i2c_start(BME280_WRITE);
	i2c_write(BME280_CTRL_HUM);
	i2c_write(BME280_OSRS_H);
	i2c_stop();
}

void bme280_measure(void)
{
	i2c_start(BME280_WRITE);
	i2c_write(BME280_CTRL_MEAS);
	i2c_write(BME280_MODE_AND_OSRS);
	i2c_stop();
	
	uint8_t status;
	do
	{
		i2c_start(BME280_WRITE);
		i2c_write(BME280_STATUS);
		i2c_start(BME280_READ);
		status = i2c_read_nack();
		i2c_stop();
	} while (BME280_STATUS_BUSY & status);
}

int32_t bme280_temp_get(int32_t * p_t_fine)
{
	int32_t temp_raw;
	i2c_start(BME280_WRITE);
	i2c_write(BME280_TEMP);
	i2c_start(BME280_READ);
	temp_raw = (uint32_t)i2c_read_ack() << 12;
	temp_raw |= (uint32_t)i2c_read_ack() << 4;
	temp_raw |= (uint32_t)i2c_read_nack() >> 4;
	i2c_stop();
	
	return BME280_compensate_T(temp_raw, &m_calib_temp, p_t_fine);
}

uint32_t bme280_press_get(int32_t const * p_t_fine)
{
	uint32_t press_raw;
	i2c_start(BME280_WRITE);
	i2c_write(BME280_PRESS);
	i2c_start(BME280_READ);
	press_raw = (uint32_t)i2c_read_ack() << 12;
	press_raw |= (uint32_t)i2c_read_ack() << 4;
	press_raw |= (uint32_t)i2c_read_nack() >> 4;
	i2c_stop();
	
	return BME280_compensate_P(press_raw, &m_calib_press, p_t_fine);
}

uint32_t bme280_hum_get(int32_t const * p_t_fine)
{
	int32_t hum_raw;
	i2c_start(BME280_WRITE);
	i2c_write(BME280_HUM);
	i2c_start(BME280_READ);
	hum_raw = (uint32_t)i2c_read_ack() << 8;
	hum_raw |= (uint32_t)i2c_read_nack();
	i2c_stop();

	return BME280_compensate_H(hum_raw,	&m_calib_hum, p_t_fine);
}
