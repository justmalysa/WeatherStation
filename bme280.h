/*
 * bme280.h
 * Author: Justyna Malysa
 */

#ifndef BME280_H
#define BME280_H

#include <stdint.h>

/**
 * @brief Function for initializing BME280 sensor.
 */
void bme280_init(void);

/**
 * @brief Function for performing a raw measurement of temperature, pressure and humidity.
 *
 * To get the actual measurement results use the following functions:
 * - temperature - @ref bme280_temp_get()
 * - pressure - @ref bme280_press_get()
 * - humidity - @ref bme280_hum_get() 
 * 
 * @note Keep in mind that in order to calculate correct pressure and humidity values,
 *		 temperature needs to be calculated first.
 */
void bme280_measure(void);

/**
 * @brief Function for getting the temperature measurement result.
 *
 * @note This function can be used only after calling the @ref bme280_measure() function.
 *
 * @param[out] p_t_fine Pointer to the variable where the temperature for pressure and humidity
 *						compensation will be stored. Pass NULL if only temperature is measured.
 *
 * @return Temperature in Celsius degrees, resolution is 0.01 DegC.
 */
int32_t bme280_temp_get(int32_t * p_t_fine);

/**
 * @brief Function for getting the pressure measurement result.
 *
 * @note This function can be used only after calling the @ref bme280_measure() function.
 * @note Pressure returned by this function is absolute. In order to calculate relative pressure 
 *		 altitude needs to be taken into account.
 *
 * @param[in] p_t_fine Pointer to the variable where the temperature for pressure compensation is stored.
 *					   This variable can be obtained using the @ref bme280_temp_get(). 
 *
 * @return Pressure in hPa, resolution is 0.01 hPa.
 */
uint32_t bme280_press_get(int32_t const * p_t_fine);

/**
 * @brief Function for getting the humidity measurement result.
 *
 * @note This function can be used only after calling the @ref bme280_measure() function.
 *
 * @param[in] p_t_fine Pointer to the variable where the temperature for humidity compensation is stored.
 *					   This variable can be obtained using the @ref bme280_temp_get(). 
 *
 * @return Humidity in %RH, resolution is 1%.
 */
uint32_t bme280_hum_get(int32_t const * p_t_fine);

#endif