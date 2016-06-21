/**
 * FILENAME : Si7020.h
 *
 * DESCRIPTION : Silicon Labs Si7020 driver
 *
 * Copyright (c) 2016 Sigma Connectivity AB. All Rights Reserved.
 * 
 * CHANGES :
 * VERS DATE        WHO              DETAIL
 * 1.0  2016-03-16  Thomas Berg      First version
 *
 * NOTES:
 * Special considerations:
 * - only hold master mode supported (this may increase power consumption,
 *   but as a first implementation it is regarded as worth it)
 * - when measuring humidity, both temperature and humidity values are updated
 *
 *
 * TODO
 * - Clean up
 * - Add comments
 *
 */
#ifndef SI7020_H
#define SI7020_H

#include "app_twi.h"

// Forward declaration 
typedef struct si7020_measurement_data_s si7020_measurement_data_t;


/**
 * @brief Measurement callback prototype.
 *
 * @param     result      Result of operation (NRF_SUCCESS on success,
 *                        otherwise a relevant error code).
 * @param[in] p_user_data Pointer to user data defined in transaction
 *                        descriptor.
 */
typedef void (* si7020_measure_callback_t)(si7020_measurement_data_t * result);


/**
 * @brief Si7020 resolution descriptor.
 */
typedef enum 
{
    SI7020_RES_12_14BIT = 0x00, // RH=12bit, T=14bit
    SI7020_RES_8_12BIT  = 0x01, // RH= 8bit, T=12bit
    SI7020_RES_10_13BIT = 0x80, // RH=10bit, T=13bit
    SI7020_RES_11_11BIT = 0x81, // RH=11bit, T=11bit
    SI7020_RES_MASK     = 0x81  // Mask for res. bits (7,0) in user reg.
} si7020_resolution_t;


/**
 * @brief Si7020 measurement command descriptor.
 */
typedef enum {
    SI7020_TEMPERATURE,
    SI7020_HUMIDITY
} si7020_measurement_type_t;


/**
 * @brief Si7020 heater command descriptor.
 */
typedef enum {
    SI7020_ON,
    SI7020_OFF
} si7020_heater_command_t;


/**
 * @brief Si7020 measurement data descriptor.
 */
typedef struct si7020_measurement_data_s {
    float temperature;
    float humidity;
    si7020_measurement_type_t measurement_type;
} si7020_measurement_data_t;


/**
 * @brief Function for initializing the Si7020 temperature/humidity sensor.
 *
 * This function performs the following steps to initialize the Si7020 sensor:
 * - Soft reset
 * - Check id
 * - Set resolution
 *
 * @param[in]  app_twi      Pointer to the twi instance to be used for twi transfers.
 * @param[in]  resolution   The wanted measurement resolution.
 */
extern void si7020_init(app_twi_t *app_twi, si7020_resolution_t resolution, si7020_measure_callback_t callback);


/**
 * @brief Function for performing a measurement with the Si7020 temperature/humidity sensor.
 *
 * This function performs either a 
 * - temperature measurement or a
 * - humidity measurement (which includes a temperature measurement)
 * based on the value of the measurement_command input parameter.
 *
 * @param[in]  command   The measurement to perform.
 */
extern void si7020_measure(si7020_measurement_type_t type);


/**
 * @brief Function for switching on and off the heater of the Si7020 temperature/humidity sensor.
 *
 *
 * @param[in]  command   ON or OFF.
 */
extern void si7020_heater(si7020_heater_command_t command);

#endif // #ifndef SI7020_H
