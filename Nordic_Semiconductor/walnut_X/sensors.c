/**
 * FILENAME : Sensors.c
 *
 * DESCRIPTION : Generic sensors driver for Walnut sensors
 *
 * Copyright (c) 2016 Sigma Connectivity AB. All Rights Reserved.
 * 
 * CHANGES :
 * VERS DATE        WHO              DETAIL
 * 1.0  2016-03-16  Thomas Berg      First version
 *
 * TODO
 * - Clean up and comment code
 * - Implement app_scheduler
 *
 */

#include "sensors.h"
#include "app_error.h"
#include "app_trace.h"
#include "app_timer.h"
#include "Si7020.h"

#define APPL_LOG         app_trace_log

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define TIMER_INTERVAL   APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */


static sensors_callback_t m_callback;
static sensors_data_t m_sensor_data;

APP_TIMER_DEF(m_sensor_timer_id);

static void si7020_measure_callback(si7020_measurement_data_t * result)
{
    APPL_LOG("[Sensors]: si7020_measure_callback.\r\n");
    
    switch (result->measurement_type) {
        case SI7020_TEMPERATURE:
            // Set BLE temperature
            m_sensor_data.temperature = result->temperature;
            m_callback(SI7020_TEMP_MEAS_EVT, &m_sensor_data);
            break;
        case SI7020_HUMIDITY:
            // Set BLE temperature and humidity
            m_sensor_data.temperature = result->temperature;
            m_sensor_data.humidity = result->humidity;
            m_callback(SI7020_HUM_MEAS_EVT, &m_sensor_data);
            break;
    }
}



void sensors_init(app_twi_t * app_twi, sensors_callback_t callback)
{
    // Store callback function
    m_callback = callback;
    
    si7020_init(app_twi, SI7020_RES_12_14BIT, si7020_measure_callback);
}


void sensors_measure(void)
{
    si7020_measure(SI7020_TEMPERATURE);
    si7020_measure(SI7020_HUMIDITY);
}


static void sensors_timer_timeout_handler(void * p_context)
{
    APPL_LOG("[Sensors]: sensors_timer_timeout_handler.\r\n");
    si7020_measure(SI7020_HUMIDITY);
}


void sensors_timers_init(void)
{
    uint32_t err_code;
    
    err_code = app_timer_create(&m_sensor_timer_id, APP_TIMER_MODE_REPEATED, sensors_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

extern void sensors_timers_start(void)
{
    uint32_t err_code;
    
    err_code = app_timer_start(m_sensor_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

extern void sensors_timers_stop(void)
{
    uint32_t err_code;
    
    err_code = app_timer_stop(m_sensor_timer_id);
    APP_ERROR_CHECK(err_code);
}
