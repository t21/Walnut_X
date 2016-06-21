/**
 * FILENAME : Si7020.c
 *
 * DESCRIPTION : Silicon Labs Si7020 driver
 *
 * Copyright (c) 2016 Sigma Connectivity AB. All Rights Reserved.
 * 
 * CHANGES :
 * VERS DATE        WHO              DETAIL
 * 1.0  2016-03-16  Thomas Berg      First version
 *
 * TODO
 * - Implement checkId in init function
 * - Implement read of battery status?
 * - Improve error handling
 *
 */
#include "Si7020.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_trace.h"

#define SI7020_ADDR 0x40U   // 7-bit I2C address of Si7020

#define SI7020_CHKSUM_POLY 0x131  //P(x)=x^8+x^5+x^4+1 = 100110001

#define SI7020_CHIPID 0x14

#define SI7020_USER_REG_W 0xE6 // command writing user register
#define SI7020_USER_REG_R 0xE7 // command reading user register
#define SI7020_SOFT_RESET 0xFE // command soft reset
#define READ_ELECTRONIC_ID_1_1 0xFA
#define READ_ELECTRONIC_ID_1_2 0x0F
#define READ_ELECTRONIC_ID_2_1 0xFC
#define READ_ELECTRONIC_ID_2_2 0xC9
#define SI7020_HEATER_ON  0x04 // heater on
#define SI7020_HEATER_OFF 0x00 // heater off
#define SI7020_HEATER_MASK 0x04 // Mask for Heater bit(2) in user reg.

typedef struct {
    si7020_measurement_type_t measurement_type;
} si7020_user_data_t;

// I2C commands for the different measurements
uint8_t const SI7020_TEMPERATURE_HOLD_MASTER                 = 0xE3;   // command trig. temp meas. hold master
uint8_t const SI7020_HUMIDITY_HOLD_MASTER                    = 0xE5;   // command trig. humidity meas. hold master
uint8_t const SI7020_TEMPERATURE_POLL                        = 0xF3;   // command trig. temp meas. no hold master
uint8_t const SI7020_HUMIDITY_POLL                           = 0xF5;   // command trig. humidity meas. no hold master
uint8_t const SI7020_READ_TEMPERATURE_FROM_PREVIOUS_HUMIDITY = 0xE0;   // command to read temperature value from previous RH measurement

// Buffer for data read from sensors.
#define BUFFER_SIZE  6
static uint8_t m_buffer[BUFFER_SIZE];

// Pointer to twi instance
static app_twi_t * p_app_twi;

static si7020_measure_callback_t m_measurement_callback;

// Local storage of the latest measurement values
static si7020_measurement_data_t m_measurement_data;

static uint8_t m_nbr_pending_twi_trans;

// Function prototypes for local functions
static float si7020_calculate_temperature(uint16_t temperature);
static float si7020_calculate_humidity(uint16_t humidity);
static uint8_t check_crc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);

//enum etSHT2xEob
//{
//    SHT2x_EOB_ON = 0x40, // end of battery
//    SHT2x_EOB_MASK = 0x40, // Mask for EOB bit(6) in user reg.
//};


void si7020_init(app_twi_t * app_twi, si7020_resolution_t resolution, si7020_measure_callback_t callback)
{
    uint32_t err_code;
    
    // Store pointer to twi
    p_app_twi = app_twi;
    
    // Store callback
    m_measurement_callback = callback;
    
    // Enable TWI
    nrf_drv_twi_enable(&p_app_twi->twi);
    
    // SoftReset
    uint8_t const RESET_COMMAND[] = { SI7020_SOFT_RESET };
    app_twi_transfer_t const SI7020_RESET_TRANSFER[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, RESET_COMMAND, sizeof(RESET_COMMAND)/sizeof(RESET_COMMAND[0]), 0)
    };
    err_code = app_twi_perform(p_app_twi, SI7020_RESET_TRANSFER, sizeof(SI7020_RESET_TRANSFER)/sizeof(SI7020_RESET_TRANSFER[0]), NULL);
    APP_ERROR_CHECK(err_code);    
    
    // Wait for Si7020 to come out of reset
    nrf_delay_ms(15);
    
    // Check ID
    // TODO: Implement CheckId function
    
    // Read "user register 1", needed as input when setting the resolution
    uint8_t const READ_USER_REG_COMMAND[] = { SI7020_USER_REG_R };
    app_twi_transfer_t const SI7020_READ_USER_REG_1_TRANSFER[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, READ_USER_REG_COMMAND, sizeof(READ_USER_REG_COMMAND)/sizeof(READ_USER_REG_COMMAND[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(SI7020_ADDR, m_buffer, 1, 0)
    };
    err_code = app_twi_perform(p_app_twi, SI7020_READ_USER_REG_1_TRANSFER, sizeof(SI7020_READ_USER_REG_1_TRANSFER)/sizeof(SI7020_READ_USER_REG_1_TRANSFER[0]), NULL);
    APP_ERROR_CHECK(err_code);    
    
    // Set measurement resolution
    uint8_t const RESOLUTION_COMMAND[] = { SI7020_USER_REG_W, (resolution | (m_buffer[0] & ~SI7020_RES_MASK)) };
    app_twi_transfer_t const SI7020_SET_RESOLUTION_TRANSFER[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, RESOLUTION_COMMAND, sizeof(RESOLUTION_COMMAND)/sizeof(RESOLUTION_COMMAND[0]), 0)
    };
    err_code = app_twi_perform(p_app_twi, SI7020_SET_RESOLUTION_TRANSFER, sizeof(SI7020_SET_RESOLUTION_TRANSFER)/sizeof(SI7020_SET_RESOLUTION_TRANSFER[0]), NULL);
    APP_ERROR_CHECK(err_code);    

    // Disable TWI
    nrf_drv_twi_disable(&p_app_twi->twi);
}


static void si7020_measure_cb(ret_code_t result, void * p_user_data)
{
    m_nbr_pending_twi_trans--;

    if (m_nbr_pending_twi_trans == 0) {
        // Disable TWI
        nrf_drv_twi_disable(&p_app_twi->twi);
    }
    
    if (result != NRF_SUCCESS)
    {
        app_trace_log("[Si7020] Measurement failed.\r\n");
        return;
    }
    
    si7020_user_data_t user_data = *(si7020_user_data_t *)p_user_data;

    switch (user_data.measurement_type) {
        case SI7020_TEMPERATURE:
            app_trace_log("[Si7020] Measurement callback - Temperature.\r\n");
            if (check_crc(&m_buffer[0], 2, m_buffer[2]) == 0) {
                m_measurement_data.temperature = si7020_calculate_temperature(m_buffer[0] << 8 | m_buffer[1]);;
                m_measurement_data.measurement_type = SI7020_TEMPERATURE;
                m_measurement_callback(&m_measurement_data);
            }
            break;
        case SI7020_HUMIDITY:
            app_trace_log("[Si7020] Measurement callback - Humidity.\r\n");
            if (check_crc(&m_buffer[0], 2, m_buffer[2]) == 0) {
                m_measurement_data.temperature = si7020_calculate_temperature(m_buffer[3] << 8 | m_buffer[4]);
                m_measurement_data.humidity = si7020_calculate_humidity(m_buffer[0] << 8 | m_buffer[1]);
                m_measurement_data.measurement_type = SI7020_HUMIDITY;
                m_measurement_callback(&m_measurement_data);
            }
            break;
    }
}


void si7020_measure_temperature_hold_master_mode(void)
{
    uint32_t err_code;
    
    // Enable TWI
    m_nbr_pending_twi_trans++;
    nrf_drv_twi_enable(&p_app_twi->twi);
    
    static si7020_user_data_t user_data =
    {
        .measurement_type = SI7020_TEMPERATURE,
    };
    
    static app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, &SI7020_TEMPERATURE_HOLD_MASTER, 1, APP_TWI_NO_STOP),
        APP_TWI_READ (SI7020_ADDR, &m_buffer, 3, 0)
    };
    
    static app_twi_transaction_t const transaction =
    {
        .callback            = si7020_measure_cb,
        .p_user_data         = &user_data,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    err_code = app_twi_schedule(p_app_twi, &transaction);
    APP_ERROR_CHECK(err_code);
}


void si7020_measure_humidity_hold_master_mode(void)
{
    // Enable TWI
    m_nbr_pending_twi_trans++;
    nrf_drv_twi_enable(&p_app_twi->twi);
    
    static si7020_user_data_t user_data =
    {
        .measurement_type = SI7020_HUMIDITY
    };
    
    static app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, &SI7020_HUMIDITY_HOLD_MASTER, 1, APP_TWI_NO_STOP),
        APP_TWI_READ (SI7020_ADDR, &m_buffer, 3, 0),
        APP_TWI_WRITE(SI7020_ADDR, &SI7020_READ_TEMPERATURE_FROM_PREVIOUS_HUMIDITY, 1, APP_TWI_NO_STOP),
        APP_TWI_READ (SI7020_ADDR, &m_buffer[3], 2, 0)
    };
    
    static app_twi_transaction_t const transaction =
    {
        .callback            = si7020_measure_cb,
        .p_user_data         = &user_data,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(app_twi_schedule(p_app_twi, &transaction));
}


void si7020_measure(si7020_measurement_type_t type)
{
    switch (type) {
        case SI7020_TEMPERATURE:
            si7020_measure_temperature_hold_master_mode();
            break;
        case SI7020_HUMIDITY:
            si7020_measure_humidity_hold_master_mode();
            break;
    }
}


void si7020_heater(si7020_heater_command_t command)
{
    uint32_t err_code;
    uint8_t heater_command[] = { SI7020_USER_REG_W, 0 };
    
    // Enable TWI
    nrf_drv_twi_enable(&p_app_twi->twi);
    
    // Read "user register 1"
    uint8_t const READ_USER_REG_COMMAND[] = { SI7020_USER_REG_R };
    app_twi_transfer_t const SI7020_READ_USER_REG_1_TRANSFER[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, READ_USER_REG_COMMAND, sizeof(READ_USER_REG_COMMAND)/sizeof(READ_USER_REG_COMMAND[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(SI7020_ADDR, m_buffer, 1, 0)
    };
    err_code = app_twi_perform(p_app_twi, SI7020_READ_USER_REG_1_TRANSFER, sizeof(SI7020_READ_USER_REG_1_TRANSFER)/sizeof(SI7020_READ_USER_REG_1_TRANSFER[0]), NULL);
    APP_ERROR_CHECK(err_code);    
    
    // Set heater value
    switch (command) {
        case SI7020_ON:
            heater_command[1] = ( SI7020_HEATER_ON | m_buffer[0]);
            break;
        case SI7020_OFF:
            heater_command[1] = ( ~SI7020_HEATER_ON & m_buffer[0]);
            break;
    }

    // Write "user register 1"
    app_twi_transfer_t const SI7020_SET_RESOLUTION_TRANSFER[] =
    {
        APP_TWI_WRITE(SI7020_ADDR, heater_command, sizeof(heater_command)/sizeof(heater_command[0]), 0)
    };
    err_code = app_twi_perform(p_app_twi, SI7020_SET_RESOLUTION_TRANSFER, sizeof(SI7020_SET_RESOLUTION_TRANSFER)/sizeof(SI7020_SET_RESOLUTION_TRANSFER[0]), NULL);
    APP_ERROR_CHECK(err_code);    

    // Disable TWI
    nrf_drv_twi_disable(&p_app_twi->twi);
}


static float si7020_calculate_temperature(uint16_t temperature)
{
    float result = (175.25 * temperature / 65536) - 46.85;
    return result;
}


static float si7020_calculate_humidity(uint16_t humidity)
{
    float result = (125.0 * humidity / 65536) - 6;
    return result;
}



//==============================================================================//
static uint8_t check_crc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//==============================================================================//
{
    int crc = 0;
    int byteCtr;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for (int bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ SI7020_CHKSUM_POLY;
            else
                crc = (crc << 1);
        }
    }
    if (crc != checksum)
//        return CHECKSUM_ERROR;
        return 1;
    else
        return 0;
}







//static bool measureTemperatureHM(int16_t *temperature)
//{
//    bool trans_ok = true;    //error variable
//    uint8_t tx[1];
//    uint8_t rx[3];
//    uint16_t raw_temp = 0;
////    int  data[2];    //data array for checksum verification

//    // Measure humidity and temperature - receive humidty value
//    //-- write I2C sensor address and command --
//    tx[0] = TRIG_T_MEASUREMENT_HM;
//    trans_ok &= twi_master_transfer(I2C_ADR_W, tx, 1, TWI_DONT_ISSUE_STOP);

//    //-- read two data bytes and one checksum byte --
//    trans_ok &= twi_master_transfer(I2C_ADR_R, rx, 3, TWI_ISSUE_STOP);

//    raw_temp = (rx[0] << 8) | rx[1];
//    *temperature = (int16_t)((calcTemperatureC(raw_temp) + 0.5) * 10);

//    //-- verify checksum --
////    data[0] = rx[0];
////    data[1] = rx[1];
////    err |= checkCrc (data, 2, rx[2]);
//// TODO: Ta hand om checksumman, skapa korrekt fel

//    return trans_ok;
//}






/*

static bool checkId(bool *mounted)
{
    bool trans_ok = true;
    uint8_t tx[2];
    uint8_t rx[8];

    tx[0] = READ_ELECTRONIC_ID_1_1;
    tx[1] = READ_ELECTRONIC_ID_1_2;
    trans_ok &= twi_master_transfer(I2C_ADR_W, tx, 2, TWI_DONT_ISSUE_STOP);
    trans_ok &= twi_master_transfer(I2C_ADR_R, rx, 8, TWI_ISSUE_STOP);

    tx[0] = READ_ELECTRONIC_ID_2_1;
    tx[1] = READ_ELECTRONIC_ID_2_2;
    trans_ok &= twi_master_transfer(I2C_ADR_W, tx, 2, TWI_DONT_ISSUE_STOP);
    trans_ok &= twi_master_transfer(I2C_ADR_R, rx, 6, TWI_ISSUE_STOP);

    if (rx[0] == SI7020_CHIPID)
    {
        *mounted = true;
    }
    else
    {
        *mounted = false;
    }

    return trans_ok;
}



*/
