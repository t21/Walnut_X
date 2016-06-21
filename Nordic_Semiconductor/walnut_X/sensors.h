#ifndef SENSORS_H__
#define SENSORS_H__

#include "app_twi.h"

typedef enum {
    SI7020_TEMP_MEAS_EVT,
    SI7020_HUM_MEAS_EVT,
} sensors_evt_t;

//Forward declaration of a few types
typedef struct sensors_data_s sensors_data_t;

/**
 * @brief Sensor callback prototype.
 *
 * @param     result      Result of operation (NRF_SUCCESS on success,
 *                        otherwise a relevant error code).
 * @param[in] p_user_data Pointer to user data defined in transaction
 *                        descriptor.
 */
typedef void (* sensors_callback_t)(sensors_evt_t evt, sensors_data_t *data);

typedef struct sensors_data_s {
    float temperature;
    float humidity;
} sensors_data_t;

extern void sensors_init(app_twi_t * p_app_twi, sensors_callback_t callback);

extern void sensors_measure(void);

extern void sensors_timers_init(void);

extern void sensors_timers_start(void);

extern void sensors_timers_stop(void);

#endif // SENSORS_H__
