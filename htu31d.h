/**
* @file	htu31d.h
* @date	2020-01-24
* @version	v3.4.3
*
*/

/*! @file htu31d.h
 * @brief Sensor driver for HTU31D sensor
 */

/*!
 * @defgroup HTU31D SENSOR API
 */
#ifndef HTU31D_H_
#define HTU31D_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "htu31d_defs.h"

void htu31d_fill_default_dev(struct htu31d_dev *dev);
void htu31d_start_conversion(struct htu31d_dev *dev);
void htu31d_read_temperature(struct htu31d_dev *dev);
void htu31d_read_humidity(struct htu31d_dev *dev);
void htu31d_read_temperature_humidity(struct htu31d_dev *dev);
uint32_t htu31d_cal_meas_delay_temperature(struct htu31d_dev *dev);
uint32_t htu31d_cal_meas_delay_humidity(struct htu31d_dev *dev);
void htu31d_start_and_read_temperature(struct htu31d_dev *dev);
void htu31d_start_and_read_humidity(struct htu31d_dev *dev);
void htu31d_start_and_read_temperature_humidity(struct htu31d_dev *dev);
void htu31d_soft_reset(struct htu31d_dev *dev);
void htu31d_turn_heater_on(struct htu31d_dev *dev);
void htu31d_turn_heater_off(struct htu31d_dev *dev);
uint8_t htu31d_read_diagnostic(struct htu31d_dev *dev);
uint8_t htu31d_read_heater_status(struct htu31d_dev *dev);
void htu31d_read_serial_number(struct htu31d_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* HTU31D_H_ */
/** @}*/
