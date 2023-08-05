/**
* @file	htu31d.c
* @date	2020-01-24
* @version	v3.4.3
*
*/

/*! @file htu31d.c
 * @brief Sensor driver for HTU31D sensor
 */
#include "htu31d.h"
#include "math.h"

void htu31d_set_reg(uint8_t reg_cmd, const struct htu31d_dev *dev);
void htu31d_get_regs(uint8_t reg_cmd, uint8_t *reg_data, uint16_t len, const struct htu31d_dev *dev);
void htu31d_compensate_temperature(struct htu31d_dev *dev);
void htu31d_compensate_humidity(struct htu31d_dev *dev);


void htu31d_fill_default_dev(struct htu31d_dev *dev)
{
	struct htu31d_data htu31d_data = {0};
	struct htu31d_ucom_data htu31d_ucom_data = {0};
	struct htu31d_settings htu31d_settings;
	htu31d_settings.osrrh = HTU31D_CMD_CONV_OSRRH_3;
	htu31d_settings.osrt = HTU31D_CMD_CONV_OSRT_3;

	struct htu31d_dev htu31_dev;
	htu31_dev.data = htu31d_data;
	htu31_dev.settings = htu31d_settings;
	htu31_dev.uncom_data = htu31d_ucom_data;
	htu31_dev.device_addr = HTU31D_I2C_ADDR_PRIM;
}
/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
void htu31d_get_regs(uint8_t reg_cmd, uint8_t *reg_data, uint16_t len, const struct htu31d_dev *dev)
{
    /* Read the data  */
    dev->read(dev->device_addr, reg_cmd, reg_data, len);
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
void htu31d_set_reg(uint8_t reg_cmd, const struct htu31d_dev *dev)
{
    dev->write(dev->device_addr, reg_cmd);
}

void htu31d_start_conversion(struct htu31d_dev *dev)
{
    htu31d_set_reg(HTU31D_CMD_CONV_START | dev->settings.osrrh | dev->settings.osrt, dev);
}

void htu31d_compensate_temperature(struct htu31d_dev *dev)
{
    uint16_t st = dev->uncom_data.temperature;
    dev->data.temperature = -40 + 165 * (st / (65536 - 1.0f));
}

void htu31d_read_temperature(struct htu31d_dev *dev)
{
    uint8_t data[HTU31D_READ_T_DATA_LEN] = {0};
    htu31d_get_regs(HTU31D_CMD_READ_T_RH, data, HTU31D_READ_T_DATA_LEN, dev);

    uint16_t st = HTU31D_CONCAT_BYTES(data[0], data[1]);
    dev->uncom_data.temperature = st;
    htu31d_compensate_temperature(dev);
}


void htu31d_compensate_humidity(struct htu31d_dev *dev)
{
    uint16_t srh = dev->uncom_data.humidity;
    dev->data.humidity = 100 * (srh / (65536 - 1.0f));
}

void htu31d_read_humidity(struct htu31d_dev *dev)
{
    uint8_t data[HTU31D_READ_RH_DATA_LEN] = {0};
    htu31d_get_regs(HTU31D_CMD_READ_RH, data, HTU31D_READ_RH_DATA_LEN, dev);

    uint16_t srh = HTU31D_CONCAT_BYTES(data[0], data[1]);
    dev->uncom_data.humidity = srh;
    htu31d_compensate_humidity(dev);
}

void htu31d_read_temperature_humidity(struct htu31d_dev *dev)
{
    uint8_t data[HTU31D_READ_T_RH_DATA_LEN] = {0};
    htu31d_get_regs(HTU31D_CMD_READ_T_RH, data, HTU31D_READ_T_RH_DATA_LEN, dev);

    uint16_t st = HTU31D_CONCAT_BYTES(data[0], data[1]);
    dev->uncom_data.temperature = st;
    htu31d_compensate_temperature(dev);

    uint16_t srh = HTU31D_CONCAT_BYTES(data[HTU31D_READ_T_DATA_LEN], data[HTU31D_READ_T_DATA_LEN + 1]);
    dev->uncom_data.humidity = srh;
    htu31d_compensate_humidity(dev);
}

uint32_t htu31d_cal_meas_delay_temperature(struct htu31d_dev *dev)
{
    uint32_t max_delay;
    uint8_t osrt = dev->settings.osrt;
	
	if (osrt == HTU31D_CMD_CONV_OSRT_0)
		max_delay = 2;
	else if (osrt == HTU31D_CMD_CONV_OSRT_1)
		max_delay = 4;
	else if (osrt == HTU31D_CMD_CONV_OSRT_2)
		max_delay = 7;
	else
		max_delay = 13;

    return max_delay;
}

uint32_t htu31d_cal_meas_delay_humidity(struct htu31d_dev *dev)
{
    uint32_t max_delay;
    uint8_t osrrh = dev->settings.osrrh;
	
	if (osrrh == HTU31D_CMD_CONV_OSRRH_0)
		max_delay = 1;
	else if (osrrh == HTU31D_CMD_CONV_OSRRH_1)
		max_delay = 2;
	else if (osrrh == HTU31D_CMD_CONV_OSRRH_2)
		max_delay = 4;
	else
		max_delay = 8;

    return max_delay;
}

void htu31d_start_and_read_temperature(struct htu31d_dev *dev)
{
    htu31d_set_reg(HTU31D_CMD_CONV_START | dev->settings.osrt, dev);
    uint32_t delay = htu31d_cal_meas_delay_temperature(dev);
    dev->delay_ms(delay);
    htu31d_read_temperature(dev);
}

void htu31d_start_and_read_humidity(struct htu31d_dev *dev)
{
    htu31d_set_reg(HTU31D_CMD_CONV_START | dev->settings.osrrh, dev);
    uint32_t delay = htu31d_cal_meas_delay_humidity(dev);
    dev->delay_ms(delay);
    htu31d_read_humidity(dev);
}
void htu31d_start_and_read_temperature_humidity(struct htu31d_dev *dev)
{
    htu31d_start_conversion(dev);
    uint32_t delay_t = htu31d_cal_meas_delay_temperature(dev);
    uint32_t delay_h = htu31d_cal_meas_delay_humidity(dev);
    //dev->delay_ms(delay_t > delay_h ? delay_t : delay_h);
    dev->delay_ms(delay_t + delay_h);
    htu31d_read_temperature_humidity(dev);
}

void htu31d_soft_reset(struct htu31d_dev *dev)
{
    htu31d_set_reg(HTU31D_CMD_RESET, dev);
    dev->delay_ms(HTU31D_RESET_DELAY);
}

void htu31d_turn_heater_on(struct htu31d_dev *dev)
{
    htu31d_set_reg(HTU31D_CMD_HEATER_ON, dev);
}

void htu31d_turn_heater_off(struct htu31d_dev *dev)
{
    htu31d_set_reg(HTU31D_CMD_HEATER_OFF, dev);
}

uint8_t htu31d_read_diagnostic(struct htu31d_dev *dev)
{
    uint8_t data = 0;
    htu31d_get_regs(HTU31D_CMD_READ_DIAG, &data, 1, dev);
    return data;
}

uint8_t htu31d_read_heater_status(struct htu31d_dev *dev)
{
    uint8_t data = htu31d_read_diagnostic(dev);
    return data & HTU31D_DIAG_HON_MASK;
}

void htu31d_read_serial_number(struct htu31d_dev *dev)
{
    uint8_t data[HTU31D_READ_SN_DATA_LEN] = {0};
    htu31d_get_regs(HTU31D_CMD_READ_SN, data, HTU31D_READ_SN_DATA_LEN, dev);
    dev->serial_number = (data[0] << 16) | (data[1] << 8) | data[2];
}
