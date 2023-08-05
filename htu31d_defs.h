/**
* @file	htu31d_defs.h
* @date	2020-01-24
* @version	v3.4.3
*
*/

/*! @file htu31d_defs.h
 * @brief Sensor driver for HTU31D sensor
 */

/*!
 * @defgroup HTU31D SENSOR API
 * @brief
 */
#ifndef HTU31D_DEFS_H_
#define HTU31D_DEFS_H_

/********************************************************/
/* header includes */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/********************************************************/

/**\I2C addresses */
/* IC_ADD Pin level GND */
#define HTU31D_I2C_ADDR_PRIM              (uint8_t)(0x40)
/* IC_ADD Pin level VDD */
#define HTU31D_I2C_ADDR_SECD              (uint8_t)(0x41)

/**\ Command Byte: Conversion */

/* Start Conversion */
#define HTU31D_CMD_CONV_START             (uint8_t)(0x40)
/* Humidity OSR = 3 (0.007%RH) */
#define HTU31D_CMD_CONV_OSRRH_3           (uint8_t)(0x18)
/* Humidity OSR = 2 (0.010%RH) */
#define HTU31D_CMD_CONV_OSRRH_2           (uint8_t)(0x10)
/* Humidity OSR = 1 (0.014%RH) */
#define HTU31D_CMD_CONV_OSRRH_1           (uint8_t)(0x08)
/* Humidity OSR = 0 (0.020%RH) */
#define HTU31D_CMD_CONV_OSRRH_0           (uint8_t)(0x00)
/* Temperature OSR = 3 (0.012°C) */
#define HTU31D_CMD_CONV_OSRT_3            (uint8_t)(0x06)
/* Temperature OSR = 2 (0.016°C) */
#define HTU31D_CMD_CONV_OSRT_2            (uint8_t)(0x04)
/* Temperature OSR = 1 (0.025°C) */
#define HTU31D_CMD_CONV_OSRT_1            (uint8_t)(0x02)
/* Temperature OSR = 0 (0.040°C) */
#define HTU31D_CMD_CONV_OSRT_0            (uint8_t)(0x00)

/**\ Command Byte: Read */

/* Read T & RH */
#define HTU31D_CMD_READ_T_RH              (uint8_t)(0x00)
/* Length of T & RH data */
#define HTU31D_READ_T_RH_DATA_LEN         HTU31D_READ_T_DATA_LEN + HTU31D_READ_RH_DATA_LEN
/* Length of T data */
#define HTU31D_READ_T_DATA_LEN            (uint8_t)(3)
/* Read RH */
#define HTU31D_CMD_READ_RH                (uint8_t)(0x10)
/* Length of RH data */
#define HTU31D_READ_RH_DATA_LEN           (uint8_t)(3)
/* Read serial number */
#define HTU31D_CMD_READ_SN                (uint8_t)(0x0A)
/* Length of SN data */
#define HTU31D_READ_SN_DATA_LEN           (uint8_t)(4)

/**\ Command Byte: Read Diagnostic */

/* Read Diagnostic */
#define HTU31D_CMD_READ_DIAG              (uint8_t)(0x08)
/* Length of Dagnostic data */
#define HTU31D_READ_DIAG_DATA_LEN         (uint8_t)(3)
/*
* NVM error (NVM err)
* bit is set if theCRC of the NVM has failed
* (CRC is done on the register copy)
*/
#define HTU31D_DIAG_NVM_MASK              (uint8_t)(0x80)
/*
* Humidity under/overrun (Hout)
* bit is set if the humidity is truncated to 0 or the max value of 2^16-1
*/
#define HTU31D_DIAG_HOUT_MASK             (uint8_t)(0x40)
/*
* Humidity high error (Hhigh)
* bit is set if the humidity calculation results above 120 %RH
*/
#define HTU31D_DIAG_HHIGH_MASK            (uint8_t)(0x20)
/*
* Humidity low error (Hlow)
* bit is set if the humidity calculation results below -10 %RH
*/
#define HTU31D_DIAG_HLOW_MASK             (uint8_t)(0x10)
/*
* Temperature under/overrun (Tout)
* bit is set if the temperature is truncated to 0 or the max value of 2^16-1
*/
#define HTU31D_DIAG_TOUT_MASK             (uint8_t)(0x08)
/*
* Temperature high error (Thigh)
* bit is set if the temperature calculation results above 150 °C
*/
#define HTU31D_DIAG_THIGH_MASK             (uint8_t)(0x04)
/*
* Temperature low error (Tlow)
* bit is set if the temperature calculation results below -50 °C
*/
#define HTU31D_DIAG_TLOW_MASK             (uint8_t)(0x02)
/*
* Heater on (Hon)
* bit is set if the heater is on
*/
#define HTU31D_DIAG_HON_MASK             (uint8_t)(0x01)

/**\ Command Byte: Reset */
/* Reset */
#define HTU31D_CMD_RESET                  (uint8_t)(0x0E)
/* Max delay after Reset. In ms */
#define HTU31D_RESET_DELAY                (uint8_t)(5)

/**\ Command Byte: Heater */
/* Turn heater on */
#define HTU31D_CMD_HEATER_ON              (uint8_t)(0x04)
/* Turn heater off */
#define HTU31D_CMD_HEATER_OFF             (uint8_t)(0x02)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define HTU31D_CONCAT_BYTES(msb, lsb)            (((uint16_t)msb << 8) | (uint16_t)lsb)

#define HTU31D_SET_BITS(reg_data, bit_mask, data) \
    ((reg_data & ~(bit_mask)) | (data & bit_mask))

#define HTU31D_GET_BITS(reg_data, bitn_mask)       (reg_data & bit_mask)

/*!
 * @brief Type definitions
 */
typedef void (*htu31d_read_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*htu31d_write_fptr_t)(uint8_t dev_id, uint8_t reg_addr);
typedef void (*htu31d_delay_fptr_t)(uint32_t period);


struct htu31d_ucom_data
{
    /*! Compensated temperature */
    uint32_t temperature;

    /*! Compensated humidity */
    uint32_t humidity;
};

struct htu31d_data
{
    /*! Compensated temperature */
    float temperature;

    /*! Compensated humidity */
    float humidity;
};

struct htu31d_settings
{
    /* HTU31D_CMD_CONV_OSRRH_* */
    uint8_t osrrh;

    /* HTU31D_CMD_CONV_OSRT_* */
    uint8_t osrt;
};

/*!
 * @brief htu31d device structure
 */
struct htu31d_dev
{
    /* HTU31D_I2C_ADDR_* */
    uint8_t device_addr;

    /*! Read function pointer */
    htu31d_read_fptr_t read;

    /*! Write function pointer */
    htu31d_write_fptr_t write;

    /*! Delay function pointer */
    htu31d_delay_fptr_t delay_ms;

    uint32_t serial_number;

    struct htu31d_ucom_data uncom_data;

    struct htu31d_data data;

    struct htu31d_settings settings;
};

#endif /* HTU31D_DEFS_H_ */
/** @}*/
/** @}*/
