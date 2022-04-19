/*
 *
 * ADXL345 Accelerometer I2C Driver
 *
 * Author: Cristian Sulighetean
 * Created: 15 November 2021
 *
 * Version 1.0 - First release
 */

#ifndef ADXL345_I2C_DRIVER_H
#define ADXL345_I2C_DRIVER_H

// Include MCU specific HAL library
#include "stm32f4xx_hal.h"

/* ===================================================================
 * I2C DEVICE ADDRESS
 * ===================================================================*/

/**
 * @brief
 * The address is  7bit and we have to shift it 1 bit to the left 
 * in order to account for  the read/write bit
 * 
 * The address can be 0x1D if ALT_ADDRESS pin = 1 (same as the SDO pin)
 * or 0x53 if ALT_ADDRESS pin = 0 (p.18)
 */
#define ADXL345_I2C_ADDR	(0x53 << 1)

/* Device ID */
#define ADXL345_DEVID		0xE5

/*===================================================================
 * REGISTERS (p.24)
 * ===================================================================*/

#define ADXL345_REG_DEVID			0x00
#define ADXL345_REG_OFSX			0x1E
#define ADXL345_REG_OFSY			0x1F
#define ADXL345_REG_OFSZ			0x20
#define ADXL345_REG_DUR				0x21
#define ADXL345_REG_LATENT			0x22
#define ADXL345_REG_WINDOW			0x23
#define ADXL345_REG_BW_RATE			0x2C
#define ADXL345_REG_POWER_CTL		0x2D
#define ADXL345_REG_INT_ENABLE		0x2E
#define ADXL345_REG_INT_MAP			0x2F
#define ADXL345_REG_INT_SOURCE		0x30
#define ADXL345_REG_DATA_FORMAT		0x31
#define ADXL345_REG_DATAX0			0x32
#define ADXL345_REG_DATAX1			0x33
#define ADXL345_REG_DATAY0			0x34
#define ADXL345_REG_DATAY1			0x35
#define ADXL345_REG_DATAZ0			0x36
#define ADXL345_REG_DATAZ1			0x37
#define ADXL345_REG_FIFO_CTL		0x38
#define ADXL345_REG_FIFO_STATUS		0x39

/*===================================================================
* CONVERSION CONSTANT
===================================================================*/
#define ADXL345_MG2G_MULTIPLIER 0.04f

/**
 * @brief Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth
 * 
 * Not used at the moment
 */

typedef enum
{
  ADXL345_DATARATE_3200_HZ = 0b1111, ///< 1600Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_1600_HZ = 0b1110, ///<  800Hz Bandwidth    90uA IDD
  ADXL345_DATARATE_800_HZ = 0b1101,  ///<  400Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_400_HZ = 0b1100,  ///<  200Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_200_HZ = 0b1011,  ///<  100Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_100_HZ = 0b1010,  ///<   50Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_50_HZ = 0b1001,   ///<   25Hz Bandwidth    90uA IDD
  ADXL345_DATARATE_25_HZ = 0b1000,   ///< 12.5Hz Bandwidth    60uA IDD
  ADXL345_DATARATE_12_5_HZ = 0b0111, ///< 6.25Hz Bandwidth    50uA IDD
  ADXL345_DATARATE_6_25HZ = 0b0110,  ///< 3.13Hz Bandwidth    45uA IDD
  ADXL345_DATARATE_3_13_HZ = 0b0101, ///< 1.56Hz Bandwidth    40uA IDD
  ADXL345_DATARATE_1_56_HZ = 0b0100, ///< 0.78Hz Bandwidth    34uA IDD
  ADXL345_DATARATE_0_78_HZ = 0b0011, ///< 0.39Hz Bandwidth    23uA IDD
  ADXL345_DATARATE_0_39_HZ = 0b0010, ///< 0.20Hz Bandwidth    23uA IDD
  ADXL345_DATARATE_0_20_HZ = 0b0001, ///< 0.10Hz Bandwidth    23uA IDD
  ADXL345_DATARATE_0_10_HZ = 0b0000  ///< 0.05Hz Bandwidth    23uA IDD (default value)
} dataRate_t;


/**
 * @brief  Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range
 *
 * Not used at the moment
 */
typedef enum {
  ADXL345_RANGE_16_G = 0b11, ///< +/- 16g
  ADXL345_RANGE_8_G = 0b10,  ///< +/- 8g
  ADXL345_RANGE_4_G = 0b01,  ///< +/- 4g
  ADXL345_RANGE_2_G = 0b00   ///< +/- 2g (default value)
} range_t;

/*
 * SENSOR STRUCT
 */
typedef struct {

	/* I2C Handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Acceleration data (X, Y, Z) in m/s^2 */
	float acc_mps2[3];

}ADXL345;

/*
 * INITIALISATION
 */
uint8_t ADXL345_Initialise(ADXL345 *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * DATA AQUISITION
 */
HAL_StatusTypeDef ADXL345_ReadAcceleration(ADXL345 *dev);

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345 *dev, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t len);

HAL_StatusTypeDef ADXL345_WriteRegister(ADXL345 *dev, uint8_t reg, uint8_t *data);

#endif
