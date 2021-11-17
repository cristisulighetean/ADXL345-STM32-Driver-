/*
 *
 * ADXL345 Accelerometer I2C Driver
 *
 * Author: Cristian Sulighetean
 * Created: 15 November 2021
 *
 */

/* Include in case we have already added the driver
 * files somewhere else
 */
#ifndef ADXL345_I2C_DRIVER_H
#define ADXL345_I2C_DRIVER_H


// Include HAL library for I2C functions
#include "stm32f4xx_hal.h"

/*
 * DEFINES
 */

/* I2C DEVICE ADDRESS
 * The address is  7bit and we have to shift it
 * 1 bit to the left in order to account for
 * the read/write bit
 */
#define ADXL345_I2C_ADDR	(0x1D << 1)
/*
 * The address can be 0x1D if ALT_ADDRESS pin = 1
 * or 0x53 if ALT_ADDRESS pin = 0 (p.18)
 */

// Device ID
#define ADXL345_DEVID		0xE5

/*
 * REGISTERS (p.24)
 */
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
