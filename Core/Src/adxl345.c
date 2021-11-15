/*
 * adxl345.c
 *
 *  Created on: Nov 15, 2021
 *      Author: cristian
 */

#include "adxl345.h"

uin8_t ADXL345_Initialise(ADXL345 *dev, I2C_HandleTypeDef *i2cHandle){

	/* Set struct parameters */
	dev->i2cHandle 			= i2cHandle;

	dev->acc_mps2[0] 		= 0.0f;
	dev->acc_mps2[1] 		= 0.0f;
	dev->acc_mps2[2] 		= 0.0f;

	dev->temp_C 			= 0.0f;

	/* Store number of transactions errors (to be returned at the end of the function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Check DEV_ID (p.25)
	 */
	uint8_t regData;
	status = ADXL345_ReadRegister(ADXL345_REG_DEVID, &regData);
	errNum += (status != HAL_OK);

	if(regData != ADXL345_DEVID){
		return 255;
	}

	/*
	 * SET power mode and rate (BW_RATE) (p.26)
	 * D4 : LOW_POWER bit = 0 normal operation & 1 low power (noisy)
	 * D0-D3 : Rate bits ODR (p.15)
	 * 		100Hz ODR = 0x0A
	 * 		200Hz ODR = 0x0B
	 * 		400HZ ODR = 0x0C
	 */
	regData = 0x0C;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_BW_RATE, &regData);
	errNum += (status != HAL_OK);

	/*
	 * SET ADXL to MEASURING MODE (POWER_CTL) (p.26)
	 *
	 * Bits Definition (p.26)
	 * D4 : Sleep Mode (Needs setup for THRESH_INACT and TIME_INACT)
	 * D3 : Measure = 1 (measuring mode on)
	 * D2 : Sleep (1 puts ADXL into sleep mode)
	 * D0-D1 : Wakeup
	 *
	 * Selected D3 = 1 - measuring mode on
	 */
	regData = 0x08;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_POWER_CTL, &regData);
	errNum += (status != HAL_OK);

	/*
	 * Turn on DATA_READY Interrupt (INT_ENABLE) (p.26)
	 * D7 : DATA_READY interrupt function (set to 1 if active)
	 */
	regData = 0x80;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_INT_ENABLE, &regData);
	errNum += (status != HAL_OK);


	/* SET on which bit to have the DATA_READY interrupt (INT_MAP)
	 * D7: If 0 -> it will be on INT1, if set on INT2
	 * I set it to be on INT1
	 */
	regData = 0x00;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_INT_MAP, &regData);
	errNum += (status != HAL_OK);




	/*
	 * SET data format (DATA_FORMAT) (p.26)
	 * D5: INT_INVERT - 0 sets the interrupts to active high; 1 - active low
	 * D3: FULL_RES mode
	 * D2: Justify Bit - TODO understand it
	 * D0-D1: Range Bits
	 * 			00 : +-2g
	 * 			01 : +-4g
	 * 			10 : +-8g
	 * 			11 : +-16g
	 */
	regData = 0x;

	/* TODO set DATA FORMAT

	/* END OF INITIALISATION */


	/* Return the number of errors (0 if successful initialization)*/
	return errNum;
}




/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t len){
	return HAL_I2C_Mem_Read(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL345_WriteRegister(ADXL345 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

