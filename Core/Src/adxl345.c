/*
 *
 * ADXL345 Accelerometer I2C Driver
 *
 * Author: Cristian Sulighetean
 * Created: 15 November 2021
 *
 * Version 1.0 - First release
 */

#include "adxl345.h"

uint8_t ADXL345_Initialise(ADXL345 *dev, I2C_HandleTypeDef *i2cHandle){

	/* Set struct parameters */
	dev->i2cHandle 			= i2cHandle;

	dev->acc_mps2[0] 		= 0.0f;
	dev->acc_mps2[1] 		= 0.0f;
	dev->acc_mps2[2] 		= 0.0f;


	/* Store number of transactions errors (to be returned at the end of the function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;


	/*
	 * Check DEV_ID (p.25)
	 */
	uint8_t regData;
	status = ADXL345_ReadRegister(dev, ADXL345_REG_DEVID, &regData);
	errNum += (status != HAL_OK);


	/* Exit if status different */
	if(regData != ADXL345_DEVID){
		return 255;
	}


	/*
	 * SET power mode and rate (BW_RATE) (p.26)
	 * D4 : LOW_POWER bit = 0 normal operation & 1 low power (noisy)
	 * D0-D3 : Rate bits ODR (p.15)
	 * 			100Hz ODR = 0xA
	 * 			200Hz ODR = 0xB
	 * 			400HZ ODR = 0xC
	 */
	regData = 0x0B;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_BW_RATE, &regData);
	errNum += (status != HAL_OK);


	/* SET on which bit to have the DATA_READY interrupt (INT_MAP)
	 * D7 (0): If 0 -> it will be on INT1, if set on INT2
	 *
	 */
	regData = 0x00;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_INT_MAP, &regData);
	errNum += (status != HAL_OK);


	/*
	 * SET data format (DATA_FORMAT) (p.26)
	 * D7: SELF_TEST 1-ON
	 * D6: SPI - a value of 1 sets the device into 3-wire SPI mode
	 * D5: INT_INVERT (0)- 0 sets the interrupts to active high; 1 - active low
	 * D4: 0
	 * D3: FULL_RES mode (1)
	 * D2: Justify Bit (1)
	 * 				0: right justified with sign extension
	 * 				1: left justified
	 * D0-D1: Range Bits (01)
	 * 			00 : +-2g
	 * 			01 : +-4g
	 * 			10 : +-8g
	 * 			11 : +-16g
	 *
	 * My selection: Full resolution and 4g (lsb on pos D5 which means >> 5)
	 */
	regData = 0b00001101;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_DATA_FORMAT, &regData);
	errNum += (status != HAL_OK);

	/*
	 * Enable LOW Power mode
	 * TODO
	 *
	 * SET Low power bit (Bit 4) in BW_RATE register (Address 0x2C)
	 */


	/*
	 * Set FIFO Modes
	 * TODO
	 */

	/*
	* Enable DATA_READY Interrupt (INT_ENABLE) (p.27)
	* D7 : DATA_READY interrupt function (set to 1 if active)
	* It is recommended that interrupts be configured before enabling their outputs
	*/
	regData = 0xFF;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_INT_ENABLE, &regData);
	errNum += (status != HAL_OK);

	/*
	* SET ADXL to MEASURING MODE (POWER_CTL) (p.26)
	*
	* Bits Definition (p.26)
	* D7 : 0
	* D6 : 0
	* D5 : Link bit
	* D4 : AUTO_SLEEP (Needs setup for THRESH_INACT and TIME_INACT)
	* D3 : Measure = 1 (measuring mode on)
	* D2 : Sleep (1 puts ADXL into sleep mode)
	* D0-D1 : Wakeup bits
	*
	* Selected D3 = 1 - measuring mode on
	*/
	regData = 0x08;
	status = ADXL345_WriteRegister(dev, ADXL345_REG_POWER_CTL, &regData);
	errNum += (status != HAL_OK);

	/* END OF INITIALISATION */


	/* Return the number of errors (0 if successful initialization)*/
	return errNum;
}


/*
 * DATA AQUISITION
 */
HAL_StatusTypeDef ADXL345_ReadAcceleration(ADXL345 *dev){

	/* Datasheet p.27 */
	/*
	 * Read raw values from acceleration (x,y,z -> 10 bits each)
	 */
	uint8_t regData[6];

	/* Read 6 registers starting with 0x32 to 0x37 */
	HAL_StatusTypeDef status = ADXL345_ReadRegisters(dev, ADXL345_REG_DATAX0, regData, 6);

	/*
	 * Combine register values to give raw (UNSIGNED) readings (10 bits each)
	 */

	int16_t accRawSigned[3];

	/* DATAX0 is the LSB and DATAX1 is the MSB (mask the last 3 bits of 2nd reg)*/
	accRawSigned[0] = ((int16_t)(((int16_t) regData[1] << 8) | ((int16_t) (regData[0] & 0xE0))) >> 5); // X-axis
	accRawSigned[1] = ((int16_t)(((int16_t) regData[3] << 8) | ((int16_t) (regData[2] & 0xE0))) >> 5); // Y-axis
	accRawSigned[2] = ((int16_t)(((int16_t) regData[5] << 8) | ((int16_t) (regData[4] & 0xE0))) >> 5); // X-axis

	/* Convert to mps^2 (given range setting of +-4g) */
	dev->acc_mps2[0] = ADXL345_MG2G_MULTIPLIER * accRawSigned[0];
	dev->acc_mps2[1] = ADXL345_MG2G_MULTIPLIER * accRawSigned[1];
	dev->acc_mps2[2] = ADXL345_MG2G_MULTIPLIER * accRawSigned[2];

	return status;
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
	return HAL_I2C_Mem_Write(dev->i2cHandle, ADXL345_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

