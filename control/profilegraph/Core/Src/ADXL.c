/**
* @Library for ADXL345 3-axis accelometer 
* @Hardware dependencies: Could be changed very easily.
						STM32L152R uC
						SPI2 
						Some GPIOs
* @Author Iman Hosseinzadeh iman[dot]hosseinzadeh AT gmail
  https://github.com/ImanHz

  @Revised for ADXL355 by Lin Fu
*/
 /**
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/




#include "ADXL.h"
#include "stm32f7xx_hal.h"


	float GAINX = 0.0f;
	float GAINY = 0.0f;
	float GAINZ = 0.0f;
	
/**
 * @brief Writing ADXL Registers.
 * @param address 8-bit address of register
 * @param value Data to be sent
 * @param num Number of bytes to be written
 * @retval none
 * @note Since the register values to be written are 8-bit, there is no need to multiple writing
 */
static void writeRegister(uint8_t address,uint8_t *value, uint8_t num)
{
		if (address > 63)
		address = 63;
	
	// Setting R/W = 0, i.e.: Write Mode
		address = (address << 1 & ~0x01);

	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPIhandler,&address,1,10);
	HAL_SPI_Transmit(&SPIhandler,value,num,10);
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	

}


/** @brief Reading ADXL Registers.
 * 	@param address 8-bit address of register
 *	@param value Pointer to buffer that stores the read value
 *	@param num Number of bytes to be written
 * 	@retval none
 */
static void readRegister(uint8_t address,uint8_t *value, uint8_t num)
{
		if (address > 63)
		address = 63;
		
		// Setting R/W = 1, i.e.: Read Mode
		address = (address << 1 | 0x01);
		
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPIhandler,&address,1,10);
	HAL_SPI_Receive(&SPIhandler,value,num,10);
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	
	
}
	
/**
	Data Format Settings
	DATA_FORMAT[7-0] = SELF_TEST  SPI  INT_INVERT  0  FULL_RES  Justify  Range[2]
	
	SPI bit: 			0 = 4-wire (Default) 
								1 = 3-wire
	INT_Invert:   		0 = Active High (Default) 
								1 = Active Low
	Full Res:			0 = 10-bit (Default) 
								1 = Full Resolution
	Justify:			0 = Signed (Default) 
								1 = MSB
	Range:
								
					 

				Range value 	|  Output Data Rate (Hz)
				---------------------------------
						0 		|  				+-2g	// Default
						1 		|  				+-4g
						2 		|  				+-8g		
						3 		|  				+-16g
	 									
		*/

static void adxlFormat(ADXL_InitTypeDef * adxl) {
	uint8_t formatreg=0;
	writeRegister(RANGE,&formatreg,1); //Set register to 0
	formatreg = (adxl->IntMode << 6 | adxl->Range) & RANGE_REG_MASK;
	writeRegister(RANGE,&formatreg,1);

	formatreg = 0;
	writeRegister(POWER_CTL, &formatreg,1);
	formatreg = (adxl->DataReadyMode << 2 | adxl->TempMode << 1 | adxl->StandbyMode) & POWER_CTL_REG_MASK;
	writeRegister(POWER_CTL, &formatreg, 1);
}

/**
 * @brief Call to reset ADXL355
 */
static void adxlReset(void) {
    uint8_t shadow_reg_store[5] = {};
    uint8_t shadow_reg_read[5] = {};
    uint8_t reset_write = ADXL_RESET_WRITE;
    //Store values from shadow registers
    readRegister(SHADOW_REG1, shadow_reg_store, 5);
    while (1) {
        //Perform a software reset
        writeRegister(ADXL_RESET, &reset_write, 1);
        //Read shadow registers after reset
        readRegister(SHADOW_REG1, shadow_reg_read, 5);
        // Check if shadow registers match the stored values
        uint8_t match = 1;
        for (int i = 0; i < 5; i++) {
            if (shadow_reg_read[i] != shadow_reg_store[i]) {
                match = 0;
                break;
            }
        }
        // If all shadow registers match, exit the loop
        if (match) break;
    }
}

// Public Functions

// Initializes the ADXL unit
adxlStatus ADXL_Init(ADXL_InitTypeDef * adxl)
{
	// CS is active low. Here we deselect the chip. In each function the CS signal is asserted individually
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	// Unknown delay should apply
	HAL_Delay(5);
	//Reset device
	adxlReset();
	uint8_t testval[2] = {};
	// The Device Address register is constant, i.e. = 0xAD
	readRegister(DEVID_AD, testval, 2);
	if (testval[0] != DEVID_AD_DEFAULT_VAL || testval[1] != DEVID_MST_DEFAULT_VAL) return ADXL_ERR;

	//Init of BW_RATE and DATAFORMAT registers
	adxlFormat(adxl);
	
	// Settings gains 
	switch (adxl->Range) {
		case ADXL_RANGE_2G:
			GAINX = GAINY = GAINZ = ADXL355_ACC_SENS_2G;
			break;
		case ADXL_RANGE_4G:
			GAINX = GAINY = GAINZ = ADXL355_ACC_SENS_4G;
			break;
		case ADXL_RANGE_8G:
			GAINX = GAINY = GAINZ = ADXL355_ACC_SENS_8G;
			break;
		default:
			GAINX = GAINY = GAINZ = ADXL355_ACC_SENS_2G;
}
			
	return ADXL_OK;
	
}


/**
 * @brief Read accelerometer data in all 3 axis
 * @param[out] Data	pointer to buffer that stores the 3 axis acceleration value
 * @retval none
 */
void ADXL_getAccelRaw(void *Data)
{
    uint8_t data[9] = {};
    readRegister(XDATA3, data, 9);

	int32_t *acc = Data;

	// Two's Complement 20-bit conversion
	acc[0] = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
	acc[1] = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
	acc[2] = (int32_t)((data[6] << 12) | (data[7] << 4) | (data[8] >> 4));

	// Sign extension for 20-bit values
	if (acc[0] & 0x80000) acc[0] |= 0xFFF00000;
	if (acc[1] & 0x80000) acc[1] |= 0xFFF00000;
	if (acc[2] & 0x80000) acc[2] |= 0xFFF00000;
}

/**
 * @brief Read accelerometer data and convert it to float
 * @param pData Pointer to buffer that stores acceleration value
 * @retval none
 */
void ADXL_getAccelFloat(void *pData) {
	int32_t rawdata[3] = {};
	float *dataout = pData;
	ADXL_getAccelRaw(rawdata);

	//Convert raw data to float
	dataout[0] = (float)rawdata[0] * GAINX;
	dataout[1] = (float)rawdata[1] * GAINY;
	dataout[2] = (float)rawdata[2] * GAINZ;
}

/**
 *  @brief Set offset on ADXL355 output data
 *  @param offset_x
 *  @param offset_y
 *  @param offset_z
 *  @retval none
 */
void ADXL_setOffset(uint16_t offset_x, uint16_t offset_y, uint16_t offset_z) {
	uint8_t set_offset[6] = {};
	//Set offset reg to 0
	writeRegister(OFFSET_X_H, set_offset, 6);
	//Set offset
	set_offset[0] = (offset_x >> 8) & 0xFF;
	set_offset[1] = offset_x & 0xFF;
	set_offset[2] = (offset_y >> 8) & 0xFF;
	set_offset[3] = offset_y & 0xFF;
	set_offset[4] = (offset_z >> 8) & 0xFF;
	set_offset[5] = offset_z & 0xFF;
	//Write user defined offset into reg
	writeRegister(OFFSET_X_H, set_offset, 6);
}
	
/** Starts Measure Mode
* @param: s = ON or OFF				

*/
void ADXL_Measure(Switch s)
		{
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			switch (s) {
				case ON:
				reg &= ~(1<<2);
				reg |= (1<<3);
				writeRegister(POWER_CTL,&reg,1);
				break;
				case OFF:
				reg &= ~(1<<3);
				writeRegister(POWER_CTL,&reg,1);
				break;				
				}
		}

void ADXL_setFilter(void) {
	uint8_t reg;
	readRegister(FILTER, &reg, 1);
	reg = 0b00001010;
	writeRegister(FILTER, &reg, 1);
}
//
///** Starts Sleep Mode
//* @param: s 		=  ON or OFF
//* @param: rate  =  SLEEP_RATE_1HZ
//									 SLEEP_RATE_2HZ
//									 SLEEP_RATE_4HZ
//									 SLEEP_RATE_8HZ
//*/
//void ADXL_Sleep(Switch s,uint8_t rate)
//		{
//			uint8_t reg;
//			readRegister(POWER_CTL,&reg,1);
//			switch (s) {
//				case ON:
//				reg |= (1<<2);
//				reg &= ~(1<<3);
//				reg += rate;
//				writeRegister(POWER_CTL,reg,1);
//				break;
//				case OFF:
//				reg &= ~(1<<2);
//				writeRegister(POWER_CTL,reg,1);
//				break;
//				}
//
//		}
//
///** Starts Standby Mode
//* @param: s = ON or OFF
//		OFF: Takes the module into sleep mode.
//
//*/
//void ADXL_Standby(Switch s)
//		{
//			uint8_t reg;
//			readRegister(POWER_CTL,&reg,1);
//			switch (s) {
//				case ON:
//				reg &= ~(1<<2);
//				reg &= ~(1<<3);
//				writeRegister(POWER_CTL,reg);
//				break;
//				case OFF:
//				reg |= (1<<2);
//				writeRegister(POWER_CTL,reg);
//				break;
//				}
//
//		}
//
//
//
///** Reading Main Registers
//regs[0] = BW_RATE
//regs[1] = DATA_FORMAT
//regs[2] = POWER_CTL
//*/
//void ADXL_test(uint8_t * regs)
//		{
//			readRegister(BW_RATE,&regs[0],1);
//			readRegister(DATA_FORMAT,&regs[1],1);
//			readRegister(POWER_CTL,&regs[2],1);
//
//		}
//
//		 /**
// Enables the self Test mode
// */
//void ADXL_enableSelfTest(void)
//			{
//			uint8_t formatreg=0;
//			writeRegister(DATA_FORMAT,formatreg);
//			formatreg |= (1<<7);
//			writeRegister(DATA_FORMAT,formatreg);
//			}
//
//
// /**
// Disables the self Test mode
// */
//void ADXL_disableSelfTest(void)
//			{
//			uint8_t formatreg=0;
//			writeRegister(DATA_FORMAT,formatreg);
//			formatreg &= ~(1<<7);
//			writeRegister(DATA_FORMAT,formatreg);
//				}
//
//
///**
// Setting the offsets for calibration
//* @param 	user-set offset adjustments in twos complement format
//					with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g).
//
//*/
//void ADXL_SetOffset(int8_t off_x,int8_t off_y,int8_t off_z)
//			{
//			writeRegister(OFFX,(uint8_t) off_x );
//			writeRegister(OFFY,(uint8_t) off_y );
//			writeRegister(OFFZ,(uint8_t) off_z );
//			}
//
//
//
//
//				//////////////////////////////////////////// I N T E R R U P T S //////////////////////
//
///** Setting TAP Int.
//* @param out : ADXL has two Int. pins.
//* @param axes: The axes of tap. Could be OR'ed.
//* @param Duration: The minimum duration for tap detection. The scale factor is 625 us/LSB. Should not be 0!
//* @param Threshold: The threshold value for tap interrupt. The scale factor is 62.5 mg/LSB. Should not be 0!
//*/
//
//void ADXL_enableSingleTap(ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold)
//
//{
//	uint8_t reg=0;
//
//	writeRegister(DUR,Duration);
//	writeRegister(THRESH_TAP,Threshold);
//
//	//Setting the Axes
//	readRegister(TAP_AXES,&reg,1);
//	reg |= axes;
//
//	writeRegister(TAP_AXES,reg);
//
//	// Settings Int output
//	readRegister(INT_MAP,&reg,1);
//	if (out == INT1) reg &= ~(1<<6); else reg |= (1<<6);
//	writeRegister(INT_MAP,reg);
//
//	// Enabling the TAP interrupt
//	readRegister(INT_ENABLE,&reg,1);
//	reg |= (1<<6);
//	writeRegister(INT_ENABLE,reg);
//
//}
//
///** Disabling TAP Int.
//
// The settings are preserved.
//
//*/
//
//void ADXL_disableSingleTap(void)
//
//{
//	uint8_t reg=0;
//	// Disabling the TAP interrupt
//	readRegister(INT_ENABLE,&reg,1);
//	reg &= ~(1<<6);
//	writeRegister(INT_ENABLE,reg);
//
//
//}
//
///**  Enabling Double TAP Int.
//* @param out : 			ADXL has two Int. pins.
//* @param axes: 			The axes of tap. Could be OR'ed.
//* @param Duration: 	The minimum duration for tap detection. The scale factor is 625 us/LSB. Should not be 0!
//* @param Threshold: The threshold value for tap interrupt. The scale factor is 62.5 mg/LSB. Should not be 0!
//* @param Latent: 		The delay time after the first Tap. Scale factor is : 1.25 ms/LSB. Should not be 0!
//* @param Windows:		The time interval between two Taps. Scale factor is : 1.25 ms/LSB.  Should not be 0!
//*/
//
//void ADXL_enableDoubleTap(ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold, uint8_t Latent, uint8_t Window)
//		{
//		uint8_t reg=0;
//
//	writeRegister(DUR,Duration);
//	writeRegister(THRESH_TAP,Threshold);
//	writeRegister(LATENT,Latent);
//	writeRegister(WINDOW,Window);
//
//
//	//Setting the Axes
//	readRegister(TAP_AXES,&reg,1);
//	reg += axes;
//	writeRegister(TAP_AXES,reg);
//
//	// Settings Int output
//	readRegister(INT_MAP,&reg,1);
//	if (out == INT1) reg &= ~(1<<5); else reg |= (1<<5);
//	writeRegister(INT_MAP,reg);
//
//	// Enabling the TAP interrupt
//	readRegister(INT_ENABLE,&reg,1);
//	reg |= (1<<5);
//	writeRegister(INT_ENABLE,reg);
//	 }
//
//
///** Disabling Double TAP Int.
//
// The settings are preserved.
//
//*/
//
//void ADXL_disableDoubleTap(void)
//
//  {
//	uint8_t reg=0;
//	// Disabling the Double TAP interrupt
//	readRegister(INT_ENABLE,&reg,1);
//	reg &= ~(1<<5);
//	writeRegister(INT_ENABLE,reg);
//	}
//
///**  Enabling Activity Int.
//* @param out : 			ADXL has two Int. pins.
//* @param axes: 			The axes of activity. Could be OR'ed.
//* @param Threshold: The threshold value for activity interrupt. The scale factor is 62.5 mg/LSB. Should not be 0!
//*/
//
//void ADXL_enableActivity(ADXL_IntOutput out, uint8_t axes, uint8_t Threshold, uint8_t AcDc)
//
//		{
//			uint8_t reg=0;
//
//	writeRegister(THRESH_ACT,Threshold);
//
//
//	//Setting the Axes
//	readRegister(ACT_INACT_CTL,&reg,1);
//	reg += (axes << 4);
//	if (AcDc == ACTIVITY_AC) reg |= (1<<7); else reg &= ~(1<<7);
//			writeRegister(TAP_AXES,reg);
//
//	// Settings Int output
//	readRegister(INT_MAP,&reg,1);
//	if (out == INT1) reg &= ~(1<<4); else reg |= (1<<4);
//	writeRegister(INT_MAP,reg);
//
//	// Enabling the TAP interrupt
//	readRegister(INT_ENABLE,&reg,1);
//	reg |= (1<<4);
//	writeRegister(INT_ENABLE,reg);
//
//
//		}
//
//
///** Disabling Double TAP Int.
//
// The settings are preserved.
//
//*/
//
//void ADXL_disableActivity(void)
//  {
//	uint8_t reg=0;
//	// Disabling the Double TAP interrupt
//	readRegister(INT_ENABLE,&reg,1);
//	reg &= ~(1<<4);
//	writeRegister(INT_ENABLE,reg);
//	}
//
///**  Enables FreeFall Int.
//* @param out : 			ADXL has two Int. pins.
//* @param time: 			Representing the minimum time that the RSS value of all axes
//										must be less than Threshold to generate a free-fall interrupt.
//										A value of 0 may result in undesirable
//										behavior if the free-fall interrupt is enabled. Values between 100 ms
//										and 350 ms (0x14 to 0x46) are recommended
//* @param Threshold: The root-sumsquare (RSS) value of all axes is calculated and compared with
//										the value in Threshold to determine if a free-fall event occurred.
//										The scale factor is 62.5 mg/LSB. Note that a value of 0 mg may
//										result in undesirable behavior if the free-fall interrupt is enabled.
//										Values between 300 mg and 600 mg (0x05 to 0x09) are
//										recommended.
//					*/
//
//void ADXL_enableFreeFall(ADXL_IntOutput out, uint8_t Threshold, uint8_t Time)
//			{
//			uint8_t reg=0;
//			writeRegister(TIME_FF,Time);
//			writeRegister(THRESH_FF,Threshold);
//			// Settings Int output
//			readRegister(INT_MAP,&reg,1);
//			if (out == INT1) reg &= ~(1<<2); else reg |= (1<<2);
//			writeRegister(INT_MAP,reg);
//
//			readRegister(INT_ENABLE,&reg,1);
//			reg |= (1<<2);
//			writeRegister(INT_ENABLE,reg);
//			}
//
//
///** Disabling Double TAP Int.
//
// The settings are preserved.
//
//*/
//
//void ADXL_disableFreeFall(void)
//{
//			uint8_t reg=0;
//			readRegister(INT_ENABLE,&reg,1);
//			reg %= ~(1<<2);
//			writeRegister(INT_ENABLE,reg);
//
//}
//
///** Interrupt prototype
//* @brief In order to interrupt flags being reset, the address 0x30 should be read.
//* Put this function wherever you want to implement interrupt routines, e.g. EXTI_Callback
//*/
//
//void ADXL_IntProto(void)
//
//{
//			uint8_t reg=0;
//			readRegister(INT_SOURCE,&reg,1);
//
//}

