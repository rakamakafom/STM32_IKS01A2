/*
 * myheader.h
 *
 *  Created on: Nov 19, 2022
 *      Author: Damian
 */

#ifndef INC_MYHEADER_H_
#define INC_MYHEADER_H_


//Include

#include "main.h"

#include "math.h"
#include "string.h"
#include "stdio.h"


//Header function
void uart_send_string_DMA(char* string, int16_t length);
float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb);
//HTS221

#define HTS221_ADR  (0b1011111 << 1)
#define HTS221_ADR_WHO_AM_I (0x0F)
#define HTS221_CTR_REG_1 (0x20) // adres bufora konfiguracyjnego
#define HTS221_HUMIDITY_OUT_L (0x28)
#define HTS221_HUMIDITY_OUT_H (0x29)
#define HTS221_TEMP_OUT_L (0x2A)
#define HTS221_TEMP_OUT_H (0x2B)

//ADDRESS DATA INTERPOLATION
#define HTS221_H0_rH_x2 (0x30)	//unsigned
#define HTS221_H1_rH_x2 (0x31) //unsigned
#define HTS221_H0_T0_OUT (0x36) // 16 bits signed
#define HTS221_H1_T0_OUT (0x3A) // 16 bits signed


//LSM6DSL

#define LSM6DSL_ADR (0b1101011 << 1)
#define LSM6DSL_ADR_WHOAMI (0x0F)
#define LSM6DSL_ADR_CTRL1_XL (0x10)


#define LSM6DSL_OUT_TEMP_L (0x20) //0b00100000 Temperature
#define LSM6DSL_OUT_TEMP_H (0x21) //0b00100001
#define LSM6DSL_TEMP_CFG_REG_A (0x1F)


#define LSM6DSL_OUTX_L_G (0x22) //0b00100010 Gyroscope
#define LSM6DSL_OUTX_H_G (0x23) //0b00100011
#define LSM6DSL_OUTY_L_G (0x24) //0b00100100
#define LSM6DSL_OUTY_H_G (0x25) //0b00100101
#define LSM6DSL_OUTZ_L_G (0x26) //0b00100110
#define LSM6DSL_OUTZ_H_G (0x27) //0b00100111

#define LSM6DSL_OUTX_L_XL (0x28) //0b00101000 Accelerometer
#define LSM6DSL_OUTX_H_XL (0x29) //0b00101001
#define LSM6DSL_OUTY_L_XL (0x2A) //0b00101010
#define LSM6DSL_OUTY_H_XL (0x2B) //0b00101011
#define LSM6DSL_OUTZ_L_XL (0x2C) //0b00101100
#define LSM6DSL_OUTZ_H_XL (0x2D) //0b00101101

//LSM303AGR
#define LSM303AGR_mask_read (0x80) // 1 MSB

#define LSM303AGR_OUT_TEMP_L_A (0x0C | LSM303AGR_mask_read)

#define LSM303AGR_ADR_ACCE (0b0011001 << 1)
#define LSM303AGR_ADR_GYRO (0b0011110 << 1)
#define LSM303AGR_ADR_WHOAMI_ACCE (0x0F)
#define LSM303AGR_ADR_WHOAMI_GYRO (0x4F)
#define LSM303AGR_CTRL_REG1_A (0x20)
#define LSM303AGR_CTRL_REG2_A (0x21)
#define LSM303AGR_CTRL_REG3_A (0x22)
#define LSM303AGR_CTRL_REG4_A (0x23)
#define LSM303AGR_TEMP_CFG_REG_A (0x1F)

#define LSM303AGR_OUT_X_L_A  (0x28 | LSM303AGR_mask_read) //(0b0101000)
#define LSM303AGR_OUT_X_H_A  (0x29 | LSM303AGR_mask_read) //(0b0101001)
#define LSM303AGR_OUT_Y_L_A  (0x2A | LSM303AGR_mask_read) //(0b0101010)
#define LSM303AGR_OUT_Y_H_A  (0x2B | LSM303AGR_mask_read) //(0b0101011)
#define LSM303AGR_OUT_Z_L_A  (0x2C | LSM303AGR_mask_read) //(0b0101100)
#define LSM303AGR_OUT_Z_H_A  (0x2D | LSM303AGR_mask_read) //(0b0101101)







#define TEMP_CFG_REG_A_ADR (0x1F)


//LPS22HB

#define LPS22HB_ADR (0b1011101 << 1)
#define LPS22HB_ADR_WHOAMI (0x0F)

#define LPS22HB_PRESS_OUT_XL  (0x28)
#define LPS22HB_PRESS_OUT_L  (0x29)
#define LPS22HB_PRESS_OUT_H  (0x2A)
#define LPS22HB_TEMP_OUT_L  (0x2B)
#define LPS22HB_TEMP_OUT_H R (0x2C)

#define LPS22HB_LPS22HB_CTRL_REG1 (0x10)

#endif /* INC_MYHEADER_H_ */
