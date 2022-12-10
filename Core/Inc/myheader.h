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
float_t hts221_compute_hummidity(int16_t H_T_OUT, uint8_t H0_rH, uint8_t H1_rH, int16_t H0_T0_OUT, int16_t H1_T0_OUT);
void uart_send_string_DMA(char* string, int16_t length);
//void uart_send_num_DMA(int16_t* string, int16_t length);
float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb);
float_t hts221_compute_temp(int16_t T_OUT, int16_t T0_OUT, int16_t T1_OUT, int16_t T0_degC, int16_t T1_degC);

float_t scale_acce_8g(int16_t data);
//HTS221
#define HTS221_mask_read (0x80) // 1 MSB

#define HTS221_ADR  (0b1011111 << 1)
#define HTS221_ADR_WHO_AM_I (0x0F)
#define HTS221_CTR_REG_1 (0x20) // adres bufora konfiguracyjnego
#define HTS221_HUMIDITY_OUT_L (0x28 | HTS221_mask_read)
#define HTS221_HUMIDITY_OUT_H (0x29 | HTS221_mask_read)

//TEMP
#define HTS221_TEMP_OUT_L (0x2A | HTS221_mask_read)
#define HTS221_TEMP_OUT_H (0x2B | HTS221_mask_read)
#define HTS221_T0_degC_x8  (0x32 | HTS221_mask_read) //u8
#define HTS221_T1_degC_x8  (0x33 | HTS221_mask_read) //u8
#define HTS221_T1T0_MSB   (0x35 | HTS221_mask_read) //T1.9 T1.8 T0.9 T0.8
#define HTS221_TEMP_T0_OUT (0x3C| HTS221_mask_read ) //signed16

//HUMMIDITY
#define HTS221_H0_rH_x2 (0x30 | HTS221_mask_read)	//unsigned
#define HTS221_H1_rH_x2 (0x31 | HTS221_mask_read) //unsigned
#define HTS221_H0_T0_OUT (0x36 | HTS221_mask_read ) // 16 bits signed
#define HTS221_H1_T0_OUT (0x3A | HTS221_mask_read) // 16 bits signed


//LSM6DSL

#define LSM6DSL_ADR (0b1101011 << 1)
#define LSM6DSL_ADR_WHOAMI (0x0F)
#define LSM6DSL_ADR_CTRL1_XL (0x10)
#define LSM6DSL_GYRRO_CTRL2_G (0x11)


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
//#define LSM6DSL_OUTZ_H_XL (0x2D) //0b00101101


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
