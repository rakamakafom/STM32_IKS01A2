/*
 * myfun.c
 *
 *  Created on: Nov 20, 2022
 *      Author: Damian
 */

#include "myheader.h"
#include "lcd_i2c.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t Call_Back_Flag = 0;
extern struct lcd_disp disp;
//LSM303AGR
extern uint8_t LSM303AGR_DATA_ACCE_buff[6];

int16_t LSM303AGR_OUTX_ACCE_buf = 0;
int16_t LSM303AGR_OUTY_ACCE_buf = 0;
int16_t LSM303AGR_OUTZ_ACCE_buf = 0;
float_t LSM303AGR_OUTX_ACCE_in_g = 0;
float_t LSM303AGR_OUTY_ACCE_in_g = 0;
float_t LSM303AGR_OUTZ_ACCE_in_g = 0;

//TEMP
int16_t LSM303AGR_OUT_TEMP_data = 0;
float_t LSM303AGR_OUT_TEMP_IN_C = 0;

//GYRRO
uint8_t LSM303AGR_DATA_GYRRO_buff[6];
int16_t LSM303AGR_OUTX_GYRRO_buf = 0;
int16_t LSM303AGR_OUTY_GYRRO_buf = 0;
int16_t LSM303AGR_OUTZ_GYRRO_buf = 0;


//LSM6DSL
extern uint8_t UART_BUFFOR[50];
uint8_t LSM6DSL_DATA_ACCE_buff[6];

int16_t LSM6DSL_OUTX_ACCE_buff = 0;
int16_t LSM6DSL_OUTY_ACCE_buff = 0;
int16_t LSM6DSL_OUTZ_ACCE_buff = 0;


float_t LSM6DSL_OUTX_ACCE_in_g = 0;
float_t LSM6DSL_OUTY_ACCE_in_g = 0;
float_t LSM6DSL_OUTZ_ACCE_in_g = 0;

uint8_t LSM6DSL_DATA_GYRRO_buff[6];

int16_t LSM6DSL_OUTX_GYRRO_buff = 0;
int16_t LSM6DSL_OUTY_GYRRO_buff = 0;
int16_t LSM6DSL_OUTZ_GYRRO_buff = 0;


float_t LSM6DSL_OUTX_GYRRO_in_ = 0;
float_t LSM6DSL_OUTY_GYRRO_in_ = 0;
float_t LSM6DSL_OUTZ_GYRRO_in_ = 0;


//LPS22HB
uint8_t LPS22HB_DATA_buff[5];

uint32_t LPS22HB_OUT_PRESS_DATA = 0;
int16_t LPS22HB_TEMP_DATA = 0;
float LPS22HB_TEMP_DATA_IN_C = 0;
float LPS22HB_OUT_PRESS_DATA_IN_HPA = 0;

//HTS221
uint8_t HTS221_DATA_buff[4];
int16_t HTS221_HUMI_DATA = 0;
int16_t HTS221_TEMP_DATA = 0;

extern uint16_t HTTS221_H0_rH_x2_sett;
extern uint16_t HTTS221_H1_rH_x2_sett;
extern int16_t HTTS221_H0_T0_OUT_sett;
extern int16_t HTTS221_H1_T0_OUT_sett;

extern uint8_t HTTS221_T0_degC;
extern uint8_t HTTS221_T1_degC;
extern uint8_t HTTS221_T_MSB;
extern int16_t HTTS221_T0_OUT;
extern int16_t HTTS221_T1_OUT;
extern int16_t HTTS221_T_OUT;

extern uint8_t HTTS221_H0_rH_x2;
extern uint8_t HTTS221_H1_rH_x2;
extern int16_t HTTS221_H0_T0_OUT;
extern int16_t HTTS221_H1_T0_OUT;

float HTS221_TEMP_DATA_IN_C = 0;
float HTS221_HUMI_DATA_IN_PER = 0;

extern int8_t sizeuart;
/* USER CODE END PV */


void uart_send_string_DMA(char* string, int16_t length)
{
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) string, length);

}
//void uart_send_num_DMA(int16_t* string, int16_t length)
//{
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) string, length);

//}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

	//ACCE LSM303AGR
	if(Call_Back_Flag == 0){
		LSM303AGR_OUTX_ACCE_buf = (LSM303AGR_DATA_ACCE_buff[1] << 8) + LSM303AGR_DATA_ACCE_buff[0];
		LSM303AGR_OUTY_ACCE_buf = (LSM303AGR_DATA_ACCE_buff[3] << 8) + LSM303AGR_DATA_ACCE_buff[2];
		LSM303AGR_OUTZ_ACCE_buf = (LSM303AGR_DATA_ACCE_buff[5] << 8) + LSM303AGR_DATA_ACCE_buff[4];
		LSM303AGR_OUTX_ACCE_in_g = scale_acce_8g(LSM303AGR_OUTX_ACCE_buf);
		LSM303AGR_OUTY_ACCE_in_g = scale_acce_8g(LSM303AGR_OUTY_ACCE_buf);
		LSM303AGR_OUTZ_ACCE_in_g = scale_acce_8g(LSM303AGR_OUTZ_ACCE_buf);
		Call_Back_Flag = 1;
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM6DSL_ADR, LSM6DSL_OUTX_L_XL, 1, LSM6DSL_DATA_ACCE_buff, 6);
	}
	//ACCE LSM6DSL
	else if(Call_Back_Flag == 1){
		LSM6DSL_OUTX_ACCE_buff = (LSM6DSL_DATA_ACCE_buff[1] << 8) + LSM6DSL_DATA_ACCE_buff[0];
		LSM6DSL_OUTY_ACCE_buff = (LSM6DSL_DATA_ACCE_buff[3] << 8) + LSM6DSL_DATA_ACCE_buff[2];
		LSM6DSL_OUTZ_ACCE_buff = (LSM6DSL_DATA_ACCE_buff[5] << 8) + LSM6DSL_DATA_ACCE_buff[4];
		LSM6DSL_OUTX_ACCE_in_g = scale_acce_8g(LSM6DSL_OUTX_ACCE_buff);
		LSM6DSL_OUTY_ACCE_in_g = scale_acce_8g(LSM6DSL_OUTY_ACCE_buff);
		LSM6DSL_OUTZ_ACCE_in_g = scale_acce_8g(LSM6DSL_OUTZ_ACCE_buff);
		sprintf((char*) UART_BUFFOR, "FFF%6.4f %6.4f %6.4f \n\r", LSM6DSL_OUTX_ACCE_in_g, LSM6DSL_OUTX_ACCE_in_g, LSM6DSL_OUTX_ACCE_in_g);
		//LSM6DSL_DATA_BUFOR[0] = LSM6DSL_OUTX_ACCE_buff;
		//LSM6DSL_DATA_BUFOR[1] = LSM6DSL_OUTY_ACCE_buff;
		//LSM6DSL_DATA_BUFOR[2] = LSM6DSL_OUTZ_ACCE_buff;
		Call_Back_Flag = 2;
		HAL_I2C_Mem_Read_IT(&hi2c1, LPS22HB_ADR, LPS22HB_PRESS_OUT_XL, 1, LPS22HB_DATA_buff, 5);
	}
	//TEMP HUMI LPS22HB
	else if(Call_Back_Flag == 2){
		LPS22HB_TEMP_DATA =  (LPS22HB_DATA_buff[4] << 8) + LPS22HB_DATA_buff[3] ;
		LPS22HB_TEMP_DATA_IN_C = ((float)LPS22HB_TEMP_DATA/100);
		LPS22HB_OUT_PRESS_DATA = ((LPS22HB_DATA_buff[2] << 16) + (LPS22HB_DATA_buff[1] << 8) + LPS22HB_DATA_buff[0]);
		LPS22HB_OUT_PRESS_DATA_IN_HPA = ((float) LPS22HB_OUT_PRESS_DATA / 4096);
		Call_Back_Flag = 3;
		HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_ADR, HTS221_TEMP_OUT_L, 1, HTS221_DATA_buff, 2);
		}
	//TEMP HTS221
	else if(Call_Back_Flag == 3){
		HTS221_TEMP_DATA = ((HTS221_DATA_buff[1] << 8) + HTS221_DATA_buff[0]);
		HTS221_TEMP_DATA_IN_C = hts221_compute_temp(HTS221_TEMP_DATA, HTTS221_T0_OUT, HTTS221_T1_OUT, HTTS221_T0_degC, HTTS221_T1_degC);
		Call_Back_Flag = 4;
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_OUT_TEMP_L_A, 1, LSM303AGR_DATA_ACCE_buff, 2);
		}
	//TEMP LSM303AGR
	else if(Call_Back_Flag == 4){
		LSM303AGR_OUT_TEMP_data = (LSM303AGR_DATA_ACCE_buff[1] << 8) + LSM303AGR_DATA_ACCE_buff[0];
		LSM303AGR_OUT_TEMP_IN_C = lsm303agr_from_lsb_nm_to_celsius(LSM303AGR_OUT_TEMP_data);
		Call_Back_Flag = 5;
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM6DSL_ADR, LSM6DSL_OUTX_L_G, 1, LSM6DSL_DATA_GYRRO_buff, 6);
			}
	//GYRRO LSM6DSL
	else if(Call_Back_Flag == 5){
		LSM6DSL_OUTX_GYRRO_buff = (LSM6DSL_DATA_GYRRO_buff[1] << 8) + LSM6DSL_DATA_GYRRO_buff[0];
		LSM6DSL_OUTY_GYRRO_buff = (LSM6DSL_DATA_GYRRO_buff[3] << 8) + LSM6DSL_DATA_GYRRO_buff[2];
		LSM6DSL_OUTZ_GYRRO_buff = (LSM6DSL_DATA_GYRRO_buff[5] << 8) + LSM6DSL_DATA_GYRRO_buff[4];
		LSM6DSL_OUTX_GYRRO_in_ = scale_acce_8g(LSM6DSL_OUTX_GYRRO_buff);
		LSM6DSL_OUTY_GYRRO_in_ = scale_acce_8g(LSM6DSL_OUTY_GYRRO_buff);
		LSM6DSL_OUTZ_GYRRO_in_ = scale_acce_8g(LSM6DSL_OUTZ_GYRRO_buff);
		Call_Back_Flag = 6;
				//sprintf((char *)disp.f_line, "To 1. linia");
				//sprintf((char *)disp.s_line, "a to druga linia");
				//lcd_display(&disp);
		HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_ADR, HTS221_HUMIDITY_OUT_L, 1, HTS221_DATA_buff, 2);
			}

	else if(Call_Back_Flag == 6){
		HTS221_HUMI_DATA= ((HTS221_DATA_buff[1] << 8) + HTS221_DATA_buff[0]);
		HTS221_HUMI_DATA_IN_PER = hts221_compute_hummidity(HTS221_HUMI_DATA, HTTS221_H0_rH_x2, HTTS221_H1_rH_x2,  HTTS221_H0_T0_OUT, HTTS221_H1_T0_OUT);
		Call_Back_Flag = 0;
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_OUT_X_L_A, 1, LSM303AGR_DATA_ACCE_buff, 6);
		}




}



float_t hts221_compute_temp(int16_t T_OUT, int16_t T0_OUT, int16_t T1_OUT, int16_t T0_degC, int16_t T1_degC)
{

	float_t val1 = 0, val2 = 0, val3 = 0, val_total = 0;
	val1 = (float_t) ( T1_degC - T0_degC );
	val2 = (float_t) ( T_OUT - T0_OUT );
	val3 = (float_t) ( T1_OUT - T0_OUT );
	val_total = ((val1 * val2) / (val3)) + T0_degC;

	return val_total;

}

float_t hts221_compute_hummidity(int16_t H_T_OUT, uint8_t H0_rH, uint8_t H1_rH, int16_t H0_T0_OUT, int16_t H1_T0_OUT)
{
	float_t val1 = 0, val2 = 0, val3 = 0, val_total = 0;
	val1 = (float_t) (H1_rH - H0_rH);
	val2 = (float_t) (H_T_OUT - H0_T0_OUT);
	val3 = (float_t) (H1_T0_OUT - H0_T0_OUT);
	val_total = ((val1 * val2) / (val3)) + H0_rH;
	if(val_total >= 100.00) val_total = 100.00;

	return val_total;

}

float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb)
{
	return ( ( (float_t)lsb / 64.0f ) / 4.0f ) + 25.0f;
}

float_t scale_acce_8g(int16_t data)
{
	return ((float_t) data * (float_t) 8)/ (float_t) 32768;
}


