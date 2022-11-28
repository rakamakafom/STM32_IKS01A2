/*
 * myfun.c
 *
 *  Created on: Nov 20, 2022
 *      Author: Damian
 */

#include "myheader.h"
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t Call_Back_Flag = 0;

//LSM303AGR
extern uint8_t LSM303AGR_DATA_ACCE_buff[6];

int16_t LSM303AGR_OUTX_ACCE_buf = 0;
int16_t LSM303AGR_OUTY_ACCE_buf = 0;
int16_t LSM303AGR_OUTZ_ACCE_buf = 0;

//TEMP
int16_t LSM303AGR_OUT_TEMP_data = 0;
float_t LSM303AGR_OUT_TEMP_IN_C = 0;

//GYRRO
uint8_t LSM303AGR_DATA_GYRRO_buff[6];
int16_t LSM303AGR_OUTX_GYRRO_buf = 0;
int16_t LSM303AGR_OUTY_GYRRO_buf = 0;
int16_t LSM303AGR_OUTZ_GYRRO_buf = 0;


//LSM6DSL
extern int16_t LSM6DSL_DATA_BUFOR[3];
uint8_t LSM6DSL_DATA_ACCE_buff[6];

int16_t LSM6DSL_OUTX_ACCE_buff = 0;
int16_t LSM6DSL_OUTY_ACCE_buff = 0;
int16_t LSM6DSL_OUTZ_ACCE_buff = 0;


//LPS22HB
uint8_t LPS22HB_DATA_buff[5];

uint32_t LPS22HB_OUT_PRESS_DATA = 0;
int16_t LPS22HB_TEMP_DATA = 0;
float LPS22HB_TEMP_DATA_IN_C = 0;
float LPS22HB_OUT_PRESS_DATA_IN_HPA = 0;

//HTS221
uint8_t HTS221_DATA_buff[4];
uint32_t HTS221_HUMI_DATA = 0;
uint16_t HTS221_TEMP_DATA = 0;

extern uint16_t HTTS221_H0_rH_x2_sett;
extern uint16_t HTTS221_H1_rH_x2_sett;
extern int16_t HTTS221_H0_T0_OUT_sett;
extern int16_t HTTS221_H1_T0_OUT_sett;


float HTS221_TEMP_DATA_IN_C = 0;
float HTS221_HUMI_DATA_IN_PER = 0;
/* USER CODE END PV */


void uart_send_string_DMA(char* string, int16_t length)
{

	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) string, length);


}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	//ACCE LSM303AGR
	if(Call_Back_Flag == 0){
		LSM303AGR_OUTX_ACCE_buf = (LSM303AGR_DATA_ACCE_buff[1] << 8) + LSM303AGR_DATA_ACCE_buff[0];
		LSM303AGR_OUTY_ACCE_buf = (LSM303AGR_DATA_ACCE_buff[3] << 8) + LSM303AGR_DATA_ACCE_buff[2];
		LSM303AGR_OUTZ_ACCE_buf = (LSM303AGR_DATA_ACCE_buff[5] << 8) + LSM303AGR_DATA_ACCE_buff[4];
		Call_Back_Flag = 1;
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM6DSL_ADR, LSM6DSL_OUTX_L_XL, 1, LSM6DSL_DATA_ACCE_buff, 6);

	}
	//ACCE LSM6DSL
	else if(Call_Back_Flag == 1){
		LSM6DSL_OUTX_ACCE_buff = (LSM6DSL_DATA_ACCE_buff[1] << 8) + LSM6DSL_DATA_ACCE_buff[0];
		LSM6DSL_OUTY_ACCE_buff = (LSM6DSL_DATA_ACCE_buff[3] << 8) + LSM6DSL_DATA_ACCE_buff[2];
		LSM6DSL_OUTZ_ACCE_buff = (LSM6DSL_DATA_ACCE_buff[5] << 8) + LSM6DSL_DATA_ACCE_buff[4];
		LSM6DSL_DATA_BUFOR[0] = LSM6DSL_OUTX_ACCE_buff;
		LSM6DSL_DATA_BUFOR[1] = LSM6DSL_OUTY_ACCE_buff;
		LSM6DSL_DATA_BUFOR[2] = LSM6DSL_OUTZ_ACCE_buff;
		//sprintf( (char*) LSM6DSL_DATA_BUFOR, "%d %d %d \n\r", LSM6DSL_OUTX_ACCE_buff, LSM6DSL_OUTY_ACCE_buff, LSM6DSL_OUTZ_ACCE_buff);

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
		HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_ADR, HTS221_HUMIDITY_OUT_L, 1, HTS221_DATA_buff, 4);
		}

	//TEMP HTS221
	else if(Call_Back_Flag == 3){
		HTS221_HUMI_DATA = (HTS221_DATA_buff[1] << 8) + HTS221_DATA_buff[0];

		//hts221_compute_hummidity(&HTS221_HUMI_DATA, HTTS221_H0_rH_x2_sett, HTTS221_H1_rH_x2_sett, HTTS221_H0_T0_OUT_sett, HTTS221_H1_T0_OUT_sett);
		Call_Back_Flag = 4;
		HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_OUT_TEMP_L_A, 1, LSM303AGR_DATA_ACCE_buff, 2);
		}

	//TEMP LSM303AGR
	else if(Call_Back_Flag == 4){
			LSM303AGR_OUT_TEMP_data = (LSM303AGR_DATA_ACCE_buff[1] << 8) + LSM303AGR_DATA_ACCE_buff[0];
			LSM303AGR_OUT_TEMP_IN_C = lsm303agr_from_lsb_nm_to_celsius(LSM303AGR_OUT_TEMP_data);
			Call_Back_Flag = 0;
			HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_OUT_X_L_A, 1, LSM303AGR_DATA_ACCE_buff, 6);
			}

/*
	else if(Call_Back_Flag == 5){
			HTS221_HUMI_DATA = (HTS221_DATA_buff[1] << 8) + HTS221_DATA_buff[0];
			//hts221_compute_hummidity(&HTS221_HUMI_DATA, HTTS221_H0_rH_x2_sett, HTTS221_H1_rH_x2_sett, HTTS221_H0_T0_OUT_sett, HTTS221_H1_T0_OUT_sett);
			Call_Back_Flag = 0;
			HAL_I2C_Mem_Read_IT(&hi2c1, LSM303AGR_ADR_ACCE, LSM303AGR_OUT_X_L_A, 1, LSM303AGR_DATA_ACCE_buff, 6);
			}
*/


}

void hts221_compute_hummidity(uint16_t* value, int16_t HTTS221_H0_rH_x2_sett, int16_t HTTS221_H1_rH_x2_sett, int16_t HTTS221_H0_T0_OUT_sett, int16_t HTTS221_H1_T0_OUT_sett){

	int32_t tmp;
	/*5. Compute the RH [%] value by linear interpolation */

	tmp = ((int32_t)(*value - HTTS221_H0_T0_OUT_sett)) * ((int32_t)(HTTS221_H1_rH_x2_sett)*10);

	*value = (tmp/( HTTS221_H1_T0_OUT_sett- HTTS221_H0_T0_OUT_sett) + HTTS221_H0_rH_x2_sett);
	/* Saturation condition*/

	 if(*value>1000) *value = 1000;


}

float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb)
{
  return ( ( (float_t)lsb / 64.0f ) / 4.0f ) + 25.0f;
}

