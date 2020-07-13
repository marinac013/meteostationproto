#include "BME280_Driver.h"

extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly

extern uint8_t tmp_data[20];
extern bme280_calibration_param cal_param;
extern int32_t adc_T, adc_P, adc_H, t_fine;

int32_t Pressure;
uint32_t Temperature;
uint16_t Humidity;

//--------------- Writes data to device - single location
static void I2C_WriteRegister(uint8_t wrAddr, uint8_t wrData) 
{
  tmp_data[0] = wrAddr;
  tmp_data[1] = wrData;
	
	/* SEND DATA TO SPECIFIED REGISTER */
	do
  {
    if(HAL_I2C_Master_Transmit_IT (&hi2c2, (uint16_t)BME280_I2C_ADDRESS1<<1,(uint8_t *) tmp_data, 2) != HAL_OK)
    {
      BME280_ErrorHandler();
    }
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
  }
  while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
  
}

//--------------- Reads data from device - single location
static uint8_t I2C_ReadRegister(uint8_t rAddr) 
{
  tmp_data[0] = rAddr;
	
	/* SEND REGISTER ADDRESS */
	do
	{
		if(HAL_I2C_Master_Transmit_IT(&hi2c2, (uint16_t)BME280_I2C_ADDRESS1<<1, (uint8_t*)tmp_data, 1)!= HAL_OK)
		{
			BME280_ErrorHandler();
		}
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	}
	while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
	
	/* REPEATED START */
	do
	{
		if(HAL_I2C_Master_Receive_IT(&hi2c2, (uint16_t)BME280_I2C_ADDRESS1<<1, (uint8_t *)tmp_data, 1) != HAL_OK)
		{
			BME280_ErrorHandler();
		}
	}
	while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	
  return tmp_data[0];
}


void BME280_ReadMeasurements() 
{
  tmp_data[0] = BME280_PRESSURE_MSB_REG;
	
	/* SEND REGISTER ADDRESS */
	do
	{
		if(HAL_I2C_Master_Transmit_IT(&hi2c2, (uint16_t)BME280_I2C_ADDRESS1<<1, (uint8_t*)tmp_data, 1)!= HAL_OK)
		{
			BME280_ErrorHandler();
		}
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	}
	while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
	
	/* REPEATED START */
	do
	{
		if(HAL_I2C_Master_Receive_IT(&hi2c2, (uint16_t)BME280_I2C_ADDRESS1<<1, (uint8_t *)tmp_data, BME280_DATA_FRAME_SIZE) != HAL_OK)
		{
			BME280_ErrorHandler();
		}
	}
	while(HAL_I2C_GetError(&hi2c2) == HAL_I2C_ERROR_AF);
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);

  adc_H = tmp_data[BME280_DATA_FRAME_HUMIDITY_LSB_BYTE];
  adc_H |= (uint32_t)tmp_data[BME280_DATA_FRAME_HUMIDITY_MSB_BYTE] << 8;

  adc_T  = (uint32_t)tmp_data[BME280_DATA_FRAME_TEMPERATURE_XLSB_BYTE] >> 4;
  adc_T |= (uint32_t)tmp_data[BME280_DATA_FRAME_TEMPERATURE_LSB_BYTE] << 4;
  adc_T |= (uint32_t)tmp_data[BME280_DATA_FRAME_TEMPERATURE_MSB_BYTE] << 12;

  adc_P  = (uint32_t)tmp_data[BME280_DATA_FRAME_PRESSURE_XLSB_BYTE] >> 4;
  adc_P |= (uint32_t)tmp_data[BME280_DATA_FRAME_PRESSURE_LSB_BYTE] << 4;
  adc_P |= (uint32_t)tmp_data[BME280_DATA_FRAME_PRESSURE_MSB_BYTE] << 12;
}


uint8_t BME280_GetID() 
{
 return I2C_ReadRegister(BME280_CHIP_ID_REG);
}

void BME280_SoftReset() 
{
  I2C_WriteRegister(BME280_RST_REG, BME280_SOFT_RESET);
}

uint8_t BME280_GetStatus() 
{
  return I2C_ReadRegister(BME280_STAT_REG);
}

uint8_t BME280_GetCtrlMeasurement() 
{
  return I2C_ReadRegister(BME280_CTRL_MEAS_REG);
}

uint8_t BME280_GetCtrlHumidity() 
{
  return I2C_ReadRegister(BME280_CTRL_HUMIDITY_REG);
}

uint8_t BME280_GetConfig() 
{
  return I2C_ReadRegister(BME280_CONFIG_REG);
}


//Read factory calibration parameters
void BME280_ReadCalibrationParams() 
{
   uint8_t lsb, msb;
   

   msb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG);
   cal_param.dig_T1 = (uint16_t) msb;
   lsb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG);
   cal_param.dig_T1 = (cal_param.dig_T1 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG);
   cal_param.dig_T2 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG);
   cal_param.dig_T2 = (cal_param.dig_T2 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG);
   cal_param.dig_T3 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG);
   cal_param.dig_T3 = (cal_param.dig_T3 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P1_MSB_REG);
   cal_param.dig_P1 = (uint16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P1_LSB_REG);
   cal_param.dig_P1 = (cal_param.dig_P1 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P2_MSB_REG);
   cal_param.dig_P2 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P2_LSB_REG);
   cal_param.dig_P2 = (cal_param.dig_P2 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P3_MSB_REG);
   cal_param.dig_P3 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P3_LSB_REG);
   cal_param.dig_P3 = (cal_param.dig_P3 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P4_MSB_REG);
   cal_param.dig_P4 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P4_LSB_REG);
   cal_param.dig_P4 = (cal_param.dig_P4 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P5_MSB_REG);
   cal_param.dig_P5 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P5_LSB_REG);
   cal_param.dig_P5 = (cal_param.dig_P5 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P6_MSB_REG);
   cal_param.dig_P6 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P6_LSB_REG);
   cal_param.dig_P6 = (cal_param.dig_P6 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P7_MSB_REG);
   cal_param.dig_P7 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P7_LSB_REG);
   cal_param.dig_P7 = (cal_param.dig_P7 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P8_MSB_REG);
   cal_param.dig_P8 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P8_LSB_REG);
   cal_param.dig_P8 = (cal_param.dig_P8 << 8) + lsb;
   

   msb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P9_MSB_REG);
   cal_param.dig_P9 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_PRESSURE_CALIB_DIG_P9_LSB_REG);
   cal_param.dig_P9 = (cal_param.dig_P9 << 8) + lsb;
   
   lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H1_REG);
   cal_param.dig_H1 = (uint8_t) lsb;
   

   msb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H2_MSB_REG);
   cal_param.dig_H2 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG);
   cal_param.dig_H2 = (cal_param.dig_H2 << 8) + lsb;
   
   lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H3_REG);
   cal_param.dig_H3 = (uint8_t) lsb;
   

   msb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG);
   cal_param.dig_H4 = (int16_t) msb;
   lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG);
   cal_param.dig_H4 = (cal_param.dig_H4 << 4) | (lsb & 0xF);
   
   msb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG);
   cal_param.dig_H5 = (int16_t) msb;
   cal_param.dig_H5 = (cal_param.dig_H5 << 4) | (lsb >> 4);
   
   lsb = I2C_ReadRegister(BME280_HUMIDITY_CALIB_DIG_H6_REG);
   cal_param.dig_H6 = (int8_t) lsb;
}

void BME280_SetOversamplingPressure(uint8_t Value) 
{
  uint8_t ctrlm;
  ctrlm = BME280_GetCtrlMeasurement();
  ctrlm &= ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK;
  ctrlm |= Value << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS;
  
  I2C_WriteRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetOversamplingTemperature(uint8_t Value) 
{
  uint8_t ctrlm;
  ctrlm = BME280_GetCtrlMeasurement();
  ctrlm &= ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK;
  ctrlm |= Value << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS;

  I2C_WriteRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetOversamplingHumidity(uint8_t Value) 
{

  I2C_WriteRegister(BME280_CTRL_HUMIDITY_REG, Value );
}

void BME280_SetOversamplingMode(uint8_t Value) 
{
  uint8_t ctrlm;
  ctrlm = BME280_GetCtrlMeasurement();
  ctrlm |= Value;

  I2C_WriteRegister(BME280_CTRL_MEAS_REG, ctrlm);
}

void BME280_SetFilterCoefficient(uint8_t Value) 
{
  uint8_t cfgv;
  cfgv = BME280_GetConfig();
  cfgv &= ~BME280_CONFIG_REG_FILTER__MSK;
  cfgv |= Value << BME280_CONFIG_REG_FILTER__POS;
}

void BME280_SetStandbyTime(uint8_t Value) 
{
  uint8_t cfgv;
  cfgv = BME280_GetConfig();
  cfgv &= ~BME280_CONFIG_REG_TSB__MSK;
  cfgv |= Value << BME280_CONFIG_REG_TSB__POS;
}

uint8_t BME280_IsMeasuring() 
{
  uint8_t output;
  output = BME280_GetStatus();
  return (output & BME280_STAT_REG_MEASURING__MSK);
}

/****************************************************************************************************/
/* Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.  */
/***************************************************************************************************/

static int32_t BME280_Compensate_T() 
{
  int32_t temp1, temp2, T;

  temp1 = ((((adc_T>>3) -((int32_t)cal_param.dig_T1<<1))) * ((int32_t)cal_param.dig_T2)) >> 11;
  temp2 = (((((adc_T>>4) - ((int32_t)cal_param.dig_T1)) * ((adc_T>>4) - ((int32_t)cal_param.dig_T1))) >> 12) * ((int32_t)cal_param.dig_T3)) >> 14;
  t_fine = temp1 + temp2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

/************************************************************************************************************/
/* Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits). */
/* Output value of “47445” represents 47445/1024 = 46.333 %RH */
/************************************************************************************************************/

static uint32_t BME280_Compensate_H() 
{
  int32_t h1;
  h1 = (t_fine - ((int32_t)76800));
  h1 = (((((adc_H << 14) - (((int32_t)cal_param.dig_H4) << 20) - (((int32_t)cal_param.dig_H5) * h1)) +
    ((int32_t)16384)) >> 15) * (((((((h1 * ((int32_t)cal_param.dig_H6)) >> 10) * (((h1 *
    ((int32_t)cal_param.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
    ((int32_t)cal_param.dig_H2) + 8192) >> 14));
  h1 = (h1 - (((((h1 >> 15) * (h1 >> 15)) >> 7) * ((int32_t)cal_param.dig_H1)) >> 4));
  h1 = (h1 < 0 ? 0 : h1);
  h1 = (h1 > 419430400 ? 419430400 : h1);
  return (uint32_t)(h1>>12);
}

/***********************************************************************************************************/
/* Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa */
/***********************************************************************************************************/

static uint32_t BME280_Compensate_P() 
{
  int32_t press1, press2;
  uint32_t P;
  
  press1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  press2 = (((press1>>2) * (press1>>2)) >> 11 ) * ((int32_t)cal_param.dig_P6);
  press2 = press2 + ((press1*((int32_t)cal_param.dig_P5))<<1);
  press2 = (press2>>2)+(((int32_t)cal_param.dig_P4)<<16);
  press1 = (((cal_param.dig_P3 * (((press1>>2) * (press1>>2)) >> 13 )) >> 3) + ((((int32_t)cal_param.dig_P2) * press1)>>1))>>18;
  press1 =((((32768+press1))*((int32_t)cal_param.dig_P1))>>15);
  if (press1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  P = (((uint32_t)(((int32_t)1048576)-adc_P)-(press2>>12)))*3125;
  if (P < 0x80000000) {
    P = (P << 1) / ((uint32_t)press1);
  } else {
    P = (P / (uint32_t)press1) * 2;
  }
  press1 = (((int32_t)cal_param.dig_P9) * ((int32_t)(((P>>3) * (P>>3))>>13)))>>12;
  press2 = (((int32_t)(P>>2)) * ((int32_t)cal_param.dig_P8))>>13;
  P = (uint32_t)((int32_t)P + ((press1 + press2 + cal_param.dig_P7) >> 4));
  return P;
}

static void BME280_ErrorHandler(void)
{
	while(1);
}

float BME280_GetTemperature() 
{
  return (float)BME280_Compensate_T() / 100;
}

float BME280_GetHumidity() 
{
  return (float)BME280_Compensate_H() / 1024;
}

float BME280_GetPressure() 
{
  return (float) BME280_Compensate_P() / 100;
}

