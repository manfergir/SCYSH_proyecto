/**
 ******************************************************************************
 * @file    lsm6dsl.c
 * @author  MCD Application Team
 * @brief   This file provides a set of functions needed to manage the LSM6DSL
 *          accelero and gyro devices
 ******************************************************************************
 * @attention
 *
 * <<h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lsm6dsl.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup LSM6DSL LSM6DSL
  * @{
  */

/** @defgroup LSM6DSL_Private_Variables LSM6DSL Private Variables
  * @{
  */ 
ACCELERO_DrvTypeDef Lsm6dslAccDrv =
{
  LSM6DSL_AccInit,
  LSM6DSL_AccDeInit,
  LSM6DSL_AccReadID,
  0,
  LSM6DSL_AccLowPower,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  LSM6DSL_AccReadXYZ
};

GYRO_DrvTypeDef Lsm6dslGyroDrv =
{
  LSM6DSL_GyroInit,
  LSM6DSL_GyroDeInit,
  LSM6DSL_GyroReadID,
  0,
  LSM6DSL_GyroLowPower,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  LSM6DSL_GyroReadXYZAngRate
};
/**
  * @}
  */ 

/** @defgroup LSM6DSL_ACC_Private_Functions LSM6DSL ACC Private Functions
  * @{
  */
/**
  * @brief  Set LSM6DSL Accelerometer Initialization.
  * @param  InitStruct: Init parameters
  */
void LSM6DSL_AccInit(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  uint8_t tmp;

  /* Read CTRL1_XL */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);

  /* Write value to ACC MEMS CTRL1_XL register: FS and Data Rate */
  ctrl = (uint8_t) InitStruct;
  tmp &= ~(0xFC);
  tmp |= ctrl;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, tmp);

  /* Read CTRL3_C */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

  /* Write value to ACC MEMS CTRL3_C register: BDU and Auto-increment */
  ctrl = ((uint8_t) (InitStruct >> 8));
  tmp &= ~(0x44);
  tmp |= ctrl; 
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);
}

/**
  * @brief  LSM6DSL Accelerometer De-initialization.
  */
void LSM6DSL_AccDeInit(void)
{
  uint8_t ctrl = 0x00;
  
  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);

  /* Clear ODR bits */
  ctrl &= ~(LSM6DSL_ODR_BITPOSITION);

  /* Set Power down */
  ctrl |= LSM6DSL_ODR_POWER_DOWN;
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, ctrl);
}

/**
  * @brief  Read LSM6DSL ID.
  * @retval ID 
  */
uint8_t LSM6DSL_AccReadID(void)
{  
  /* IO interface initialization */
  SENSOR_IO_Init();
  /* Read value at Who am I register address */
  return (SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WHO_AM_I_REG));
}

/**
  * @brief  Set/Unset Accelerometer in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  */
void LSM6DSL_AccLowPower(uint16_t status)
{
  uint8_t ctrl = 0x00;
  
  /* Read CTRL6_C value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL6_C);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x10);

  /* Set Low Power Mode */
  if(status)
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_XL_ENABLED;
  }else
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_XL_DISABLED;
  }
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL6_C, ctrl);
}

/**
  * @brief  Read X, Y & Z Acceleration values 
  * @param  pData: Data out pointer
  */
void LSM6DSL_AccReadXYZ(int16_t* pData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx= 0;
  uint8_t buffer[6];
  uint8_t i = 0;
  float sensitivity = 0;
  
  /* Read the acceleration control register content */
  ctrlx = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);
  
  /* Read output register X, Y & Z acceleration */
  SENSOR_IO_ReadMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_OUTX_L_XL, buffer, 6);
  
  for(i=0; i<3; i++)
  {
    pnRawData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }
  
  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL1_XL */
  switch(ctrlx & 0x0C)
  {
  case LSM6DSL_ACC_FULLSCALE_2G:
    sensitivity = LSM6DSL_ACC_SENSITIVITY_2G;
    break;
  case LSM6DSL_ACC_FULLSCALE_4G:
    sensitivity = LSM6DSL_ACC_SENSITIVITY_4G;
    break;
  case LSM6DSL_ACC_FULLSCALE_8G:
    sensitivity = LSM6DSL_ACC_SENSITIVITY_8G;
    break;
  case LSM6DSL_ACC_FULLSCALE_16G:
    sensitivity = LSM6DSL_ACC_SENSITIVITY_16G;
    break;    
  }
  
  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pData[i]=( int16_t )(pnRawData[i] * sensitivity);
  }
}

/**
  * @}
  */ 

/** @defgroup LSM6DSL_GYRO_Private_Functions LSM6DSL GYRO Private Functions
  * @{
  */

/**
  * @brief  Set LSM6DSL Gyroscope Initialization.
  * @param  InitStruct: pointer to a LSM6DSL_InitTypeDef structure 
  *         that contains the configuration setting for the LSM6DSL.
  */
void LSM6DSL_GyroInit(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  uint8_t tmp;

  /* Read CTRL2_G */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Write value to GYRO MEMS CTRL2_G register: FS and Data Rate */
  ctrl = (uint8_t) InitStruct;
  tmp &= ~(0xFC);
  tmp |= ctrl;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, tmp);

  /* Read CTRL3_C */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

  /* Write value to GYRO MEMS CTRL3_C register: BDU and Auto-increment */
  ctrl = ((uint8_t) (InitStruct >> 8));
  tmp &= ~(0x44);
  tmp |= ctrl; 
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);
}


/**
  * @brief LSM6DSL Gyroscope De-initialization
  */
void LSM6DSL_GyroDeInit(void)
{
  uint8_t ctrl = 0x00;
  
  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Clear ODR bits */
  ctrl &= ~(LSM6DSL_ODR_BITPOSITION);

  /* Set Power down */
  ctrl |= LSM6DSL_ODR_POWER_DOWN;
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, ctrl);
}

/**
  * @brief  Read ID address of LSM6DSL
  * @retval ID 
  */
uint8_t LSM6DSL_GyroReadID(void)
{
  /* IO interface initialization */
  SENSOR_IO_Init();  
  /* Read value at Who am I register address */
  return SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WHO_AM_I_REG);
}

/**
  * @brief Set/Unset LSM6DSL Gyroscope in low power mode
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled 
  */
void LSM6DSL_GyroLowPower(uint16_t status)
{  
  uint8_t ctrl = 0x00;
  
  /* Read CTRL7_G value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL7_G);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x80);

  /* Set Low Power Mode */
  if(status)
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_G_ENABLED;
  }else
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_G_DISABLED;
  }
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL7_G, ctrl);
}

/**
* @brief  Calculate the LSM6DSL angular data.
* @param  pfData: Data out pointer
*/
void LSM6DSL_GyroReadXYZAngRate(float *pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlg= 0;
  uint8_t buffer[6];
  uint8_t i = 0;
  float sensitivity = 0;
  
  /* Read the gyro control register content */
  ctrlg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);
  
  /* Read output register X, Y & Z acceleration */
  SENSOR_IO_ReadMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_OUTX_L_G, buffer, 6);
  
  for(i=0; i<3; i++)
  {
    pnRawData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }
  
  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL2_G */
  switch(ctrlg & 0x0C)
  {
  case LSM6DSL_GYRO_FS_245:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_245DPS;
    break;
  case LSM6DSL_GYRO_FS_500:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_500DPS;
    break;
  case LSM6DSL_GYRO_FS_1000:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_1000DPS;
    break;
  case LSM6DSL_GYRO_FS_2000:
    sensitivity = LSM6DSL_GYRO_SENSITIVITY_2000DPS;
    break;    
  }
  
  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=( float )(pnRawData[i] * sensitivity);
  }
}

void LSM6DSL_FifoReset(void)
{
  /* Poner FIFO en bypass y volver a FIFO para “vaciarla” */
  uint8_t ctrl5 = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_CTRL5);

  /* FIFO_MODE[2:0] = 000 bypass (mantén ODR bits) */
  ctrl5 &= ~0x07;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_CTRL5, ctrl5);

  /* vuelve a FIFO mode (001) */
  ctrl5 = (ctrl5 & ~0x07) | 0x01;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_CTRL5, ctrl5);
}

uint16_t LSM6DSL_FifoGetLevelWords(void)
{
  uint8_t s1 = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_STATUS1);
  uint8_t s2 = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_STATUS2);
  return (uint16_t)(((uint16_t)(s2 & 0x0F) << 8) | s1);
}

void LSM6DSL_FifoReadXYZRaw(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t buf[6];

  SENSOR_IO_ReadMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                         LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_L,
                         buf, 6);

  *x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  *y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
  *z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
}

void LSM6DSL_FifoConfig(uint16_t samples_xyz)
{
  uint16_t watermark_words = samples_xyz * 3; /* XYZ = 3 words */
  uint8_t wtm_l = (uint8_t)(watermark_words & 0xFF);
  uint8_t wtm_h = (uint8_t)((watermark_words >> 8) & 0x0F);

  /* Asegura auto-increment (IF_INC) y BDU como ya haces en AccInit */
  uint8_t tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);
  tmp |= LSM6DSL_ACC_GYRO_IF_INC_ENABLED; /* 0x04 */
  tmp |= LSM6DSL_BDU_BLOCK_UPDATE;        /* 0x40 */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);

  /* Configura acelerómetro en modo normal (ej 52Hz, 2G) */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);
  tmp &= ~0xFC; /* limpia ODR+FS */
  tmp |= (LSM6DSL_ODR_52Hz | LSM6DSL_ACC_FULLSCALE_2G);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, tmp);

  /* FIFO_CTRL3: mete acelerómetro en FIFO (sin decimación) */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_CTRL3, 0x01);

  /* Watermark */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_CTRL1, wtm_l);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FIFO_CTRL2, wtm_h);

  /* FIFO_CTRL5: ODR FIFO = 52Hz (0x30) + FIFO mode (001) */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                  LSM6DSL_ACC_GYRO_FIFO_CTRL5,
                  (uint8_t)(0x30 | 0x01));

  /* INT1_CTRL: habilita FIFO watermark en INT1 (bit3 en LSM6DSL) */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);
  tmp |= (1U << 3);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, tmp);
}


/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 
  
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

