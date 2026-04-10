/* lsm6dso_imu.h */
#ifndef LSM6DSO_IMU_H
#define LSM6DSO_IMU_H

#include "lsm6dso_reg.h"
#include "stm32h7xx_hal.h"

/* === Résultat === */
#define LSM6DSO_IMU_OK    0
#define LSM6DSO_IMU_ERR  -1

/* WHO_AM_I attendu */
#define LSM6DSO_WHOAMI    0x6C

/* Structure d'un sample IMU complet */
typedef struct {
    float accel_mg[3];   // mg  (X, Y, Z)
    float gyro_mdps[3];  // mdps (X, Y, Z)
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
} LSM6DSO_Sample_t;


/*********** structure pour IMU *********/
typedef struct {
    float accel_mg[3];   /* [0]=X [1]=Y [2]=Z en mg */
    float gyro_mdps[3];  /* [0]=X [1]=Y [2]=Z en mdps */
    uint32_t timestamp;  /* TIM2 en µs */
} IMU_Sample_t;
/**********************************************/

/* Fonctions publiques */
int32_t LSM6DSO_IMU_Init(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin);
int32_t LSM6DSO_IMU_Start(void);

int32_t  LSM6DSO_IMU_GetFIFOLevel(uint16_t *level);

int32_t LSM6DSO_IMU_ReadSample(LSM6DSO_Sample_t *sample);
int32_t LSM6DSO_IMU_ReadFIFO(IMU_Sample_t *sample) ;

/* Contexte global (accessible pour vTaskIMUAcq) */
extern stmdev_ctx_t lsm6dso_ctx;

#endif /* LSM6DSO_IMU_H */
