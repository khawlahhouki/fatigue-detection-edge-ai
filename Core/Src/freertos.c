/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "max86141.h"
#include "ads1292r.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "string.h"
#include "lsm6dso_imu.h"
#include "tmp117.h"
#include "shtc3.h"
#include "ppg_filter.h"
#include "nlms_filter.h"
#include "imu_interpolation.h"
#include "ppg_features.h"
#include "emg_filter.h"
#include "emg_features.h"
#include "temp_features.h"
#include "safe_printf.h"
#include "imu_features.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t t_ref;
    uint16_t ppg_win_idx;
    uint32_t ppg_count;
} PPG_Notify_t;

typedef struct {

    /* --- PPG : 8 features --- */
    float ppg_bpm;          /* Fréquence cardiaque (bpm)          */
    float ppg_sdnn;         /* Écart-type intervalles R-R (ms)    */
    float ppg_rmssd;        /* Racine carrée diff. R-R (ms)       */
    float ppg_pnn50;        /* Proportion écarts R-R > 50ms (%)   */
    float ppg_lf_hf;        /* Ratio puissances LF/HF             */
    float ppg_spo2;         /* Saturation oxygène (%)             */
    float ppg_amplitude;    /* Amplitude crête-à-crête moyenne    */
    float ppg_skewness;     /* Asymétrie du signal                */

    /* --- IMU : 4 features --- */
    float imu_rms_accel;    /* RMS accélération (mg)              */
    float imu_variance;     /* Variance accélération              */
    float imu_peaks;        /* Nombre pics mouvement / seconde    */
    float imu_rms_gyro;     /* RMS gyroscope (dps)                */

    /* --- EMG : 8 features --- */
    float emg_rms_ch1;      /* RMS canal 1                        */
    float emg_rms_ch2;      /* RMS canal 2                        */
    float emg_mav_ch1;      /* Mean Absolute Value canal 1        */
    float emg_mav_ch2;      /* Mean Absolute Value canal 2        */
    float emg_iemg_ch1;     /* Intégrale EMG canal 1              */
    float emg_iemg_ch2;     /* Intégrale EMG canal 2              */
    float emg_mdf;          /* Fréquence médiane (Hz)             */
    float emg_mnf;          /* Fréquence moyenne (Hz)             */

    /* --- Température : 4 features --- */
    float temp_mean;        /* Température peau moyenne (°C)      */
    float temp_delta;       /* Écart peau - ambiante (°C)         */
    float temp_drift;       /* Dérive temporelle (°C/s)           */
    float temp_variance;    /* Variance température               */

    /* --- Métadonnées --- */
    uint32_t t_ref;         /* Ancre temporelle de cette fenêtre  */
    uint32_t ppg_count;     /* Compteur total samples PPG         */

} FeatureVector_t;
/* sizeof(FeatureVector_t) = 24×4 + 4 + 4 = 104 octets         */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PD */
/****************************** Elle regroupe des données de 3 sources
   → TMP117 + SHTC3 + TIM2*******************************************/
/* Taille = 4+4+4+4 = 16 octets
 * cohérent avec Q_TEMPHandle configuré 10 × 16 octets */
typedef struct {
    float    temp_skin_c;      /* TMP117 — température peau en °C  */
    float    temp_ambient_c;   /* SHTC3  — température ambiante °C */
    float    humidity_pct;     /* SHTC3  — humidité relative en %  */
    uint32_t timestamptemp117; /* TIM2   — instant mesure TMP117   */
    uint32_t timestampSHTC3;   /* TIM2   — instant mesure SHTC3    */
} Temp_Sample_t;


typedef struct {
    uint32_t porteur_id;      // 4 octets
    float    confiance;       // 4 octets
    uint8_t  classe;          // 1 octet
} LoRaMessage_t;

/* USER CODE END Elle regroupe des données de 3 sources
   → TMP117 + SHTC3 + TIM2PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StreamBufferHandle_t SB_PPG = NULL;
StreamBufferHandle_t SB_EMG = NULL;
StreamBufferHandle_t SB_IMU = NULL;
StreamBufferHandle_t SB_IMU_FEATURES = NULL;

SemaphoreHandle_t sem_dma_ppg = NULL;

/* Heartbeats des 3 tâches critiques — mis à jour à chaque cycle
 * valide. Utilisés par vTaskWatchdog pour décider si on rafraîchit
 * l'IWDG ou si on laisse le système redémarrer.                    */
volatile uint32_t hb_emg = 0;
volatile uint32_t hb_ppg = 0;
volatile uint32_t hb_imu = 0;

volatile uint32_t ppg_isr_timestamp = 0;
volatile uint32_t emg_isr_timestamp = 0;
volatile uint32_t Imu_isr_timestamp = 0;

/* Buffer features IMU, partagé entre StartTaskIMUProcess et TaskFeatures
 * 208 samples = 2s à 104Hz — glisse par moitié une fois plein,
 * exactement le même principe que emg_ch1_buf/emg_ch2_buf.           */
#define IMU_FEATURE_BUF_SIZE 208

float    imu_accel_buf[IMU_FEATURE_BUF_SIZE][3];  /* ax, ay, az (mg)   */
float    imu_gyro_buf[IMU_FEATURE_BUF_SIZE][3];   /* gx, gy, gz (mdps) */
uint32_t imu_feat_ts_buf[IMU_FEATURE_BUF_SIZE];    /* timestamp         */
uint16_t imu_feat_buf_idx   = 0;
uint8_t  imu_feat_buf_ready = 0;                   /* 1 = 1s de données disponible */
static float   imu_peak_threshold       = 999999.0f; /* très haut par défaut, tant que pas calibré : évite tout faux pic avant calibration */
static uint8_t imu_peak_threshold_ready = 0;

/* pour calculer le temps de chaque sample */
volatile uint32_t ppg_isr_timestamp;
volatile uint32_t emg_isr_timestamp;
extern uint8_t ADS1292R_data_buf[9];
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef  htim2;


/* Buffer circulaire PPG 20s */
#define PPG_WINDOW_SIZE  8000
static float    ppg_ir_buf[PPG_WINDOW_SIZE];
static float    ppg_red_buf[PPG_WINDOW_SIZE];

	  /*  tourne en cercle de 0 à 7999. Quand il atteint 8000 il revient à 0
	  c'est le buffer circulaire. Les 8000 derniers samples (= 20 s à 400 Hz) sont toujours disponibles */

/* Buffers EMG partages entre vTaskEMGProcess et TaskFeatures */
/* Buffer EMG 2 secondes = 1000 samples @ 500Hz */


float    emg_ch1_buf[EMG_BUF_SIZE];
float    emg_ch2_buf[EMG_BUF_SIZE];
uint32_t emg_ts_buf[EMG_BUF_SIZE];   /* timestamp chaque sample */
uint16_t emg_buf_idx   = 0;
uint8_t  emg_buf_ready = 0;          /* 1 = 2s de données disponibles */


/* Buffers temperature partages entre StartTempProcess et TaskFeatures */
/* 79 samples = 31s x 2.54Hz                                            */
float    temp_skin_buf[TEMP_BUF_SIZE];     /* temperature peau (C)      */
float    temp_ambient_buf[TEMP_BUF_SIZE];  /* temperature ambiante (C)  */
uint32_t temp_ts_buf[TEMP_BUF_SIZE];       /* timestamp chaque sample   */
uint8_t  temp_buf_idx   = 0;              /* index ecriture courant     */
uint8_t  temp_buf_ready = 0;              /* 1 = 31s de donnees         */

/* USER CODE END Variables */
/* Definitions for vTaskEMGAcq */
osThreadId_t vTaskEMGAcqHandle;
const osThreadAttr_t vTaskEMGAcq_attributes = {
  .name = "vTaskEMGAcq",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for vTaskPPGAcq */
osThreadId_t vTaskPPGAcqHandle;
const osThreadAttr_t vTaskPPGAcq_attributes = {
  .name = "vTaskPPGAcq",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for vTaskIMUAcq */
osThreadId_t vTaskIMUAcqHandle;
const osThreadAttr_t vTaskIMUAcq_attributes = {
  .name = "vTaskIMUAcq",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for vTaskTempAcq */
osThreadId_t vTaskTempAcqHandle;
const osThreadAttr_t vTaskTempAcq_attributes = {
  .name = "vTaskTempAcq",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vTaskDSP */
osThreadId_t vTaskDSPHandle;
const osThreadAttr_t vTaskDSP_attributes = {
  .name = "vTaskDSP",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempProcessTask */
osThreadId_t TempProcessTaskHandle;
const osThreadAttr_t TempProcessTask_attributes = {
  .name = "TempProcessTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskFeatures */
osThreadId_t TaskFeaturesHandle;
const osThreadAttr_t TaskFeatures_attributes = {
  .name = "TaskFeatures",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for vTaskEMGProcess */
osThreadId_t vTaskEMGProcessHandle;
const osThreadAttr_t vTaskEMGProcess_attributes = {
  .name = "vTaskEMGProcess",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vTaskLoRa */
osThreadId_t vTaskLoRaHandle;
const osThreadAttr_t vTaskLoRa_attributes = {
  .name = "vTaskLoRa",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMUProcessTask */
osThreadId_t IMUProcessTaskHandle;
const osThreadAttr_t IMUProcessTask_attributes = {
  .name = "IMUProcessTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Q_TEMP */
osMessageQueueId_t Q_TEMPHandle;
const osMessageQueueAttr_t Q_TEMP_attributes = {
  .name = "Q_TEMP"
};
/* Definitions for Q_RESULTS */
osMessageQueueId_t Q_RESULTSHandle;
const osMessageQueueAttr_t Q_RESULTS_attributes = {
  .name = "Q_RESULTS"
};
/* Definitions for Q_FEATURES */
osMessageQueueId_t Q_FEATURESHandle;
const osMessageQueueAttr_t Q_FEATURES_attributes = {
  .name = "Q_FEATURES"
};
/* Definitions for MTX_SPI2 */
osMutexId_t MTX_SPI2Handle;
const osMutexAttr_t MTX_SPI2_attributes = {
  .name = "MTX_SPI2"
};
/* Definitions for MTX_I2C1 */
osMutexId_t MTX_I2C1Handle;
const osMutexAttr_t MTX_I2C1_attributes = {
  .name = "MTX_I2C1"
};
/* Definitions for MTX_PPG_BUF */
osMutexId_t MTX_PPG_BUFHandle;
const osMutexAttr_t MTX_PPG_BUF_attributes = {
  .name = "MTX_PPG_BUF"
};
/* Definitions for MTX_EMG_BUF */
osMutexId_t MTX_EMG_BUFHandle;
const osMutexAttr_t MTX_EMG_BUF_attributes = {
  .name = "MTX_EMG_BUF"
};
/* Definitions for MTX_IMU_BUF */
osMutexId_t MTX_IMU_BUFHandle;
const osMutexAttr_t MTX_IMU_BUF_attributes = {
  .name = "MTX_IMU_BUF"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskEMG(void *argument);
void StartTaskPPG(void *argument);
void StartTaskIMU(void *argument);
void StartTaskTemp(void *argument);
void StartTaskDSP(void *argument);
void StartTempProcess(void *argument);
void StartTaskFeatures(void *argument);
void StartEMGProcess(void *argument);
void StartTaskLoRa(void *argument);
void StartIMUProcessTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MTX_SPI2 */
  MTX_SPI2Handle = osMutexNew(&MTX_SPI2_attributes);

  /* creation of MTX_I2C1 */
  MTX_I2C1Handle = osMutexNew(&MTX_I2C1_attributes);

  /* creation of MTX_PPG_BUF */
  MTX_PPG_BUFHandle = osMutexNew(&MTX_PPG_BUF_attributes);

  /* creation of MTX_EMG_BUF */
  MTX_EMG_BUFHandle = osMutexNew(&MTX_EMG_BUF_attributes);

  /* creation of MTX_IMU_BUF */
  MTX_IMU_BUFHandle = osMutexNew(&MTX_IMU_BUF_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  sem_dma_ppg = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Q_TEMP */
  Q_TEMPHandle = osMessageQueueNew (10, 20, &Q_TEMP_attributes);

  /* creation of Q_RESULTS */
  Q_RESULTSHandle = osMessageQueueNew (20, 64, &Q_RESULTS_attributes);

  /* creation of Q_FEATURES */
  Q_FEATURESHandle = osMessageQueueNew (1, 10, &Q_FEATURES_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of vTaskEMGAcq */
  vTaskEMGAcqHandle = osThreadNew(StartTaskEMG, NULL, &vTaskEMGAcq_attributes);

  /* creation of vTaskPPGAcq */
  vTaskPPGAcqHandle = osThreadNew(StartTaskPPG, NULL, &vTaskPPGAcq_attributes);

  /* creation of vTaskIMUAcq */
  vTaskIMUAcqHandle = osThreadNew(StartTaskIMU, NULL, &vTaskIMUAcq_attributes);

  /* creation of vTaskTempAcq */
  vTaskTempAcqHandle = osThreadNew(StartTaskTemp, NULL, &vTaskTempAcq_attributes);

  /* creation of vTaskDSP */
  vTaskDSPHandle = osThreadNew(StartTaskDSP, NULL, &vTaskDSP_attributes);

  /* creation of TempProcessTask */
  TempProcessTaskHandle = osThreadNew(StartTempProcess, NULL, &TempProcessTask_attributes);

  /* creation of TaskFeatures */
  TaskFeaturesHandle = osThreadNew(StartTaskFeatures, NULL, &TaskFeatures_attributes);

  /* creation of vTaskEMGProcess */
  vTaskEMGProcessHandle = osThreadNew(StartEMGProcess, NULL, &vTaskEMGProcess_attributes);

  /* creation of vTaskLoRa */
  vTaskLoRaHandle = osThreadNew(StartTaskLoRa, NULL, &vTaskLoRa_attributes);

  /* creation of IMUProcessTask */
  IMUProcessTaskHandle = osThreadNew(StartIMUProcessTask, NULL, &IMUProcessTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* SB_PPG — trigger = 1 lot complet de 64 paires */
    SB_PPG = xStreamBufferCreate(
        64 * sizeof(MAX86141_Sample_t) * 2,
        64 * sizeof(MAX86141_Sample_t)
    );



    SB_EMG = xStreamBufferCreate(
        128 * sizeof(ADS1292R_Sample_t),
        sizeof(ADS1292R_Sample_t)
    );

    SB_IMU = xStreamBufferCreate(
        4 * sizeof(IMU_Batch_t),   /*  4 × 273 = 1092 octets */
        1 * sizeof(IMU_Batch_t)    /* trigger = 1 batch    */
    );

    /* SB_IMU_FEATURES — trigger = 1 lot complet de 17 échantillons combinés */
    SB_IMU_FEATURES = xStreamBufferCreate(
        4 * sizeof(IMU_Feature_Batch_t),
        1 * sizeof(IMU_Feature_Batch_t)
    );
    if(SB_PPG == NULL || SB_EMG == NULL || SB_IMU == NULL || SB_IMU_FEATURES == NULL)
    {
        Error_Handler();
    }

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskEMG */
/**
  * @brief  Function implementing the vTaskEMGAcq thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskEMG */
void StartTaskEMG(void *argument)
{
  /* USER CODE BEGIN StartTaskEMG */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
	     SafePrintf("[EMG] tick\r\n");
	     osDelay(500);
  }
  /* USER CODE END StartTaskEMG */
}

/* USER CODE BEGIN Header_StartTaskPPG */
/**
* @brief Function implementing the vTaskPPGAcq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPPG */
void StartTaskPPG(void *argument)
{
  /* USER CODE BEGIN StartTaskPPG */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
	     SafePrintf("[PPG] tick\r\n");
	     osDelay(700);
  }
  /* USER CODE END StartTaskPPG */
}

/* USER CODE BEGIN Header_StartTaskIMU */
/**
* @brief Function implementing the vTaskIMUAcq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIMU */
void StartTaskIMU(void *argument)
{
  /* USER CODE BEGIN StartTaskIMU */

  IMU_Batch_t batch;
  IMU_Feature_Batch_t feature_batch;

  if (LSM6DSO_IMU_Start() != LSM6DSO_IMU_OK)
  {
    SafePrintf("[IMU_ACQ] START FAILED\r\n");
  }
  else
  {
    SafePrintf("[IMU_ACQ] Start OK\r\n");
  }

  for(;;)
  {
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)) != 0)
    {
      uint32_t sample_timestamp = Imu_isr_timestamp;
      SafePrintf("[IMU_ACQ] INT1 recu, timestamp=%lu us\r\n", sample_timestamp);

      int32_t ret = LSM6DSO_IMU_ReadFIFO_Full(batch.slots,&feature_batch,&batch.count,sample_timestamp);

      if (ret == LSM6DSO_IMU_OK)
      {
        SafePrintf("[IMU_ACQ] FIFO lu OK, count=%d\r\n", feature_batch.count);

        if (SB_IMU_FEATURES != NULL)
        {
          size_t sent = xStreamBufferSend(SB_IMU_FEATURES, &feature_batch,
                                            sizeof(IMU_Feature_Batch_t), 0);
          if (sent == sizeof(IMU_Feature_Batch_t))
          {
            SafePrintf("[IMU_ACQ] Envoye vers SB_IMU_FEATURES (%d octets)\r\n", (int)sent);
          }
          else
          {
            SafePrintf("[IMU_ACQ] ECHEC envoi SB_IMU_FEATURES (buffer plein ?)\r\n");
          }
        }
      }
      else
      {
        SafePrintf("[IMU_ACQ] ReadFIFO_Full FAILED (ret=%ld)\r\n", ret);
      }
    }
    else
    {
      SafePrintf("[IMU_ACQ] Timeout - pas d'INT1\r\n");
    }
  }

  /* USER CODE END StartTaskIMU */
}

/* USER CODE BEGIN Header_StartTaskTemp */
/**
* @brief Function implementing the vTaskTempAcq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTemp */
void StartTaskTemp(void *argument)
{
  /* USER CODE BEGIN StartTaskTemp */
	/***************************************************************************/
	/*Fréquences réelles :
	  TMP117 : 2.54 Hz (394ms par cycle)
	  SHTC3  : 2.54 Hz (394ms par cycle)
	  → suffisant car signal température < 0.1 Hz */
	/***************************************************************************/
	  static uint8_t tmp117_buf[4];

	  double skin_temp_d = 0.0;

	  int32_t shtc3_temp_raw = 0;
	  int32_t shtc3_hum_raw  = 0;

	  bool tmp_ok = false;
	  uint32_t shtc_ok = 0;

	  uint32_t tmp117_timestamp = 0;
	  uint32_t shtc3_timestamp  = 0;

	  Temp_Sample_t temp_sample;

	  for(;;)
	  {
	    osDelay(250);

	    if (osMutexAcquire(MTX_I2C1Handle, 100) == osOK)
	    {
	        setConversionMode(&hi2c1, tmp117_buf, TMP117_OS_MODE);
	        shtc3_wakeup(&hi2c1);

	        osMutexRelease(MTX_I2C1Handle);
	        osDelay(1);

	        osDelay(129);
	        tmp117_timestamp = __HAL_TIM_GET_COUNTER(&htim2);

	        if (osMutexAcquire(MTX_I2C1Handle, 100) == osOK)
	        {
	            tmp_ok = getResultTemperature(&hi2c1, tmp117_buf, &skin_temp_d);

	            shtc_ok = shtc3_perform_measurements(&hi2c1,
	                                                  &shtc3_temp_raw,
	                                                  &shtc3_hum_raw);
	            shtc3_timestamp = __HAL_TIM_GET_COUNTER(&htim2);

	            shtc3_sleep(&hi2c1);
	            osMutexRelease(MTX_I2C1Handle);
	        }
	        else
	        {
	            SafePrintf("[TEMP] MTX_I2C1Handle indisponible (lecture, timeout 100ms)\r\n");
	        }

	        /* --- Traçage : état individuel de chaque capteur --- */
	        if (!tmp_ok)
	        {
	            SafePrintf("[TEMP] TMP117 lecture FAILED\r\n");
	        }
	        if (shtc_ok != 1)
	        {
	            SafePrintf("[TEMP] SHTC3 lecture FAILED (crc/status=%lu)\r\n", shtc_ok);
	        }

	        if (tmp_ok && (shtc_ok == 1))
	        {
	          temp_sample.temp_skin_c    = (float)skin_temp_d;
	          temp_sample.temp_ambient_c = (float)shtc3_temp_raw / 100.0f;
	          temp_sample.humidity_pct   = (float)shtc3_hum_raw;
	          temp_sample.timestampSHTC3   = shtc3_timestamp;
	          temp_sample.timestamptemp117 = tmp117_timestamp;

	          /* Affichage 1 fois sur 5 : la temperature evolue lentement,
	           * inutile de tracer chaque cycle de 400ms. Reduit la
	           * contention sur le mutex SafePrintf partage. */
	          static uint8_t temp_ok_print_counter = 0;
	          temp_ok_print_counter++;
	          if (temp_ok_print_counter >= 5)
	          {
	              temp_ok_print_counter = 0;
	              SafePrintf("[TEMP] OK -> skin=%.2f C, ambient=%.2f C, hum=%.1f %%, ts_tmp117=%lu us, ts_shtc3=%lu us\r\n",
	                         temp_sample.temp_skin_c,
	                         temp_sample.temp_ambient_c,
	                         temp_sample.humidity_pct,
	                         tmp117_timestamp,
	                         shtc3_timestamp);
	          }

	          osMessageQueuePut(Q_TEMPHandle,
	                            &temp_sample,
	                            0,
	                            0);
	        }
	    }
	    else
	    {
	        SafePrintf("[TEMP] MTX_I2C1Handle indisponible (lancement mesure, timeout 100ms)\r\n");
	    }
	  }
  /* USER CODE END StartTaskTemp */
}

/* USER CODE BEGIN Header_StartTaskDSP */
/**
* @brief Function implementing the vTaskDSP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDSP */
void StartTaskDSP(void *argument)
{
  /* USER CODE BEGIN StartTaskDSP */
	uint32_t counter = 0;
  /* Infinite loop */

	  for(;;)
	  {
	    SafePrintf("[DSP] alive, counter=%lu\r\n", counter++);
	    osDelay(300);
	  }
  /* USER CODE END StartTaskDSP */
}

/* USER CODE BEGIN Header_StartTempProcess */
/**
* @brief Function implementing the TempProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempProcess */
void StartTempProcess(void *argument)
{
  /* USER CODE BEGIN StartTempProcess */
	  /* Recevoir samples temperature depuis Q_TEMP
	   * Stocker dans buffer 31s ordonne chronologiquement
	   * memmove garantit ordre chronologique comme EMG     */
	  Temp_Sample_t ts;

	  for(;;)
	  {
	    /* 1. Attendre sample depuis StartTaskTemp */
	    if(osMessageQueueGet(Q_TEMPHandle,
	                         &ts,
	                         NULL,
	                         osWaitForever) != osOK)
	    {
	      continue;
	    }

	    /* 2. Stocker dans buffer partage */
	    temp_skin_buf[temp_buf_idx]    = ts.temp_skin_c;
	    temp_ambient_buf[temp_buf_idx] = ts.temp_ambient_c;
	    temp_ts_buf[temp_buf_idx]      = ts.timestamptemp117;
	    temp_buf_idx++;

	    /* Affichage 1 fois sur 5 seulement : la temperature evolue
	     * lentement (<0.1Hz), inutile de tracer chaque echantillon.
	     * Reduit la contention sur le mutex SafePrintf partage avec
	     * les autres taches (notamment StartIMUProcessTask, plus
	     * sensible au timing). */
	    static uint8_t temp_print_counter = 0;
	    temp_print_counter++;
	    if (temp_print_counter >= 5)
	    {
	        temp_print_counter = 0;
	        SafePrintf("[TEMP_PROCESS] Sample stocke, idx=%d, skin=%.2f C, ambient=%.2f C\r\n",
	                   temp_buf_idx, ts.temp_skin_c, ts.temp_ambient_c);
	    }

	    /* 3. Buffer plein apres 79 samples = 31s */
	    if(temp_buf_idx >= TEMP_BUF_SIZE)
	    {
	      /* memmove : decaler 3 samples anciens
	       * garder les 76 derniers dans l ordre */
	      memmove(&temp_skin_buf[0],
	              &temp_skin_buf[3],
	              76 * sizeof(float));
	      memmove(&temp_ambient_buf[0],
	              &temp_ambient_buf[3],
	              76 * sizeof(float));
	      memmove(&temp_ts_buf[0],
	              &temp_ts_buf[3],
	              76 * sizeof(uint32_t));
	      temp_buf_idx   = 76;
	      temp_buf_ready = 1;

	      SafePrintf("[TEMP_PROCESS] Buffer plein -> decalage de 3, idx repositionne a %d, temp_buf_ready=1\r\n",
	                 temp_buf_idx);
	    }
	  }
  /* USER CODE END StartTempProcess */
}

/* USER CODE BEGIN Header_StartTaskFeatures */
/**
* @brief Function implementing the TaskFeatures thread.
* @param argument: Not used
* @retval None
*/

/**
  * @brief  Trouve l'index dont le timestamp est le plus proche de t_ref,
  *         dans un buffer linéaire trié par ordre chronologique croissant.
  *         Recherche à rebours depuis le plus récent (rapide en pratique :
  *         t_ref est toujours "récent", donc peu d'itérations).
  * @retval index trouvé, ou -1 si aucun timestamp n'est dans max_gap_us
  */
static int32_t Find_Closest_Timestamp_Index(uint32_t *ts_buf, uint16_t buf_idx,
                                             uint32_t t_ref, uint32_t max_gap_us)
{
    if (buf_idx == 0) return -1;

    int32_t  best_idx  = (int32_t)buf_idx - 1;
    uint32_t best_diff = (ts_buf[best_idx] > t_ref)
                          ? (ts_buf[best_idx] - t_ref)
                          : (t_ref - ts_buf[best_idx]);

    for (int32_t i = best_idx - 1; i >= 0; i--)
    {
        uint32_t diff = (ts_buf[i] > t_ref) ? (ts_buf[i] - t_ref) : (t_ref - ts_buf[i]);
        if (diff < best_diff)
        {
            best_diff = diff;
            best_idx  = i;
        }
        else
        {
            /* Buffer trié croissant → l'écart ne peut que remonter
             * après être passé sous le minimum : on peut s'arrêter. */
            break;
        }
    }

    if (best_diff > max_gap_us) return -1;   /* trop vieux → pas fiable */

    return best_idx;
}

/* USER CODE END Header_StartTaskFeatures */
void StartTaskFeatures(void *argument)
{
  /* USER CODE BEGIN StartTaskFeatures */

    FeatureVector_t fv;

    static float local_imu_accel[104][3];
    static float local_imu_gyro[104][3];

    for(;;)
    {
        /* --- TEST IMU SEUL : tick périodique 1s, t_ref pris
         * directement sur TIM2 (pas de dépendance au PPG). --- */
        osDelay(1000);
        uint32_t t_ref = __HAL_TIM_GET_COUNTER(&htim2);

        SafePrintf("[FEATURES] tick, t_ref=%lu us\r\n", t_ref);

        memset(&fv, 0, sizeof(FeatureVector_t));
        fv.t_ref = t_ref;

        /* -----------------------------------------------------
         * FEATURES IMU
         * ----------------------------------------------------- */
        uint8_t imu_data_ok = 0;

        if (!imu_feat_buf_ready)
        {
            SafePrintf("[FEATURES] IMU buffer pas encore plein, en attente...\r\n");
        }
        else if (osMutexAcquire(MTX_IMU_BUFHandle, 10) == osOK)
        {
            int32_t idx = Find_Closest_Timestamp_Index(
                              imu_feat_ts_buf, imu_feat_buf_idx,
                              t_ref, 300000);

            SafePrintf("[FEATURES] Find_Closest_Timestamp_Index -> idx=%ld (buf_idx=%d)\r\n",
                   idx, imu_feat_buf_idx);

            if (idx >= 103)
            {
                memcpy(local_imu_accel, &imu_accel_buf[idx - 103],
                       104 * sizeof(local_imu_accel[0]));
                memcpy(local_imu_gyro, &imu_gyro_buf[idx - 103],
                       104 * sizeof(local_imu_gyro[0]));
                imu_data_ok = 1;
            }
            else
            {
                SafePrintf("[FEATURES] idx insuffisant (< 103), pas assez d'historique\r\n");
            }
            osMutexRelease(MTX_IMU_BUFHandle);
        }
        else
        {
            SafePrintf("[FEATURES] MTX_IMU_BUFHandle indisponible (timeout 10ms)\r\n");
        }

        if (imu_data_ok)
        {
            fv.imu_rms_accel = IMU_Calc_RMS(local_imu_accel, 104);
            fv.imu_variance  = IMU_Calc_Var(local_imu_accel, 104);
            fv.imu_rms_gyro  = IMU_Calc_RMSGyro(local_imu_gyro, 104);
            fv.imu_peaks = IMU_Calc_Peaks(local_imu_accel, 104, imu_peak_threshold);

            SafePrintf("[FEATURES] IMU OK -> rms_accel=%.3f mg, var=%.3f mg2, peaks=%.0f, rms_gyro=%.3f mdps\r\n",
                   fv.imu_rms_accel, fv.imu_variance,
                   fv.imu_peaks, fv.imu_rms_gyro);
        }
        else
        {
            SafePrintf("[FEATURES] IMU features non calculees (donnees indisponibles)\r\n");
        }

        /* -----------------------------------------------------
         * TOUT LE RESTE COMMENTÉ POUR CE TEST (PPG, EMG, TEMP,
         * envoi vers Q_RESULTSHandle) — à réactiver plus tard.
         * ----------------------------------------------------- */

    } /* fin for(;;) */

  /* USER CODE END StartTaskFeatures */
}

/* USER CODE BEGIN Header_StartEMGProcess */
/**
* @brief Function implementing the vTaskEMGProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEMGProcess */
void StartEMGProcess(void *argument)
{
  /* USER CODE BEGIN StartEMGProcess */
	uint32_t counter = 0;
  /* Infinite loop */

	  for(;;)
	  {
	    SafePrintf("[EMG_PROCESS] alive, counter=%lu\r\n", counter++);
	    osDelay(1000);
	  }
  /* USER CODE END StartEMGProcess */
}

/* USER CODE BEGIN Header_StartTaskLoRa */
/**
* @brief Function implementing the vTaskLoRa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLoRa */
void StartTaskLoRa(void *argument)
{
  /* USER CODE BEGIN StartTaskLoRa */
  /* Infinite loop */
	 uint32_t counter = 0;
	  for(;;)
	  {
	    SafePrintf("[LORA] alive, counter=%lu\r\n", counter++);
	    osDelay(3000);
	  }
  /* USER CODE END StartTaskLoRa */
}

/* USER CODE BEGIN Header_StartIMUProcessTask */
/**
* @brief Function implementing the IMUProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUProcessTask */
void StartIMUProcessTask(void *argument)
{
	/* USER CODE BEGIN StartIMUProcessTask */
	  IMU_Feature_Batch_t feature_batch;

	  for(;;)
	  {
	    size_t bytes = xStreamBufferReceive(
	                       SB_IMU_FEATURES,
	                       &feature_batch,
	                       sizeof(IMU_Feature_Batch_t),
	                       portMAX_DELAY);

	    if (bytes != sizeof(IMU_Feature_Batch_t))
	    {
	        continue;
	    }

	    if (osMutexAcquire(MTX_IMU_BUFHandle, 5) == osOK)
	    {
	        for (uint8_t s = 0; s < feature_batch.count; s++)
	        {
	            imu_accel_buf[imu_feat_buf_idx][0] = feature_batch.slots[s].accel_mg[0];
	            imu_accel_buf[imu_feat_buf_idx][1] = feature_batch.slots[s].accel_mg[1];
	            imu_accel_buf[imu_feat_buf_idx][2] = feature_batch.slots[s].accel_mg[2];

	            imu_gyro_buf[imu_feat_buf_idx][0]  = feature_batch.slots[s].gyro_mdps[0];
	            imu_gyro_buf[imu_feat_buf_idx][1]  = feature_batch.slots[s].gyro_mdps[1];
	            imu_gyro_buf[imu_feat_buf_idx][2]  = feature_batch.slots[s].gyro_mdps[2];

	            imu_feat_ts_buf[imu_feat_buf_idx]  = feature_batch.slots[s].timestamp;

	            imu_feat_buf_idx++;

	            if (imu_feat_buf_idx >= IMU_FEATURE_BUF_SIZE)
	            {
	                /* Calibration du seuil de pics, une seule fois, sur
	                 * les 104 derniers échantillons avant le memmove. */
	                if (!imu_peak_threshold_ready)
	                {
	                    imu_peak_threshold = IMU_Calibrate_PeakThreshold(
	                                              &imu_accel_buf[104], 104, 2.0f);
	                    imu_peak_threshold_ready = 1;
	                    SafePrintf("[IMU_PROCESS] Seuil de pics calibre: %.3f mg (capteur suppose immobile)\r\n",
	                               imu_peak_threshold);
	                }

	                memmove(&imu_accel_buf[0], &imu_accel_buf[104],
	                        104 * sizeof(imu_accel_buf[0]));
	                memmove(&imu_gyro_buf[0], &imu_gyro_buf[104],
	                        104 * sizeof(imu_gyro_buf[0]));
	                memmove(&imu_feat_ts_buf[0], &imu_feat_ts_buf[104],
	                        104 * sizeof(uint32_t));

	                imu_feat_buf_idx   = 104;
	                imu_feat_buf_ready = 1;
	            }
	        }
	        SafePrintf("[IMU_PROCESS] Batch reçu, count=%d, idx=%d\r\n",
	               feature_batch.count, imu_feat_buf_idx);
	        osMutexRelease(MTX_IMU_BUFHandle);
	    }
	  }
	  /* USER CODE END StartIMUProcessTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

