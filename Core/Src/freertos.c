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
#include "lora_interface.h"
#include "max30102.h"
#include "debug_config.h"
#include "user_profile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t t_ref;
    uint16_t ppg_win_idx;
    uint32_t ppg_count;
} PPG_Notify_t;

typedef struct {
    uint32_t ir;
    uint32_t red;
    uint32_t timestamp;
} PPG_Sample_t;

typedef struct {
    /* --- Capteurs, calculés à chaque fenêtre de 1s --- */
    float acc_x, acc_y, acc_z;   /* mg, moyenne sur la fenêtre IMU */
    float wrist_hr;               /* bpm — nécessite PPG réactivé */
    float ibi;                    /* ms — nécessite PPG réactivé */
    float temp;                   /* °C, température peau */
    float temp_amb;               /* °C, température ambiante */
    float hum_amb;                /* %, humidité ambiante */
    float breathing_rpm;          /* resp/min — nécessite PPG réactivé */

    /* --- Profil utilisateur, copié depuis user_profile (saisie unique) --- */
    float age, gender, reserved, generally_trusting, lazy,
          relaxed_handless_stress, few_artistic_interests, sociable,
          criticize_others, thorough_job, easily_nervous, active_magination,
          phisical_fitness, cardiorespiratory_fitness, muscular_strength,
          agility_speed, flexibility, cardiovascular_health_conditions;

    uint32_t t_ref;
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


static float    ppg_ir_raw_buf[PPG_WINDOW_SIZE];   /* reste en RAM_D1 */
__attribute__((section(".ram_d2")))
static float    ppg_red_raw_buf[PPG_WINDOW_SIZE];  /* deplace en RAM_D2 (32 Ko libres) */

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
  .priority = (osPriority_t) osPriorityAboveNormal,
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
  .priority = (osPriority_t) osPriorityAboveNormal,
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
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for vTaskLoRaEmerge */
osThreadId_t vTaskLoRaEmergeHandle;
const osThreadAttr_t vTaskLoRaEmerge_attributes = {
  .name = "vTaskLoRaEmerge",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for vTaskInferenceI */
osThreadId_t vTaskInferenceIHandle;
const osThreadAttr_t vTaskInferenceI_attributes = {
  .name = "vTaskInferenceI",
  .stack_size = 512 * 4,
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
/* Definitions for loraTxQueue */
osMessageQueueId_t loraTxQueueHandle;
const osMessageQueueAttr_t loraTxQueue_attributes = {
  .name = "loraTxQueue"
};
/* Definitions for loraEmergencyQueue */
osMessageQueueId_t loraEmergencyQueueHandle;
const osMessageQueueAttr_t loraEmergencyQueue_attributes = {
  .name = "loraEmergencyQueue"
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
/* Definitions for loraRadioMutex */
osMutexId_t loraRadioMutexHandle;
const osMutexAttr_t loraRadioMutex_attributes = {
  .name = "loraRadioMutex"
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
void TaskLoRaEmergency(void *argument);
void StartTaskInferenceIA(void *argument);

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

  /* creation of loraRadioMutex */
  loraRadioMutexHandle = osMutexNew(&loraRadioMutex_attributes);

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

  Q_FEATURESHandle = osMessageQueueNew (1, sizeof(PPG_Notify_t), &Q_FEATURES_attributes);

  /* creation of loraTxQueue */
  loraTxQueueHandle = osMessageQueueNew (5, 12, &loraTxQueue_attributes);

  /* creation of loraEmergencyQueue */
  loraEmergencyQueueHandle = osMessageQueueNew (3, 12, &loraEmergencyQueue_attributes);

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

  /* creation of vTaskLoRaEmerge */
  vTaskLoRaEmergeHandle = osThreadNew(TaskLoRaEmergency, NULL, &vTaskLoRaEmerge_attributes);

  /* creation of vTaskInferenceI */
  vTaskInferenceIHandle = osThreadNew(StartTaskInferenceIA, NULL, &vTaskInferenceI_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* SB_PPG — trigger = 1 lot complet de 64 paires */
  SB_PPG = xStreamBufferCreate(
          64 * sizeof(PPG_Sample_t) * 2,
          64 * sizeof(PPG_Sample_t)
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

	     DBG_PRINT(DEBUG_EMG, "[EMG] tick\r\n");
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
  /* ---------------------------------------------------------------------
   * MAX30102 (I2C4, PD12/PD13) -- projet de test Test_LED_H723ZG
   * ---------------------------------------------------------------------
   * La FIFO du MAX30102 ne contient que 32 echantillons max par
   * interruption. On accumule ici, dans un buffer statique de la tache,
   * across plusieurs cycles d'interruption (typiquement 4 x ~17
   * echantillons), et on n'envoie a SB_PPG QUE lorsqu'on a exactement
   * 64 echantillons accumules.
   * --------------------------------------------------------------------- */

  #define PPG_LOT_SIZE 64

  static PPG_Sample_t lot_buf[PPG_LOT_SIZE];
  static uint8_t lot_fill = 0;  /* nombre d'échantillons déjà accumulés */
  static uint32_t ppg_timeout_counter = 0;

  uint32_t trigger_time = 0;

  /* Période d'échantillonnage nominale à 400 sps (SPO2_CONFIG = 0x2F). */
  const float ppg_sample_period_us = 2500.0f; /* 1/400Hz nominal */

  for(;;)
  {
    /* Timeout 150ms : temps max attendu entre deux interruptions FIFO
     * du MAX30102 (typiquement ~42.5ms à 400sps / FIFO_A_FULL=17). */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(150)) > 0)
    {
      trigger_time = ppg_isr_timestamp;

      /* Lire le statut efface l'interruption côté capteur */
      MAX30102_ClearInterruptFlags(&hi2c4);

      /* Vider la FIFO dans un buffer temporaire local (pas de mutex :
       * I2C4 est un bus dédié, aucune autre tâche n'y accède). */
      MAX30102_Sample raw[32];
      uint8_t raw_count = 0;
      while (MAX30102_GetNumSamplesInFifo(&hi2c4) > 0 && raw_count < 32)
      {
          MAX30102_Sample s;
          if (!MAX30102_ReadFifoSample(&hi2c4, &s))
              break;
          raw[raw_count] = s;
          raw_count++;
      }

      /* Horodatage par échantillon de CE paquet FIFO : le dernier
       * échantillon lu est le plus proche de trigger_time, on remonte
       * le temps pour les précédents. */
      uint32_t raw_ts[32];
      for (uint8_t i = 0; i < raw_count; i++)
      {
          float offset = (float)(raw_count - 1 - i) * ppg_sample_period_us;
          raw_ts[i] = trigger_time - (uint32_t)offset;
      }

      /* Accumulation dans lot_buf, échantillon par échantillon, jusqu'à
       * atteindre PPG_LOT_SIZE (64) -- alors seulement on envoie. */
      for (uint8_t i = 0; i < raw_count; i++)
      {
          lot_buf[lot_fill].ir        = raw[i].ir;
          lot_buf[lot_fill].red       = raw[i].red;
          lot_buf[lot_fill].timestamp = raw_ts[i];
          lot_fill++;

          if (lot_fill == PPG_LOT_SIZE)
          {
              if (SB_PPG != NULL)
              {
                  size_t sent = xStreamBufferSend(SB_PPG,
                                    lot_buf,
                                    PPG_LOT_SIZE * sizeof(PPG_Sample_t),
                                    0);

                  /* DEBUG THROTTLE : 1 seule ligne par lot de 64 (~170ms),
                   * pas par interruption -- a retirer une fois valide */
                  if (sent == PPG_LOT_SIZE * sizeof(PPG_Sample_t)) {

                      DBG_PRINT(DEBUG_PPG , "[PPG_ACQ] Lot de 64 envoye vers SB_PPG, IR[0]=%lu RED[0]=%lu\r\n",
                              (unsigned long)lot_buf[0].ir, (unsigned long)lot_buf[0].red);
                  } else {

                      DBG_PRINT(DEBUG_PPG , "[PPG_ACQ] ECHEC envoi SB_PPG (buffer plein ?)\r\n");
                  }
              }
              lot_fill = 0; /* on repart à zéro pour le prochain lot de 64 */
          }
      } /* fin for accumulation */
    }
    else
    {
      /* DEBUG THROTTLE : 1 ligne toutes les ~3 secondes (20 x 150ms),
       * pas à chaque timeout -- confirme que la tâche tourne et si
       * l'interruption arrive ou pas */
      if (++ppg_timeout_counter % 20 == 0) {

          DBG_PRINT(DEBUG_PPG ,"[PPG_ACQ] tick (pas d'interruption depuis 150ms x%lu)\r\n",
                  (unsigned long)ppg_timeout_counter);
      }
    }
  } /* fin for(;;) */
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

	  /* USER CODE BEGIN StartTaskIMU */

	  IMU_Batch_t batch;
	  IMU_Feature_Batch_t feature_batch;

	  if (LSM6DSO_IMU_Start() != LSM6DSO_IMU_OK)
	  {
	    DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] START FAILED\r\n");
	  }
	  else
	  {

	    DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] Start OK\r\n");

	  }

	  for(;;)
	  {
	    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)) != 0)
	    {
	      uint32_t sample_timestamp = Imu_isr_timestamp;
	      DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] INT1 recu, timestamp=%lu us\r\n", sample_timestamp);


	      int32_t ret = LSM6DSO_IMU_ReadFIFO_Full(batch.slots,&feature_batch,&batch.count,sample_timestamp);

	      if (ret == LSM6DSO_IMU_OK)
	      {

	        DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] FIFO lu OK, count=%d\r\n", feature_batch.count);

	        /* Envoi vers SB_IMU (batch brut) -- consommé par StartTaskDSP
	         * pour l'interpolation/correction NLMS du signal PPG. Distinct
	         * de SB_IMU_FEATURES (feature_batch), qui alimente une autre
	         * chaîne de traitement en aval (StartTaskIMUProcess). */
	        if (SB_IMU != NULL)
	        {
	          size_t sent_imu = xStreamBufferSend(SB_IMU, &batch,
	                                                sizeof(IMU_Batch_t), 0);
	          if (sent_imu != sizeof(IMU_Batch_t))
	          {

	            DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] ECHEC envoi SB_IMU (buffer plein ?)\r\n");
	          }
	        }

	        if (SB_IMU_FEATURES != NULL)
	        {
	          size_t sent = xStreamBufferSend(SB_IMU_FEATURES, &feature_batch,
	                                            sizeof(IMU_Feature_Batch_t), 0);
	          if (sent == sizeof(IMU_Feature_Batch_t))
	          {

	            DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] Envoye vers SB_IMU_FEATURES (%d octets)\r\n", (int)sent);
	          }
	          else
	          {
	        	 DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] ECHEC envoi SB_IMU_FEATURES (buffer plein ?)\r\n");

	          }
	        }
	      }
	      else
	      {
	    	  DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] ReadFIFO_Full FAILED (ret=%ld)\r\n", ret);

	      }
	    }
	    else
	    {
	      DBG_PRINT(DEBUG_IMU, "[IMU_ACQ] Timeout - pas d'INT1\r\n");

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
	            DBG_PRINT(DEBUG_TEMP , "[TEMP] MTX_I2C1Handle indisponible (lecture, timeout 100ms)\r\n");
	        }

	        /* --- Traçage : état individuel de chaque capteur --- */
	        if (!tmp_ok)
	        {   DBG_PRINT(DEBUG_TEMP , "[TEMP] TMP117 lecture FAILED\r\n");

	        }
	        if (shtc_ok != 1)
	        {   DBG_PRINT(DEBUG_TEMP ,"[TEMP] SHTC3 lecture FAILED (crc/status=%lu)\r\n", shtc_ok);

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

	              DBG_PRINT(DEBUG_TEMP ,"[TEMP] OK -> skin=%.2f C, ambient=%.2f C, hum=%.1f %%, ts_tmp117=%lu us, ts_shtc3=%lu us\r\n",
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
	    	 DBG_PRINT(DEBUG_TEMP ,"[TEMP] MTX_I2C1Handle indisponible (lancement mesure, timeout 100ms)\r\n");

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

/* USER CODE BEGIN Header_StartTaskDSP */
/**
* @brief Function implementing the vTaskDSP thread.
* @param argument: Not used
* @retval None
*/static void DSP_StorePPGSample(float ir_val, float red_val,
        float ir_raw, float red_raw,
        uint32_t sample_timestamp,
        uint16_t *ppg_win_idx,
        uint32_t *ppg_count,
        uint8_t  *features_armed,
        uint32_t *last_ppg_timestamp)
{
if (osMutexAcquire(MTX_PPG_BUFHandle, 5) == osOK)
{
ppg_ir_buf[*ppg_win_idx]      = ir_val;
ppg_red_buf[*ppg_win_idx]     = red_val;
ppg_ir_raw_buf[*ppg_win_idx]  = ir_raw;   /* signal brut, meme index */
ppg_red_raw_buf[*ppg_win_idx] = red_raw;
osMutexRelease(MTX_PPG_BUFHandle);
}

*ppg_win_idx = (*ppg_win_idx + 1) % PPG_WINDOW_SIZE;
(*ppg_count)++;
*last_ppg_timestamp = sample_timestamp;

if (!(*features_armed) && *ppg_count >= PPG_WINDOW_SIZE)
{
*features_armed = 1;
}

if (*features_armed && (*ppg_count % 400 == 0))
{
PPG_Notify_t notif;
notif.t_ref       = *last_ppg_timestamp;
notif.ppg_win_idx = *ppg_win_idx;
notif.ppg_count   = *ppg_count;

osMessageQueueReset(Q_FEATURESHandle);
osMessageQueuePut(Q_FEATURESHandle, &notif, 0, 0);
}
}

/* USER CODE END Header_StartTaskDSP */
void StartTaskDSP(void *argument)
{
		  /* USER CODE BEGIN StartTaskDSP */

		  /* Filtres passe-bande PPG — un par canal optique (IR et RED) */
		  static BandpassFilter_t ppg_filter_ir;
		  static BandpassFilter_t ppg_filter_red;
		  PPG_Filter_Init(&ppg_filter_ir);
		  PPG_Filter_Init(&ppg_filter_red);

		  /* Filtres adaptatifs NLMS — soustraient l'artefact de mouvement du
		   * signal PPG en utilisant l'IMU comme référence. */
		  static NLMS_Filter_t nlms_ir;
		  static NLMS_Filter_t nlms_red;
		  NLMS_Init(&nlms_ir,  0.01f);
		  NLMS_Init(&nlms_red, 0.01f);

		  /* Interpolateur IMU : magnitude de mouvement à tout instant précis,
		   * même entre deux échantillons IMU réels (IMU 104Hz vs PPG 400Hz).
		   * Timeline continue maintenue via imu_prev_end -- voir imu_interpolation.c */
		  static IMU_Interp_t imu_interp;
		  IMU_Interp_Init(&imu_interp);

		  /* Buffers de réception, alloués sur la pile de la tâche. */
		  PPG_Sample_t ppg_lot[64];          /* un lot complet de 64 échantillons PPG */
		  IMU_Batch_t  imu_received;         /* un batch IMU (jusqu'à 17 slots)       */
		  size_t       bytes_ppg = 0;

		  /* Position d'écriture courante dans le buffer circulaire ppg_ir_buf/
		   * ppg_red_buf (8000 cases, déclarés en variables globales du fichier). */
		  static uint16_t ppg_win_idx = 0;

		  /* Compteur total d'échantillons PPG traités depuis le démarrage. */
		  static uint32_t ppg_count = 0;

		  /* Passe à 1 définitivement une fois le buffer circulaire rempli
		   * une première fois en entier. */
		  static uint8_t features_armed = 0;

		  /* Horodatage du dernier échantillon traité. */
		  static uint32_t last_ppg_timestamp = 0;

		  /* Compteur de cycles, pour throttler l'affichage debug (1 ligne
		   * toutes les ~10 cycles, jamais dans la boucle des 64 echantillons). */
		  static uint32_t dsp_cycle_counter = 0;

		  for(;;)
		  {
		    /* ÉTAPE 1 — Attendre un lot complet de 64 échantillons PPG (bloquant).
		     * Timeout 500ms : à 400Hz un lot de 64 se remplit en ~160-200ms → marge. */
		    bytes_ppg = xStreamBufferReceive(
		                    SB_PPG,
		                    ppg_lot,
		                    sizeof(ppg_lot),
		                    pdMS_TO_TICKS(500));

		    if (bytes_ppg == 0) {
		        continue; /* timeout réel : capteur probablement déconnecté */
		    }

		    uint8_t ppg_ok = (bytes_ppg == sizeof(ppg_lot)) ? 1 : 0;
		    if (!ppg_ok) {
		        continue; /* lot incomplet/corrompu */
		    }

		    dsp_cycle_counter++;
		    uint8_t dsp_should_print = (dsp_cycle_counter % 10 == 0);

		    /* ÉTAPE 2 — Vider ENTIÈREMENT SB_IMU, non-bloquant (timeout=0).
		     * Maintient la timeline d'interpolation IMU continue meme si le
		     * rythme d'accumulation PPG (MAX30102) ne correspond plus 1:1 au
		     * rythme natif de l'IMU. Ne bloque JAMAIS le traitement PPG. */
		    uint8_t imu_batches_this_cycle = 0;
		    while (xStreamBufferReceive(SB_IMU, &imu_received, sizeof(IMU_Batch_t), 0)
		           == sizeof(IMU_Batch_t))
		    {
		        IMU_Interp_SetBatch(&imu_interp, &imu_received, 1);
		        imu_batches_this_cycle++;
		    }

		    if (dsp_should_print) {
		        DBG_PRINT(DEBUG_DSP, "[DSP] cycle %lu: bytes_ppg=%u, batches IMU consommes=%u\r\n",
		                   (unsigned long)dsp_cycle_counter,
		                   (unsigned)bytes_ppg,
		                   (unsigned)imu_batches_this_cycle);
		    }

		    /* ÉTAPE 3 — Traiter les 64 échantillons du lot PPG.
		     * Un seul chemin de traitement : IMU_Interp_GetMag() interroge
		     * la timeline continue à l'instant précis de chaque échantillon. */
		    for (uint8_t p = 0; p < 64; p++)
		    {
		        float mag      = IMU_Interp_GetMag(&imu_interp, ppg_lot[p].timestamp);
		        float mag_norm = IMU_Interp_Normalize(mag);

		        float ir_f  = PPG_Filter_Apply(&ppg_filter_ir,  (float)ppg_lot[p].ir);
		        float red_f = PPG_Filter_Apply(&ppg_filter_red, (float)ppg_lot[p].red);

		        float ir_clean  = NLMS_Apply(&nlms_ir,  ir_f,  mag_norm);
		        float red_clean = NLMS_Apply(&nlms_red, red_f, mag_norm);

		        DSP_StorePPGSample(ir_clean, red_clean,
		                            (float)ppg_lot[p].ir, (float)ppg_lot[p].red,  /* <-- brut ajoute */
		                            ppg_lot[p].timestamp,
		                            &ppg_win_idx, &ppg_count,
		                            &features_armed, &last_ppg_timestamp);
		    }
		  } /* fin for(;;) */

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

	        DBG_PRINT(DEBUG_TEMPPROC , "[TEMP_PROCESS] Sample stocke, idx=%d, skin=%.2f C, ambient=%.2f C\r\n",
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



	        DBG_PRINT(DEBUG_TEMPPROC , "[TEMP_PROCESS] Buffer plein -> decalage de 3, idx repositionne a %d, temp_buf_ready=1\r\n",
	                 temp_buf_idx);
	    }
	  }
  /* USER CODE END StartTempProcess */
}

/* USER CODE BEGIN Header_StartTaskFeatures */
/* USER CODE BEGIN Header_StartTaskFeatures */
/**
* @brief Function implementing the TaskFeatures thread.
* @param argument: Not used
* @retval None
*/

/**
  * @brief  Trouve l'index dont le timestamp est le plus proche de t_ref,
  *         dans un buffer linéaire trié par ordre chronologique croissant.
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
            break;
        }
    }

    if (best_diff > max_gap_us) return -1;

    return best_idx;
}

/* USER CODE END Header_StartTaskFeatures */
void StartTaskFeatures(void *argument)
{

	  /* USER CODE BEGIN StartTaskFeatures */

	    /* Notification envoyee par DSP_StorePPGSample (dans StartTaskDSP)
	     * -- contient t_ref, ppg_win_idx, ppg_count. C'est le SEUL moyen
	     * propre de recuperer ppg_win_idx, qui est une variable statique
	     * LOCALE a StartTaskDSP, invisible ici autrement. */
	    PPG_Notify_t notif;

	    static FeatureVector_t fv;

	    /* Structure intermediaire pics + intervalles R-R, partagee entre
	     * toutes les fonctions PPG_Calc* pour eviter de recalculer les
	     * pics plusieurs fois. static : trop grande pour la pile (~420o). */
	    static PPG_RR_t rr;

	    static float local_imu_accel[104][3];
	    static float local_imu_gyro[104][3];

	    for(;;)
	    {
	        /* ÉTAPE 1 -- attendre la notification de StartTaskDSP (pas un
	         * tick periodique) : bloque la tache, 0% CPU, jusqu'a ce que
	         * le buffer circulaire PPG soit plein une fois (~20s a 400Hz)
	         * puis toutes les 1s ensuite (400 echantillons = 1s a 400Hz). */
	        if (osMessageQueueGet(Q_FEATURESHandle,
	                              &notif,
	                              NULL,
	                              osWaitForever) != osOK)
	        {
	            continue;
	        }

	        DBG_PRINT(DEBUG_FEATURES, "[FEATURES] Notification recue, t_ref=%lu us, ppg_count=%lu\r\n",
	                   (unsigned long)notif.t_ref, (unsigned long)notif.ppg_count);

	        /* Protection defensive : le buffer doit etre plein (8000
	         * echantillons) avant tout calcul, sinon on calculerait sur
	         * des zeros (valeurs initiales du buffer circulaire). */
	        if (notif.ppg_count < PPG_WINDOW_SIZE)
	        {
	            DBG_PRINT(DEBUG_FEATURES, "[FEATURES] Buffer PPG pas encore plein (%lu/%d), skip\r\n",
	                       (unsigned long)notif.ppg_count, PPG_WINDOW_SIZE);
	            continue;
	        }

	        uint32_t t_ref = notif.t_ref;

	        /* ÉTAPE 2+3 fusionnées — copie ET réordonnancement chronologique en
	         * une seule passe, directement depuis les buffers globaux. Évite de
	         * dupliquer inutilement 4 tableaux de 32 Ko chacun (economie de
	         * 128 Ko de RAM statique par rapport a la version precedente).
	         * DEUX paires de buffers : filtre (BPM/HRV) et brut/DC (SpO2). */
	        static float local_ir[PPG_WINDOW_SIZE];       /* filtré, ordonné -- pour BPM/HRV */
	        static float local_red[PPG_WINDOW_SIZE];
	        static float local_dc_ir[PPG_WINDOW_SIZE];    /* brut, ordonné -- pour SpO2 */
	        static float local_dc_red[PPG_WINDOW_SIZE];

	        if (osMutexAcquire(MTX_PPG_BUFHandle, 10) == osOK)
	        {
	            uint16_t start = notif.ppg_win_idx;
	            for (uint16_t i = 0; i < PPG_WINDOW_SIZE; i++)
	            {
	                uint16_t idx = (start + i) % PPG_WINDOW_SIZE;
	                local_ir[i]     = ppg_ir_buf[idx];
	                local_red[i]    = ppg_red_buf[idx];
	                local_dc_ir[i]  = ppg_ir_raw_buf[idx];
	                local_dc_red[i] = ppg_red_raw_buf[idx];
	            }
	            osMutexRelease(MTX_PPG_BUFHandle);

	            /* DEBUG : verification que les 2 paires de buffers contiennent
	             * bien des donnees DIFFERENTES -- filtre proche de 0 (DC retire),
	             * brut proche de la vraie amplitude ADC (DC preserve). A retirer
	             * une fois le mecanisme confirme fonctionnel. */
	            DBG_PRINT(DEBUG_FEATURES,
	                       "[FEATURES] Verif buffers -> filtre[0]=%.2f brut[0]=%.0f | filtre[100]=%.2f brut[100]=%.0f\r\n",
	                       local_ir[0], local_dc_ir[0],
	                       local_ir[100], local_dc_ir[100]);
	        }
	        else
	        {
	            DBG_PRINT(DEBUG_FEATURES, "[FEATURES] MTX_PPG_BUFHandle indisponible (timeout 10ms), skip\r\n");
	            continue;
	        }

	        /* Reinitialisation de fv A CHAQUE cycle -- sans ce memset, un
	         * champ non recalcule ce tour-ci (ex: BPM si rr.n_rr<2) garderait
	         * silencieusement la valeur du cycle PRECEDENT au lieu de 0,
	         * donnant une fausse impression de continuite. */
	        memset(&fv, 0, sizeof(FeatureVector_t));
	                fv.t_ref     = t_ref;

	                /* -----------------------------------------------------
	                 * PPG — seulement wrist_hr et ibi sont retenus dans la
	                 * nouvelle liste de features (spo2/sdnn/rmssd/pnn50/lf_hf/
	                 * amplitude/skewness ne sont plus demandés). ppg_count
	                 * n'existe plus non plus dans FeatureVector_t.
	                 * ----------------------------------------------------- */
	                PPG_DetectPeaks(local_ir, PPG_WINDOW_SIZE, &rr);   /* filtré : detection de pics */
	                PPG_CalcRR(&rr);

	                DBG_PRINT(DEBUG_FEATURES, "[FEATURES] PPG_DetectPeaks -> n_peaks=%d n_rr=%d\r\n",
	                           rr.n_peaks, rr.n_rr);

	                if (rr.n_rr >= 2)
	                {
	                    float bpm = PPG_CalcBPM(&rr);
	                    fv.wrist_hr = bpm;
	                    /* IBI (ms) = 60000 / HR(bpm) — intervalle moyen inter-battements */
	                    fv.ibi = (bpm > 0.0f) ? (60000.0f / bpm) : 0.0f;

	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] PPG OK -> wrist_hr=%.1f bpm, ibi=%.1f ms (n_peaks=%d)\r\n",
	                               fv.wrist_hr, fv.ibi, rr.n_peaks);
	                }
	                else
	                {
	                    fv.wrist_hr = 0.0f;
	                    fv.ibi      = 0.0f;
	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] n_rr insuffisant (< 2) -> wrist_hr/ibi a 0\r\n");
	                }

	                fv.breathing_rpm = 0.0f;  /* TODO: nécessite une fonction dédiée d'estimation
	                                            * de fréquence respiratoire par variation
	                                            * d'amplitude PPG (filtrage passe-bas de
	                                            * l'enveloppe + comptage de pics dans la
	                                            * bande 0.1-0.5 Hz). Pas encore écrite. */

	                /* -----------------------------------------------------
	                 * FEATURES TEMPÉRATURE — calculée AVANT son utilisation,
	                 * les déclarations static doivent précéder leur usage
	                 * dans le même bloc (règle du C).
	                 * ----------------------------------------------------- */
	                static float local_temp_skin[TEMP_BUF_SIZE];
	                static float local_temp_ambient[TEMP_BUF_SIZE];
	                uint8_t temp_data_ok = 0;

	                if (!temp_buf_ready)
	                {
	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] TEMP buffer pas encore plein (31s), en attente...\r\n");
	                }
	                else
	                {
	                    int32_t idx = Find_Closest_Timestamp_Index(
	                                      temp_ts_buf, temp_buf_idx,
	                                      t_ref, 2000000);

	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] TEMP Find_Closest_Timestamp_Index -> idx=%ld (buf_idx=%d)\r\n",
	                               idx, temp_buf_idx);

	                    if (idx >= (TEMP_WIN_LONG - 1))
	                    {
	                        memcpy(local_temp_skin, &temp_skin_buf[idx - (TEMP_WIN_LONG - 1)],
	                               TEMP_WIN_LONG * sizeof(float));
	                        memcpy(local_temp_ambient, &temp_ambient_buf[idx - (TEMP_WIN_LONG - 1)],
	                               TEMP_WIN_LONG * sizeof(float));
	                        temp_data_ok = 1;
	                    }
	                    else
	                    {
	                        DBG_PRINT(DEBUG_FEATURES, "[FEATURES] TEMP idx insuffisant (< %d), pas assez d'historique\r\n",
	                                   TEMP_WIN_LONG - 1);
	                    }
	                }

	                if (temp_data_ok)
	                {
	                    fv.temp     = Temp_Calc_Mean(local_temp_skin, TEMP_WIN_LONG);
	                    fv.temp_amb = Temp_Calc_Mean(local_temp_ambient, TEMP_WIN_LONG);
	                    fv.hum_amb  = 0.0f; /* TODO: humidité pas encore stockée dans un buffer glissant */

	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] TEMP OK -> temp=%.2f C, temp_amb=%.2f C\r\n",
	                               fv.temp, fv.temp_amb);
	                }
	                else
	                {
	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] TEMP features non calculees (donnees indisponibles)\r\n");
	                }

	                /* -----------------------------------------------------
	                 * FEATURES IMU — synchronisées sur t_ref.
	                 * ----------------------------------------------------- */
	                uint8_t imu_data_ok = 0;

	                if (!imu_feat_buf_ready)
	                {
	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] IMU buffer pas encore plein, en attente...\r\n");
	                }
	                else if (osMutexAcquire(MTX_IMU_BUFHandle, 10) == osOK)
	                {
	                    int32_t idx = Find_Closest_Timestamp_Index(
	                                      imu_feat_ts_buf, imu_feat_buf_idx,
	                                      t_ref, 300000);

	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] IMU Find_Closest_Timestamp_Index -> idx=%ld (buf_idx=%d)\r\n",
	                               idx, imu_feat_buf_idx);

	                    if (idx >= 103)
	                    {
	                        memcpy(local_imu_accel, &imu_accel_buf[idx - 103],
	                               104 * sizeof(local_imu_accel[0]));
	                        imu_data_ok = 1;
	                    }
	                    else
	                    {
	                        DBG_PRINT(DEBUG_FEATURES, "[FEATURES] IMU idx insuffisant (< 103), pas assez d'historique\r\n");
	                    }
	                    osMutexRelease(MTX_IMU_BUFHandle);
	                }
	                else
	                {
	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] MTX_IMU_BUFHandle indisponible (timeout 10ms)\r\n");
	                }

	                if (imu_data_ok)
	                {
	                    IMU_Calc_MeanXYZ(local_imu_accel, 104, &fv.acc_x, &fv.acc_y, &fv.acc_z);

	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] IMU OK -> acc_x=%.3f mg, acc_y=%.3f mg, acc_z=%.3f mg\r\n",
	                               fv.acc_x, fv.acc_y, fv.acc_z);
	                }
	                else
	                {
	                    DBG_PRINT(DEBUG_FEATURES, "[FEATURES] IMU features non calculees (donnees indisponibles)\r\n");
	                    fv.acc_x = fv.acc_y = fv.acc_z = 0.0f;
	                }

	                /* -----------------------------------------------------
	                 * PROFIL UTILISATEUR — copié intégralement (18 champs),
	                 * valeurs stables sur tout le run, saisies une fois au
	                 * démarrage par UserProfile_PromptAndRead().
	                 * ----------------------------------------------------- */
	                fv.age                              = user_profile.age;
	                fv.gender                           = user_profile.gender;
	                fv.reserved                         = user_profile.reserved;
	                fv.generally_trusting               = user_profile.generally_trusting;
	                fv.lazy                             = user_profile.lazy;
	                fv.relaxed_handless_stress          = user_profile.relaxed_handless_stress;
	                fv.few_artistic_interests           = user_profile.few_artistic_interests;
	                fv.sociable                         = user_profile.sociable;
	                fv.criticize_others                 = user_profile.criticize_others;
	                fv.thorough_job                     = user_profile.thorough_job;
	                fv.easily_nervous                   = user_profile.easily_nervous;
	                fv.active_magination                = user_profile.active_magination;
	                fv.phisical_fitness                 = user_profile.phisical_fitness;
	                fv.cardiorespiratory_fitness        = user_profile.cardiorespiratory_fitness;
	                fv.muscular_strength                = user_profile.muscular_strength;
	                fv.agility_speed                    = user_profile.agility_speed;
	                fv.flexibility                      = user_profile.flexibility;
	                fv.cardiovascular_health_conditions = user_profile.cardiovascular_health_conditions;
	        /* -----------------------------------------------------
	         * RESTE COMMENTÉ POUR CE TEST (EMG, envoi vers
	         * Q_RESULTSHandle) — à réactiver plus tard.
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

  /* Infinite loop */

	  for(;;)
	  {


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


	 osThreadSuspend(osThreadGetId());

  LoRaMessage_t msg;
  DBG_PRINT(DEBUG_LORA , "\r\n\r\n########## AVANT LoRa_Init ##########\r\n\r\n");

  int lora_status = LoRa_Init();

  DBG_PRINT(DEBUG_LORA ,"\r\n\r\n########## APRES LoRa_Init, status=%d ##########\r\n\r\n", lora_status);
  for(;;)
  {
    if (osMessageQueueGet(loraTxQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      osMutexAcquire(loraRadioMutexHandle, osWaitForever);

      int sendResult = LoRa_Send((uint8_t*)&msg, sizeof(msg));
      if (sendResult == 0)
      {
        if (LoRa_WaitDone(3000))
        {
          printf("[LoRa-Routine] TX confirme, porteur=%lu classe=%d conf=%.2f\r\n",
                 msg.porteur_id, msg.classe, msg.confiance);
        }
        else
        {
          printf("[LoRa-Routine] TX timeout\r\n");
        }
      }
      else
      {
        printf("[LoRa-Routine] Echec envoi, code=%d\r\n", sendResult);
      }

      osMutexRelease(loraRadioMutexHandle);
    }
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

	                    DBG_PRINT(DEBUG_IMUPROC ,"[IMU_PROCESS] Seuil de pics calibre: %.3f mg (capteur suppose immobile)\r\n",
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
	        DBG_PRINT(DEBUG_IMUPROC ,"[IMU_PROCESS] Batch reçu, count=%d, idx=%d\r\n",
		               feature_batch.count, imu_feat_buf_idx);

	        osMutexRelease(MTX_IMU_BUFHandle);
	    }
	  }
  /* USER CODE END StartIMUProcessTask */
}

/* USER CODE BEGIN Header_TaskLoRaEmergency */
/**
* @brief Function implementing the VTaskLoRaEmerge thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskLoRaEmergency */
void TaskLoRaEmergency(void *argument)
{
  /* USER CODE BEGIN TaskLoRaEmergency */
	 osThreadSuspend(osThreadGetId());
  LoRaMessage_t msg;

  for(;;)
  {
    if (osMessageQueueGet(loraEmergencyQueueHandle, &msg, NULL, osWaitForever) == osOK)
    {
      osMutexAcquire(loraRadioMutexHandle, osWaitForever);

      SafePrintf("[LoRa-URGENCE] Envoi immediat, porteur=%lu conf=%.2f\r\n",
             msg.porteur_id, msg.confiance);

      int sendResult = LoRa_Send((uint8_t*)&msg, sizeof(msg));
      if (sendResult == 0)
      {
        if (LoRa_WaitDone(3000))
        {
          SafePrintf("[LoRa-URGENCE] TX confirme\r\n");
        }
        else
        {
          SafePrintf("[LoRa-URGENCE] TX timeout\r\n");
        }
      }
      else
      {
        SafePrintf("[LoRa-URGENCE] Echec envoi, code=%d\r\n", sendResult);
      }

      osMutexRelease(loraRadioMutexHandle);
    }
  }  // ← ACCOLADE MANQUANTE AJOUTÉE ICI (ferme le for(;;))
  /* USER CODE END TaskLoRaEmergency */
}

/* USER CODE BEGIN Header_StartTaskInferenceIA */
/**
* @brief Function implementing the vTaskInferenceI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskInferenceIA */
void StartTaskInferenceIA(void *argument)
{
  /* USER CODE BEGIN StartTaskInferenceIA */
	 osThreadSuspend(osThreadGetId());
	  LoRaMessage_t alert;
	  uint8_t testCounter = 0;

	  for(;;)
	  {
	    alert.porteur_id = 1;
	    alert.classe = (testCounter % 3 == 0) ? 1 : 0;
	    alert.confiance = (alert.classe == 1) ? 0.85f : 0.95f;
	    testCounter++;

	    if (alert.classe == 1)
	    {
	      osMessageQueuePut(loraEmergencyQueueHandle, &alert, 0, osWaitForever);
	      printf("[InferenceIA] Message envoye vers EMERGENCY (classe=%d, conf=%.2f)\r\n",
	             alert.classe, alert.confiance);
	    }
	    else
	    {
	      osMessageQueuePut(loraTxQueueHandle, &alert, 0, osWaitForever);
	      printf("[InferenceIA] Message envoye vers ROUTINE (classe=%d, conf=%.2f)\r\n",
	             alert.classe, alert.confiance);
	    }

	    osDelay(pdMS_TO_TICKS(2000));
	  }  // ← ACCOLADE MANQUANTE AJOUTÉE ICI (ferme le for(;;))
  /* USER CODE END StartTaskInferenceIA */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

