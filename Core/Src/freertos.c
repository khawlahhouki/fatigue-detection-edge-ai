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
#include <lsm6dso_imu.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include  <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max86141.h"
#include "ads1292r.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "string.h"
#include "lsm6dso_imu.h"
#include "tmp117.h"    /* ← doit être présent */
#include "shtc3.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PD */
/****************************** Elle regroupe des données de 3 sources
   → TMP117 + SHTC3 + TIM2*******************************************/
/* Taille = 4+4+4+4 = 16 octets
 * cohérent avec Q_TEMPHandle configuré 10 × 16 octets */
typedef struct {
    float    temp_skin_c;     /* TMP117 — température peau en °C    */
    float    temp_ambient_c;  /* SHTC3  — température ambiante °C   */
    float    humidity_pct;    /* SHTC3  — humidité relative en %    */
    uint32_t timestamp;       /* TIM2   — instant mesure en µs      */
} Temp_Sample_t;

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

SemaphoreHandle_t sem_dma_ppg = NULL;


/* pour calculer le temps de chaque sample */
extern volatile uint32_t ppg_isr_timestamp;
extern volatile uint32_t emg_isr_timestamp;
extern volatile uint32_t Imu_isr_timestamp;
extern uint8_t ADS1292R_data_buf[9];
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef  htim2;




/* ajouté pour la tache IMU */







/* USER CODE END Variables */
/* Definitions for vTaskE ADS1MGAcq */
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskEMG(void *argument);
void StartTaskPPG(void *argument);
void StartTaskIMU(void *argument);
void StartTaskTemp(void *argument);
void StartTaskDSP(void *argument);

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
  Q_TEMPHandle = osMessageQueueNew (10, 16, &Q_TEMP_attributes);

  /* creation of Q_RESULTS */
  Q_RESULTSHandle = osMessageQueueNew (20, 64, &Q_RESULTS_attributes);

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


  /* SB_EMG et SB_IMU — trigger = 1 octet pour l'instant
     on ajustera quand vTaskDSP sera codé */
  SB_EMG = xStreamBufferCreate(512, 1);
  SB_IMU = xStreamBufferCreate(512, 1);

  if(SB_PPG == NULL || SB_EMG == NULL || SB_IMU == NULL)
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
	  ADS1292R_Sample_t emg_sample;
	  uint32_t trigger_time_emg = 0;

	  // 1. Initialisation et démarrage du flux continu (RDATAC)
	  if(xSemaphoreTake(MTX_SPI2Handle, portMAX_DELAY) == pdTRUE) {
	      ADS1292R_ADCStartNormal();
	      xSemaphoreGive(MTX_SPI2Handle);
 }

  for(;;)
  {

	    // 2. Attendre l'interruption DRDY (PE12)
	    // Timeout 10ms (à 1000Hz, DRDY arrive toutes les 1ms)
	    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)) > 0)
	    {
	      // 3. Capturer le timestamp immédiat pris dans l'ISR
	      trigger_time_emg = emg_isr_timestamp;

	      // 4. Prendre le Mutex SPI2 (priorité haute pour ne pas rater le sample)
	      if(xSemaphoreTake(MTX_SPI2Handle, pdMS_TO_TICKS(1)) == pdTRUE)
	      {
	        // 5. Lire les 9 octets (Polling rapide)
	        ADS1292R_GetValue();
	        xSemaphoreGive(MTX_SPI2Handle);

	        // 6. Décoder les données brutes
	        ADS1292R_Decode(ADS1292R_data_buf, &emg_sample, trigger_time_emg);

	        // 7. Envoyer vers le StreamBuffer pour la tâche DSP / IA
	        if(SB_EMG != NULL) {
	            xStreamBufferSend(SB_EMG, &emg_sample, sizeof(ADS1292R_Sample_t), 0);
	        }
	      }
	    }
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
	/* ---------------------------------------------------------------
	 * Buffers SPI pour la lecture FIFO du MAX86141
	 *
	 * STATIC LOCAL — deux raisons :
	 *   1. DURÉE DE VIE : ces buffers doivent exister en permanence.
	 *      Le driver SPI DMA travaille dessus de manière asynchrone.
	 *      Sans static, ils seraient sur le stack et risqueraient
	 *      d'être écrasés ou détruits avant la fin du DMA.
	 *
	 *   2. TAILLE STACK : 385 + 385 = 770 octets.
	 *      Le stack de cette tâche est 512 words = 2048 octets.
	 *      Mettre 770 octets sur le stack laisserait peu de marge
	 *      → risque de stack overflow.
	 *      Avec static, ces buffers vont en zone RAM statique (BSS),
	 *      hors du stack, sans risque.
	 *
	 * TAILLE 385 octets = 1 octet commande + 64 paires * 6 octets
	 *   - 1 octet  : adresse registre FIFO_DATA (0x88 = READ | 0x08)
	 *   - 64 paires * 3 octets IR  = 192 octets
	 *   - 64 paires * 3 octets RED = 192 octets
	 *   - Total données            = 384 octets
	 * --------------------------------------------------------------- */

	  static uint8_t ppg_dummy_tx[1 + MAX86141_FIFO_MAX_SAMPLES * 6];
	  static uint8_t ppg_rx_buf [1 + MAX86141_FIFO_MAX_SAMPLES * 6];

	  MAX86141_Sample_t samples[MAX86141_FIFO_MAX_SAMPLES];
	  uint8_t count = 0;

	  memset(ppg_dummy_tx, 0x00, sizeof(ppg_dummy_tx));
	  /* metre des 0 dans tous les tx[]*/
	  ppg_dummy_tx[0] = MAX86141_SPI_READ | MAX86141_REG_FIFO_DATA;
	  uint32_t trigger_time = 0;


	  if(xSemaphoreTake(MTX_SPI2Handle,  portMAX_DELAY) == pdTRUE)
	   {
	       // Commande pour sortir du mode Shutdown (0x00 = Actif)
	       MAX86141_WriteReg(&hspi2, CS_PPG_GPIO_Port, CS_PPG_Pin, MAX86141_REG_SYS_CTRL, 0x00);
	       xSemaphoreGive(MTX_SPI2Handle);
	   }


	  for(;;)
	  {
	    /* Étape 1 — dormir maximum 150ms jusqu'à notification ISR PE11
	     * il faut 160ms pour perdre un fifo de 64 samples donc en a choisir de se bloquer au maximum 150ms */

		  if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(150)) > 0)
	    {
	    /* Étape 2 — prendre le mutex SPI2 */
	    if(xSemaphoreTake(MTX_SPI2Handle, pdMS_TO_TICKS(2)) == pdTRUE)
	    {
	    	/*
	    	   * On ne lit plus le TIM2 ici !
	    	   * On récupère la "photo" du temps prise par l'interruption.
	    	   */
	    	trigger_time = ppg_isr_timestamp;

	      /* Étape 3 — lire nombre de paires disponibles */
	      count = MAX86141_GetFIFOCount(&hspi2, CS_PPG_GPIO_Port, CS_PPG_Pin);

	      if(count > 0)
	      {
	        /* Étape 4 — CS_LOW · lancer le DMA · CPU libre */
	        HAL_GPIO_WritePin(CS_PPG_GPIO_Port, CS_PPG_Pin, GPIO_PIN_RESET);

	        HAL_SPI_TransmitReceive_DMA(&hspi2,
	                                     ppg_dummy_tx,
	                                     ppg_rx_buf,
	                                     count * 6 + 1);

	        /* Attendre DMA — Sémaphore binaire SÉPARÉ */
	              xSemaphoreTake(sem_dma_ppg, pdMS_TO_TICKS(50));
	      }

	      /* Étape 6 — libérer le mutex */
	      xSemaphoreGive(MTX_SPI2Handle);

	      /* Étape 7 — décoder et envoyer dans SB_PPG */
	     /*C'est pour l'efficacité (optimisation) :
		   Si le capteur n'a aucune nouvelle donnée (count = 0),
		   il est inutile d'appeler la fonction xStreamBufferSend*/



	      /* SB_PPG est NULL, le processeur va tenter d'écrire des données à l'adresse 0x00000000.
             Sur un STM32H7, cela provoque immédiatement une HardFault
             (le processeur s'arrête net et notre appareil médical plante).*/

	      if(count > 0 && SB_PPG != NULL)
	      {
	        MAX86141_DecodeFIFO(ppg_rx_buf, samples, count,trigger_time);
	        xStreamBufferSend(SB_PPG,
	                          samples,
	                          count * sizeof(MAX86141_Sample_t),
	                          0);
	      }
	    }
	  }
	  }

  }
  /* USER CODE END StartTaskPPG */


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

	IMU_Sample_t imu_sample;


	if (xSemaphoreTake(MTX_SPI2Handle, pdMS_TO_TICKS(100)) == pdTRUE)
	  {
	      LSM6DSO_IMU_Start();
	      xSemaphoreGive(MTX_SPI2Handle);
	  }

	  /* Imu_isr_timestamp déclaré dans freertos.c
	   * capturé dans HAL_GPIO_EXTI_Callback        */


	  for(;;)
	  {
		/*******************************************************************/
		  /*ODR = 104 Hz → 1 sample toutes les 9.6ms
		  XL + GY batchés → 2 slots par période = 208 slots/seconde

		  Watermark = 20 slots
		  Temps pour remplir 20 slots = 20 / 208 = ~96ms

		  → INT1 se déclenche toutes les ~96ms
	    /* --- Attendre INT1_IMU (PE13) — watermark FIFO atteint
	     * Timeout 200ms : capteur silencieux → on retourne attendre */
	    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200)) != 0)
	    {
	      /* Sauvegarder le timestamp capturé dans l'ISR
	       * C'est l'instant réel du déclenchement INT1   */
	      uint32_t sample_timestamp = Imu_isr_timestamp;

	      /* --- Prendre le mutex SPI2
	       * SPI2 partagé entre MAX86141, ADS1292R, LSM6DSO
	       * Timeout 50ms : bus occupé → on abandonne      */
	      if (xSemaphoreTake(MTX_SPI2Handle, pdMS_TO_TICKS(50)) == pdTRUE)
	      {
	        /* --- Lire et décoder le FIFO
	         * ReadFIFO lit tous les slots, décode XL et GY,
	         * calcule la moyenne et remplit imu_sample     */
	        int32_t ret = LSM6DSO_IMU_ReadFIFO(&imu_sample);

	        /* --- Libérer le mutex dès que la lecture est terminée */
	        xSemaphoreGive(MTX_SPI2Handle);

	        /* --- Appliquer le timestamp ISR et envoyer --- */
	        if (ret == LSM6DSO_IMU_OK && SB_IMU != NULL)
	        {
	          /* Timestamp = moment réel de l'interruption
	           * pas le moment de la lecture (qui peut être
	           * retardée par le mutex)                     */
	          imu_sample.timestamp = sample_timestamp;

	          xStreamBufferSend(SB_IMU,
	                            &imu_sample,
	                            sizeof(IMU_Sample_t),
	                            0);
	        }
	      }
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
	  /* ---------------------------------------------------------------
	   * Buffer I2C pour le driver TMP117
	   * static : permanent, hors stack, privé à cette tâche
	   * Taille 4 : writeReg2B utilise 3 octets max + 1 marge
	   * --------------------------------------------------------------- */
	  static uint8_t tmp117_buf[4];

	  /* Résultat TMP117 en double
	   * Le driver impose double (résolution 0.0078125°C)
	   * Converti en float pour Temp_Sample_t               */
	  double skin_temp_d = 0.0;

	  /* Résultats SHTC3 en int32_t × 100
	   * temp : 2410  = 24.10°C
	   * hum  : 65    = 65%                                 */
	  int32_t shtc3_temp_raw = 0;
	  int32_t shtc3_hum_raw  = 0;

	  /* Flags de résultat */
	  bool tmp_ok = false;
	  uint32_t shtc_ok = 0;

	  /* Timestamps séparés pour chaque capteur */
	  uint32_t tmp117_timestamp = 0;
	  uint32_t shtc3_timestamp  = 0;

	  /* Structure de sortie vers vTaskDSP */
	  Temp_Sample_t temp_sample;

	  for(;;)
	  {
	    /* -----------------------------------------------------------
	     * ÉTAPE 1 — Attendre 250ms
	     *
	     * Pas d'interruption pour la température :
	     *   → pas de FIFO qui déborde
	     *   → température évolue en secondes pas en ms
	     *   → osDelay suffit largement
	     *
	     * osDelay libère le CPU pour les autres tâches
	     * contrairement à HAL_Delay (busy wait)
	     * ----------------------------------------------------------- */
	    osDelay(250);

	    /* -----------------------------------------------------------
	     * ÉTAPE 2 — Prendre le mutex I2C1
	     *
	     * Pour envoyer la commande One-Shot au TMP117.
	     * On libère juste après car la conversion dure
	     * 124ms sans utiliser le bus I2C.
	     * Timeout 100ms : I2C1 normalement jamais occupé longtemps.
	     * ----------------------------------------------------------- */
	    if (xSemaphoreTake(MTX_I2C1Handle, pdMS_TO_TICKS(100)) == pdTRUE)
	    {

	    	/* Lancer TMP117 One-Shot */
	    	setConversionMode(&hi2c1, tmp117_buf, TMP117_OS_MODE);

	    	/* Lancer SHTC3 pendant la même fenêtre */
	    	shtc3_wakeup(&hi2c1);

	    	xSemaphoreGive(MTX_I2C1Handle);
	    	osDelay(1);  /* attendre 240µs SHTC3 wakeup */

	    	/* Les deux capteurs "travaillent" maintenant */
	    	/* TMP117 : conversion interne ~124ms         */
	    	/* SHTC3  : prêt en IDLE, attend commande     */

	    	osDelay(129); /* attendre fin TMP117 (130ms - 1ms déjà attendu) */
	    	tmp117_timestamp = __HAL_TIM_GET_COUNTER(&htim2);
	    	if (xSemaphoreTake(MTX_I2C1Handle, pdMS_TO_TICKS(100)) == pdTRUE)
	    	{
	    	    /* Lire TMP117 */

	    	    tmp_ok = getResultTemperature(&hi2c1, tmp117_buf, &skin_temp_d);

	    	    /* Lire SHTC3 immédiatement après
	    	     * Les deux mesures sont maintenant séparées de ~1ms
	    	     * au lieu de ~14ms                                  */
	    	    shtc_ok = shtc3_perform_measurements(&hi2c1,
	    	                                          &shtc3_temp_raw,
	    	                                          &shtc3_hum_raw);
	    	    shtc3_timestamp = __HAL_TIM_GET_COUNTER(&htim2);

	    	    shtc3_sleep(&hi2c1);
	    	    xSemaphoreGive(MTX_I2C1Handle);
	    	}



	        /* -------------------------------------------------------
	         * ÉTAPE 14 — Convertir et envoyer si les deux OK
	         *
	         * tmp_ok  : bool     → true  = TMP117 lu correctement
	         * shtc_ok : uint32_t → 1     = SHTC3 CRC OK
	         *
	         * Si un seul échoue → on abandonne ce cycle
	         * Prochain cycle dans 250ms → acceptable
	         * ------------------------------------------------------- */
	        if (tmp_ok && (shtc_ok == 1))
	        {
	          /* TMP117 : double → float
	           * float = 7 chiffres significatifs
	           * largement suffisant pour 0.0078°C résolution  */
	          temp_sample.temp_skin_c = (float)skin_temp_d;

	          /* SHTC3 température : centièmes d'°C → °C
	           * 2410 / 100.0f = 24.10°C
	           * 100.0f obligatoire → sinon division entière
	           * 2410 / 100 = 24 (perte des décimales !)       */
	          temp_sample.temp_ambient_c = (float)shtc3_temp_raw / 100.0f;

	          /* SHTC3 humidité : déjà en %
	           * 65 → 65.0%                                    */
	          temp_sample.humidity_pct = (float)shtc3_hum_raw;

	          /* Timestamp TMP117 comme référence principale
	           * Signal médical le plus important
	           * shtc3_timestamp disponible si vTaskDSP en a
	           * besoin séparément                             */
	          temp_sample.timestamp = tmp117_timestamp;

	          /* Envoyer vers vTaskDSP via Q_TEMPHandle
	           * osMessageQueuePut COPIE les données
	           * timeout 0 = non bloquant
	           * Si queue pleine → données perdues
	           * acceptable : prochaine mesure dans 250ms      */
	          osMessageQueuePut(Q_TEMPHandle,
	                            &temp_sample,
	                            0,
	                            0);
	        }
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskDSP */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

