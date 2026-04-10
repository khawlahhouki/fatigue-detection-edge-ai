/* lsm6dso_imu.c */
#include "lsm6dso_imu.h"
#include <string.h>

/* ================================================================
 * VARIABLES PRIVÉES
 * ================================================================ */

/* Contexte du driver ST — contient les callbacks read/write */

stmdev_ctx_t lsm6dso_ctx;



/* Mémorisés ici par Init() pour que les callbacks
 * SPI puissent les utiliser plus tard.
 * static = ils restent en mémoire pour toujours
 *        = personne d'autre ne peut y toucher      */
/* Handle SPI et broche CS — stockés pour les callbacks */




static SPI_HandleTypeDef *_hspi;
static GPIO_TypeDef      *_cs_port;
static uint16_t           _cs_pin;

/* ================================================================
 * CALLBACKS SPI — C'EST LE CŒUR DU BRIDGE
 *
 * Le driver ST appelle ctx.read_reg() et ctx.write_reg()
 * Ces fonctions doivent respecter le prototype :
 *   int32_t func(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
 *
 * Protocole SPI LSM6DSO :
 *   - Lecture  : bit7 du registre = 1  (0x80 | reg_addr)
 *   - Écriture : bit7 du registre = 0  (reg_addr tel quel)
 *   - CS actif bas pendant toute la transaction
 * ================================================================ */

static int32_t LSM6DSO_SPI_Write(void *handle, uint8_t reg,
                                  uint8_t *data, uint16_t len)
{
    (void)handle; /* non utilisé, on utilise les variables statiques */

    HAL_StatusTypeDef status;

    /* CS bas */
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);

    /* Envoyer l'adresse du registre (bit7=0 = écriture) */
    status = HAL_SPI_Transmit(_hspi, &reg, 1, 10);

    if (status == HAL_OK) {
        /* Envoyer les données */
        status = HAL_SPI_Transmit(_hspi, data, len, 10);
    }

    /* CS haut */
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK) ? LSM6DSO_IMU_OK : LSM6DSO_IMU_ERR;
}

static int32_t LSM6DSO_SPI_Read(void *handle, uint8_t reg,
                                 uint8_t *data, uint16_t len)
{
    (void)handle;

    HAL_StatusTypeDef status;

    /* Bit7 = 1 pour signaler une LECTURE */
    uint8_t reg_read = reg | 0x80U;

    /* CS bas */
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);

    /* Envoyer l'adresse */
    status = HAL_SPI_Transmit(_hspi, &reg_read, 1, 10);

    if (status == HAL_OK) {
        /* Recevoir les données */
        status = HAL_SPI_Receive(_hspi, data, len, 10);
    }

    /* CS haut */
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK) ? LSM6DSO_IMU_OK : LSM6DSO_IMU_ERR;
}

/* ================================================================
 * INITIALISATION
 * ================================================================ */

int32_t LSM6DSO_IMU_Init(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin)
{
    _hspi    = hspi;
    _cs_port = cs_port;
    _cs_pin  = cs_pin;

    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);

    lsm6dso_ctx.write_reg = LSM6DSO_SPI_Write;
    lsm6dso_ctx.read_reg  = LSM6DSO_SPI_Read;
    lsm6dso_ctx.handle    = NULL;

    HAL_Delay(10);

    /* WHO_AM_I */
    uint8_t who_am_i = 0;
    if (lsm6dso_device_id_get(&lsm6dso_ctx, &who_am_i) != 0)
        return LSM6DSO_IMU_ERR;
    if (who_am_i != LSM6DSO_WHOAMI)
        return LSM6DSO_IMU_ERR;

    /* Reset */
    lsm6dso_reset_set(&lsm6dso_ctx, PROPERTY_ENABLE);
    uint8_t rst = 1;
    uint32_t timeout = 100;
    while (rst && timeout--) {
        lsm6dso_reset_get(&lsm6dso_ctx, &rst);
        HAL_Delay(1);
    }
    if (rst) return LSM6DSO_IMU_ERR;

    /* BDU et auto-incrément */
    lsm6dso_block_data_update_set(&lsm6dso_ctx, PROPERTY_ENABLE);
    lsm6dso_auto_increment_set(&lsm6dso_ctx, PROPERTY_ENABLE);

    /* -------------------------------------------------------
     * Configuration SANS démarrage
     *
     * Full-scale et FIFO configurés MAIS :
     *   ODR = OFF    → pas de mesure
     *   FIFO = BYPASS → pas de stockage
     *   INT1 = rien  → pas d'interruption
     *
     * Le capteur est prêt mais silencieux.
     * LSM6DSO_IMU_Start() démarrera tout.
     * ------------------------------------------------------- */

    /* Accéléromètre — full scale uniquement, ODR = OFF */
    lsm6dso_xl_full_scale_set(&lsm6dso_ctx, LSM6DSO_4g);
    lsm6dso_xl_data_rate_set(&lsm6dso_ctx, LSM6DSO_XL_ODR_OFF);
    /* ODR_OFF = 0x00 → capteur en power-down             */

    /* Gyroscope — full scale uniquement, ODR = OFF */
    lsm6dso_gy_full_scale_set(&lsm6dso_ctx, LSM6DSO_2000dps);
    lsm6dso_gy_data_rate_set(&lsm6dso_ctx, LSM6DSO_GY_ODR_OFF);

    /* FIFO — watermark configuré MAIS mode BYPASS
     * BYPASS = FIFO désactivé → rien ne s'accumule    */
    lsm6dso_fifo_watermark_set(&lsm6dso_ctx, 20U);
    lsm6dso_fifo_xl_batch_set(&lsm6dso_ctx, LSM6DSO_XL_NOT_BATCHED);
    lsm6dso_fifo_gy_batch_set(&lsm6dso_ctx, LSM6DSO_GY_NOT_BATCHED);
    lsm6dso_fifo_mode_set(&lsm6dso_ctx, LSM6DSO_BYPASS_MODE);
    /* BYPASS_MODE → FIFO vide et inactif               */

    /* INT1 — aucune source activée
     * Pas d'interruption tant que Start() n'est pas appelé */
    lsm6dso_pin_int1_route_t int1_route;
    memset(&int1_route, 0, sizeof(int1_route));
    /* int1_route.fifo_th = 0 → INT1 silencieux          */
    lsm6dso_pin_int1_route_set(&lsm6dso_ctx, int1_route);

    return LSM6DSO_IMU_OK;
}
int32_t LSM6DSO_IMU_Start(void)
{
    /* -------------------------------------------------------
     * Démarre les mesures et les interruptions
     *
     * Appelé depuis StartTaskIMU, donc :
     *   → FreeRTOS est démarré
     *   → vTaskIMUAcqHandle est valide
     *   → les notifications peuvent être reçues
     * ------------------------------------------------------- */

    /* Démarrer l'accéléromètre à 104 Hz */
    lsm6dso_xl_data_rate_set(&lsm6dso_ctx, LSM6DSO_XL_ODR_104Hz);

    /* Démarrer le gyroscope à 104 Hz */
    lsm6dso_gy_data_rate_set(&lsm6dso_ctx, LSM6DSO_GY_ODR_104Hz);

    /* Activer le batch dans le FIFO */
    lsm6dso_fifo_xl_batch_set(&lsm6dso_ctx, LSM6DSO_XL_BATCHED_AT_104Hz);
    lsm6dso_fifo_gy_batch_set(&lsm6dso_ctx, LSM6DSO_GY_BATCHED_AT_104Hz);

    /* Passer en mode STREAM — FIFO circulaire actif */
    lsm6dso_fifo_mode_set(&lsm6dso_ctx, LSM6DSO_STREAM_MODE);

    /* Activer l'interruption FIFO watermark sur INT1
     * À partir de maintenant INT1 peut se déclencher
     * et réveiller vTaskIMUAcq                        */
    lsm6dso_pin_int1_route_t int1_route;
    memset(&int1_route, 0, sizeof(int1_route));
    int1_route.fifo_th = PROPERTY_ENABLE;
    lsm6dso_pin_int1_route_set(&lsm6dso_ctx, int1_route);

    return LSM6DSO_IMU_OK;
}
/* ================================================================
 * LECTURE D'UN SAMPLE (utilisé en mode polling dans vTaskIMUAcq)
 *
 * Lit les registres de sortie directs (pas le FIFO).
 * Retourne accel en mg et gyro en mdps.
 * ================================================================ */

int32_t LSM6DSO_IMU_ReadSample(LSM6DSO_Sample_t *sample)
{
    if (sample == NULL) return LSM6DSO_IMU_ERR;

    /* Lecture brute accéléromètre — 6 octets consécutifs */
    if (lsm6dso_acceleration_raw_get(&lsm6dso_ctx, sample->accel_raw) != 0) {
        return LSM6DSO_IMU_ERR;
    }

    /* Lecture brute gyroscope — 6 octets consécutifs */
    if (lsm6dso_angular_rate_raw_get(&lsm6dso_ctx, sample->gyro_raw) != 0) {
        return LSM6DSO_IMU_ERR;
    }

    /* Conversion en unités physiques
     *
     * Sensibilité accéléromètre à ±4g = 0.122 mg/LSB
     * Sensibilité gyroscope à ±2000dps = 70.0 mdps/LSB */
    for (int i = 0; i < 3; i++) {
        sample->accel_mg[i]  = lsm6dso_from_fs4_to_mg(sample->accel_raw[i]);
        sample->gyro_mdps[i] = lsm6dso_from_fs2000_to_mdps(sample->gyro_raw[i]);
    }

    return LSM6DSO_IMU_OK;
}
/* ================================================================
 * LSM6DSO_IMU_GetFIFOLevel
 *
 * Lit le nombre de slots disponibles dans le FIFO.
 * Équivalent de MAX86141_GetFIFOCount().
 *
 * Paramètre :
 *   level → pointeur vers uint16_t qui recevra le count
 *
 * Retour :
 *   LSM6DSO_IMU_OK  (0)  = succès
 *   LSM6DSO_IMU_ERR (-1) = échec lecture SPI
 * ================================================================ */
int32_t LSM6DSO_IMU_GetFIFOLevel(uint16_t *level)
{
    if (level == NULL) return LSM6DSO_IMU_ERR;

    /* Lit FIFO_STATUS1 (0x3A) et FIFO_STATUS2 (0x3B)
     * Résultat sur 10 bits → max 511 slots             */
    if (lsm6dso_fifo_data_level_get(&lsm6dso_ctx, level) != 0)
    {
        return LSM6DSO_IMU_ERR;
    }

    return LSM6DSO_IMU_OK;
}


/* ================================================================
 * LSM6DSO_IMU_ReadFIFO
 *
 * Vide complètement le FIFO, décode les slots XL et GY,
 * calcule la moyenne du batch et remplit la structure IMU_Sample_t.
 *
 * Équivalent de MAX86141_ReadFIFO() + MAX86141_DecodeFIFO().
 *
 * Paramètre :
 *   sample → pointeur vers la structure à remplir
 *
 * Retour :
 *   LSM6DSO_IMU_OK  = succès, sample est rempli
 *   LSM6DSO_IMU_ERR = échec ou FIFO vide
 * ================================================================ */
int32_t LSM6DSO_IMU_ReadFIFO(IMU_Sample_t *sample)
{
    if (sample == NULL) return LSM6DSO_IMU_ERR;

    uint16_t           fifo_level = 0;
    uint8_t            fifo_raw[6];
    lsm6dso_fifo_tag_t tag;
    int16_t            raw[3];

    /* Accumulateurs pour moyenner le batch */
    float    accel_acc[3] = {0.0f, 0.0f, 0.0f};
    float    gyro_acc[3]  = {0.0f, 0.0f, 0.0f};
    uint16_t xl_count     = 0;
    uint16_t gy_count     = 0;

    /* --- Lire le nombre de slots disponibles --- */
    if (lsm6dso_fifo_data_level_get(&lsm6dso_ctx, &fifo_level) != 0)
    {
        return LSM6DSO_IMU_ERR;
    }

    if (fifo_level == 0) return LSM6DSO_IMU_ERR;

    /* --- Vider le FIFO slot par slot --- */
    for (uint16_t i = 0; i < fifo_level; i++)
    {
        /* Lire le TAG : identifie le type du slot courant
         * (gyroscope, accéléromètre, température...) */
        lsm6dso_fifo_sensor_tag_get(&lsm6dso_ctx, &tag);

        /* Lire les 6 octets de données brutes du slot */
        lsm6dso_fifo_out_raw_get(&lsm6dso_ctx, fifo_raw);

        /* Reconstruire les int16_t — LSM6DSO envoie little-endian
         * fifo_raw[0]=LSB X, fifo_raw[1]=MSB X, etc. */
        raw[0] = (int16_t)((fifo_raw[1] << 8) | fifo_raw[0]);
        raw[1] = (int16_t)((fifo_raw[3] << 8) | fifo_raw[2]);
        raw[2] = (int16_t)((fifo_raw[5] << 8) | fifo_raw[4]);

        if (tag == LSM6DSO_XL_NC_TAG)
        {
            /* Accéléromètre — convertir en mg (±4g = 0.122 mg/LSB) */
            accel_acc[0] += lsm6dso_from_fs4_to_mg(raw[0]);
            accel_acc[1] += lsm6dso_from_fs4_to_mg(raw[1]);
            accel_acc[2] += lsm6dso_from_fs4_to_mg(raw[2]);
            xl_count++;
        }
        else if (tag == LSM6DSO_GYRO_NC_TAG)
        {
            /* Gyroscope — convertir en mdps (±2000dps = 70.0 mdps/LSB) */
            gyro_acc[0] += lsm6dso_from_fs2000_to_mdps(raw[0]);
            gyro_acc[1] += lsm6dso_from_fs2000_to_mdps(raw[1]);
            gyro_acc[2] += lsm6dso_from_fs2000_to_mdps(raw[2]);
            gy_count++;
        }
        /* Autres tags ignorés (TIMESTAMP, CFG_CHANGE...) */
    }

    /* --- Vérifier qu'on a des données valides --- */
    if (xl_count == 0 || gy_count == 0) return LSM6DSO_IMU_ERR;

    /* --- Calculer la moyenne du batch --- */
    sample->accel_mg[0] = accel_acc[0] / (float)xl_count;
    sample->accel_mg[1] = accel_acc[1] / (float)xl_count;
    sample->accel_mg[2] = accel_acc[2] / (float)xl_count;

    sample->gyro_mdps[0] = gyro_acc[0] / (float)gy_count;
    sample->gyro_mdps[1] = gyro_acc[1] / (float)gy_count;
    sample->gyro_mdps[2] = gyro_acc[2] / (float)gy_count;

    /* Timestamp rempli par la tâche (accès TIM2 = couche HAL/app) */
    sample->timestamp = 0;

    return LSM6DSO_IMU_OK;
}
