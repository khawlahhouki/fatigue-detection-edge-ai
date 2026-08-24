/*
 ******************************************************************************
 * @file:    max30102.c
 * @brief:   Driver I2C minimal pour le MAX30102 (voir max30102.h)
 ******************************************************************************
 */

#include "max30102.h"

#define I2C_TIMEOUT_MS  100

/* ---- helpers bas niveau ------------------------------------------------ */

static bool MAX30102_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool MAX30102_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, MAX30102_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool MAX30102_ReadBurst(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MAX30102_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, I2C_TIMEOUT_MS) == HAL_OK;
}

/* ---- API publique -------------------------------------------------------- */

bool MAX30102_CheckPartID(I2C_HandleTypeDef *hi2c, uint8_t *part_id)
{
    return MAX30102_ReadReg(hi2c, MAX30102_REG_PART_ID, part_id);
}

bool MAX30102_ClearFifo(I2C_HandleTypeDef *hi2c)
{
    bool ok = true;
    ok &= MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_WR_PTR, 0x00);
    ok &= MAX30102_WriteReg(hi2c, MAX30102_REG_OVF_COUNTER, 0x00);
    ok &= MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_RD_PTR, 0x00);
    return ok;
}

bool MAX30102_Shutdown(I2C_HandleTypeDef *hi2c, bool shutdown)
{
    uint8_t mode = shutdown ? 0x80 : MAX30102_MODE_SPO2;
    return MAX30102_WriteReg(hi2c, MAX30102_REG_MODE_CONFIG, mode);
}

bool MAX30102_SetLedCurrent(I2C_HandleTypeDef *hi2c, uint8_t red_pa, uint8_t ir_pa)
{
    bool ok = true;
    ok &= MAX30102_WriteReg(hi2c, MAX30102_REG_LED1_PA, red_pa);
    ok &= MAX30102_WriteReg(hi2c, MAX30102_REG_LED2_PA, ir_pa);
    return ok;
}

bool MAX30102_ClearInterruptFlags(I2C_HandleTypeDef *hi2c)
{
    uint8_t s1, s2;
    bool ok = true;
    ok &= MAX30102_ReadReg(hi2c, MAX30102_REG_INTR_STATUS_1, &s1);
    ok &= MAX30102_ReadReg(hi2c, MAX30102_REG_INTR_STATUS_2, &s2);
    return ok;
}

bool MAX30102_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t part_id = 0;

    /* 1) Reset logiciel (bit RESET du MODE_CONFIG), auto-effacé par le capteur */
    if (!MAX30102_WriteReg(hi2c, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET_BIT))
        return false;

    HAL_Delay(10); /* le reset prend quelques ms, on laisse une marge */

    /* 2) Vérification de l'identité du capteur (Part ID = 0x15) */
    if (!MAX30102_CheckPartID(hi2c, &part_id) || part_id != MAX30102_EXPECTED_PART_ID)
        return false;

    /* 3) FIFO : PAS de moyenne matérielle (SMP_AVE=0), pas de rollover,
     * presque-plein = 17 échantillons restants (FIFO_A_FULL=0xF -> 32-15=17).
     * IMPORTANT : contrairement au prototype de test (0x4F, moyenne x4),
     * ici SMP_AVE=0 est OBLIGATOIRE pour obtenir le vrai débit configuré
     * en SPO2_CONFIG (400 sps) en sortie de FIFO. Avec une moyenne
     * matérielle x4, le débit réel en FIFO retomberait à 100 sps malgré
     * la config SR=400, ce qui casserait toutes les hypothèses temporelles
     * du pipeline DSP existant (filtre passe-bande, NLMS, interpolation IMU). */
    if (!MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_CONFIG, 0x0F))
        return false;

    if (!MAX30102_ClearFifo(hi2c))
        return false;

    /* 4) Mode SpO2 : LED RED + LED IR actives */
    if (!MAX30102_WriteReg(hi2c, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2))
        return false;

    /* 5) SpO2 config : plage ADC 4096 nA, 400 échantillons/s, impulsion 411 us
     * (résolution 18 bits -- ce point est valide car le datasheet autorise
     * jusqu'à 800 sps avec une impulsion de 411 us, donc 400 sps tient
     * largement). 400 sps choisi pour matcher le débit du MAX86141 sur
     * lequel tout le pipeline DSP existant (StartTaskDSP) a été calé. */
    if (!MAX30102_WriteReg(hi2c, MAX30102_REG_SPO2_CONFIG, 0x2F))
        return false;

    /* 6) Courant LED de départ (~7.4 mA), à ajuster selon le point de mesure
     * (doigt vs poignet -- voir tests de validation) */
    if (!MAX30102_SetLedCurrent(hi2c, 0x24, 0x24))
        return false;

    /* 7) Interruptions : FIFO presque pleine + nouvel échantillon prêt */
    if (!MAX30102_WriteReg(hi2c, MAX30102_REG_INTR_ENABLE_1,
                            MAX30102_INTR1_A_FULL_EN | MAX30102_INTR1_PPG_RDY_EN))
        return false;

    /* Lire les registres de statut efface les flags résiduels et libère la broche INT (active basse) */
    MAX30102_ClearInterruptFlags(hi2c);

    return true;
}

uint8_t MAX30102_GetNumSamplesInFifo(I2C_HandleTypeDef *hi2c)
{
    uint8_t wr_ptr = 0, rd_ptr = 0;

    if (!MAX30102_ReadReg(hi2c, MAX30102_REG_FIFO_WR_PTR, &wr_ptr))
        return 0;
    if (!MAX30102_ReadReg(hi2c, MAX30102_REG_FIFO_RD_PTR, &rd_ptr))
        return 0;

    int16_t n = (int16_t)wr_ptr - (int16_t)rd_ptr;
    if (n < 0)
        n += 32; /* FIFO circulaire de 32 échantillons */

    return (uint8_t)n;
}

bool MAX30102_ReadFifoSample(I2C_HandleTypeDef *hi2c, MAX30102_Sample *sample)
{
    uint8_t buf[6];

    if (!MAX30102_ReadBurst(hi2c, MAX30102_REG_FIFO_DATA, buf, sizeof(buf)))
        return false;

    /* Chaque voie fait 3 octets, seuls les 18 bits de poids faible sont valides */
    sample->red = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]) & 0x3FFFF;
    sample->ir  = (((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5]) & 0x3FFFF;

    return true;
}
