/*
 ******************************************************************************
 * @file:        max30102.h
 * @brief:       Driver I2C minimal pour le capteur PPG/SpO2 MAX30102
 *               (utilisé comme capteur de remplacement rapide au MAX86141
 *               le temps de résoudre le câblage FPC 25 broches / 0.3 mm).
 *
 *               Bus utilisé : I2C1 (déjà initialisé dans i2c.c, partagé
 *               avec le TMP117 et le SHTC3).
 *               Adresse I2C 7 bits du MAX30102 : 0x57 (fixe, non configurable).
 *
 * @datasheet:   https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf
 ******************************************************************************
 */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Adresse I2C ------------------------------------------------------------ */
#define MAX30102_I2C_ADDR            (0x57 << 1)   /* adresse 8 bits attendue par HAL_I2C_Mem_xxx */

/* Registres --------------------------------------------------------------- */
#define MAX30102_REG_INTR_STATUS_1   0x00
#define MAX30102_REG_INTR_STATUS_2   0x01
#define MAX30102_REG_INTR_ENABLE_1   0x02
#define MAX30102_REG_INTR_ENABLE_2   0x03
#define MAX30102_REG_FIFO_WR_PTR     0x04
#define MAX30102_REG_OVF_COUNTER     0x05
#define MAX30102_REG_FIFO_RD_PTR     0x06
#define MAX30102_REG_FIFO_DATA       0x07
#define MAX30102_REG_FIFO_CONFIG     0x08
#define MAX30102_REG_MODE_CONFIG     0x09
#define MAX30102_REG_SPO2_CONFIG     0x0A
#define MAX30102_REG_LED1_PA         0x0C   /* RED  */
#define MAX30102_REG_LED2_PA         0x0D   /* IR   */
#define MAX30102_REG_TEMP_INTR       0x1F
#define MAX30102_REG_TEMP_FRAC       0x20
#define MAX30102_REG_TEMP_CONFIG     0x21
#define MAX30102_REG_REV_ID          0xFE
#define MAX30102_REG_PART_ID         0xFF

#define MAX30102_EXPECTED_PART_ID    0x15

/* Bits utiles */
#define MAX30102_MODE_RESET_BIT      0x40
#define MAX30102_MODE_SPO2           0x03   /* RED + IR actifs */
#define MAX30102_INTR1_A_FULL_EN     0x80
#define MAX30102_INTR1_PPG_RDY_EN    0x40

/* Résultat brut d'un échantillon FIFO (18 bits utiles / voie) */
typedef struct {
    uint32_t red;
    uint32_t ir;
} MAX30102_Sample;

/* API publique ------------------------------------------------------------ */

/* Initialise le capteur : reset, vide la FIFO, configure FIFO/SpO2/mode/LED,
 * active les interruptions FIFO_A_FULL + PPG_RDY.
 * Retourne true si le Part ID (registre 0xFF) vaut bien 0x15. */
bool MAX30102_Init(I2C_HandleTypeDef *hi2c);

/* Lit le registre Part ID (0xFF), doit renvoyer 0x15 pour un MAX30102. */
bool MAX30102_CheckPartID(I2C_HandleTypeDef *hi2c, uint8_t *part_id);

/* Nombre d'échantillons disponibles dans la FIFO (0..32). */
uint8_t MAX30102_GetNumSamplesInFifo(I2C_HandleTypeDef *hi2c);

/* Lit UN échantillon (6 octets : 3 RED + 3 IR) depuis la FIFO. */
bool MAX30102_ReadFifoSample(I2C_HandleTypeDef *hi2c, MAX30102_Sample *sample);

/* Réinitialise les pointeurs FIFO (WR=RD=OVF=0). */
bool MAX30102_ClearFifo(I2C_HandleTypeDef *hi2c);

/* Met le capteur en veille (true) ou le réveille (false). */
bool MAX30102_Shutdown(I2C_HandleTypeDef *hi2c, bool shutdown);

/* Règle le courant des LED (0x00 = 0 mA ... 0xFF = ~51 mA, pas de 0.2 mA). */
bool MAX30102_SetLedCurrent(I2C_HandleTypeDef *hi2c, uint8_t red_pa, uint8_t ir_pa);

/* Efface les flags d'interruption en lisant INTR_STATUS_1/2 (obligatoire
 * après une interruption pour désasserter la broche INT, active basse). */
bool MAX30102_ClearInterruptFlags(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAX30102_H_ */
