#ifndef __ADS1292R_H
#define __ADS1292R_H

/*
 * ===========================================================================
 * Fichier  : ads1292r.h
 * Capteur  : ADS1292R - Capteur ECG/EMG 2 canaux 24 bits
 * Auteur   : Etudiante - Projet Biosignal STM32H7A3
 * Date     : 2026
 *
 * Description :
 *   Driver adapté pour STM32H7A3 HAL SPI2.
 *   Basé sur le projet sjZhu1020/STM32-HAL-ADS1292R (STM32F407)
 *   Modifications :
 *     - hspi1 → hspi2
 *     - Suppression dépendance hmi_uart.h
 *     - Correction boucle infinie dans Init() → timeout 5 tentatives
 *     - Broches GPIO adaptées pour Nucleo-H7A3
 *
 * Broches utilisées :
 *   CS    = PG10  (CS_EMG  - défini dans main.h par CubeMX)
 *   RESET = PE14  (à configurer dans CubeMX comme GPIO Output)
 *   START = PE15  (à configurer dans CubeMX comme GPIO Output)
 *   DRDY  = PE12  (EXTI12 - déjà configuré dans notre architecture)
 *
 * Référence : ADS1292R Datasheet Rev.C - Texas Instruments SBAS502C
 * ===========================================================================
 */

#include "main.h"   /* contient toutes les définitions HAL et GPIO */
#include "spi.h"    /* contient extern SPI_HandleTypeDef hspi2 */

/* ---------------------------------------------------------------------------
 * MACROS GPIO — Chip Select · Reset · Start
 * Les noms de pins doivent correspondre à ceux générés par CubeMX dans main.h
 * --------------------------------------------------------------------------- */

/* CS = PG10 — défini dans CubeMX comme CS_EMG_Pin sur GPIOG */
#define ADS1292R_CS_HIGH    HAL_GPIO_WritePin(CS_EMG_GPIO_Port, CS_EMG_Pin, GPIO_PIN_SET)
#define ADS1292R_CS_LOW     HAL_GPIO_WritePin(CS_EMG_GPIO_Port, CS_EMG_Pin, GPIO_PIN_RESET)

/* RESET = PE14 — à configurer dans CubeMX : GPIO Output · Label = RESET_EMG */
#define ADS1292R_RES_HIGH   HAL_GPIO_WritePin(RESET_EMG_GPIO_Port, RESET_EMG_Pin, GPIO_PIN_SET)
#define ADS1292R_RES_LOW    HAL_GPIO_WritePin(RESET_EMG_GPIO_Port, RESET_EMG_Pin, GPIO_PIN_RESET)

/* START = PE15 — à configurer dans CubeMX : GPIO Output · Label = START_EMG */
#define ADS1292R_START_HIGH HAL_GPIO_WritePin(START_EMG_GPIO_Port, START_EMG_Pin, GPIO_PIN_SET)
#define ADS1292R_START_LOW  HAL_GPIO_WritePin(START_EMG_GPIO_Port, START_EMG_Pin, GPIO_PIN_RESET)

/* ---------------------------------------------------------------------------
 * COMMANDES SPI — Datasheet Table 13 (page 37)
 * --------------------------------------------------------------------------- */

/* Commandes système */
#define ADS1292R_WAKEUP     0x02  /* sortir du mode standby */
#define ADS1292R_STANDBY    0x04  /* entrer en mode standby */
#define ADS1292R_ADSRESET   0x06  /* reset du système */
#define ADS1292R_START      0x08  /* démarrer les conversions */
#define ADS1292R_STOP       0x0A  /* arrêter les conversions */
#define ADS1292R_OFFSETCAL  0x1A  /* calibration de l'offset */

/* Commandes de lecture des données */
#define ADS1292R_RDATAC     0x10  /* Read Data Continuously — mode continu */
#define ADS1292R_SDATAC     0x11  /* Stop Read Data Continuously */
#define ADS1292R_RDATA      0x12  /* Read Data on Demand — lecture unique */

/* Commandes de lecture/écriture des registres
 * Format : 001r rrrr 000n nnnn
 * r rrrr = adresse du registre
 * n nnnn = nombre de registres à lire/écrire - 1 */
#define ADS1292R_RREG       0x20  /* lire  registre : 0x20 | adresse */
#define ADS1292R_WREG       0x40  /* écrire registre : 0x40 | adresse */

/* ---------------------------------------------------------------------------
 * ADRESSES DES REGISTRES INTERNES — Datasheet Table 14 (page 38)
 * --------------------------------------------------------------------------- */
#define ADS1292R_ID         0x00  /* registre ID — lecture seule */
#define ADS1292R_CONFIG1    0x01  /* configuration 1 — fréquence échantillonnage */
#define ADS1292R_CONFIG2    0x02  /* configuration 2 — référence, test signal */
#define ADS1292R_LOFF       0x03  /* contrôle lead-off */
#define ADS1292R_CH1SET     0x04  /* configuration canal 1 */
#define ADS1292R_CH2SET     0x05  /* configuration canal 2 */
#define ADS1292R_RLD_SENS   0x06  /* sélection RLD */
#define ADS1292R_LOFF_SENS  0x07  /* sélection lead-off sense */
#define ADS1292R_LOFF_STAT  0x08  /* statut lead-off — lecture seule */
#define ADS1292R_RESP1      0x09  /* contrôle respiration 1 */
#define ADS1292R_RESP2      0x0A  /* contrôle respiration 2 */
#define ADS1292R_GPIO       0x0B  /* contrôle GPIO */

/* ---------------------------------------------------------------------------
 * VALEURS DU REGISTRE ID — Datasheet Table 15 (page 38)
 * --------------------------------------------------------------------------- */
#define DEVICE_ID_ADS1292   0x53  /* identifiant ADS1292 */
#define DEVICE_ID_ADS1292R  0x73  /* identifiant ADS1292R — notre capteur */

/* ---------------------------------------------------------------------------
 * CONFIG1 — Registre 0x01 — Fréquence d'échantillonnage
 * Datasheet Table 16 (page 39)
 * --------------------------------------------------------------------------- */
#define DATA_RATE_125SPS    0x00  /* 125 samples/seconde */
#define DATA_RATE_250SPS    0x01  /* 250 samples/seconde */
#define DATA_RATE_500SPS    0x02  /* 500 samples/seconde — NOTRE CHOIX pour EMG */
#define DATA_RATE_1kSPS     0x03  /* 1000 samples/seconde */
#define DATA_RATE_2kSPS     0x04  /* 2000 samples/seconde */
#define DATA_RATE_4kSPS     0x05  /* 4000 samples/seconde */
#define DATA_RATE_8kSPS     0x06  /* 8000 samples/seconde */


/* ---------------------------------------------------------------------------
 * CONFIG2 — Registre 0x02
 * Datasheet Table 17 (page 39)
 * --------------------------------------------------------------------------- */
#define PDB_LOFF_COMP_OFF   0     /* comparateurs lead-off désactivés (défaut) */
#define PDB_LOFF_COMP_ON    1     /* comparateurs lead-off activés */
#define PDB_REFBUF_OFF      0     /* référence interne éteinte (défaut) */
#define PDB_REFBUF_ON       1     /* référence interne activée */
#define VREF_242V           0     /* référence 2.42V (défaut) */
#define VREF_4V             1     /* référence 4.033V */
#define CLK_EN_OFF          0     /* sortie horloge désactivée (défaut) */
#define CLK_EN_ON           1     /* sortie horloge activée */
#define INT_TEST_OFF        0     /* signal test interne désactivé (défaut) */
#define INT_TEST_ON         1     /* signal test interne activé */
#define TEST_FREQ_DC        0     /* signal test DC (défaut) */
#define TEST_FREQ_AC        1     /* signal test AC 1Hz carré */

/* ---------------------------------------------------------------------------
 * CH1SET / CH2SET — Registres 0x04 et 0x05
 * Datasheet Table 19 (page 41)
 * --------------------------------------------------------------------------- */
#define PD_ON               0     /* canal actif (défaut) */
#define PD_OFF              1     /* canal en veille */
#define GAIN_6              0     /* gain PGA = 6 (défaut) */
#define GAIN_1              1     /* gain PGA = 1 */
#define GAIN_2              2     /* gain PGA = 2 */
#define GAIN_3              3     /* gain PGA = 3 */
#define GAIN_4              4     /* gain PGA = 4 */
#define GAIN_8              5     /* gain PGA = 8 */
#define GAIN_12             6     /* gain PGA = 12 */
#define MUX_Normal_input    0     /* entrée électrode normale (défaut) */
#define MUX_input_shorted   1     /* entrée court-circuitée */
#define MUX_Test_signal     5     /* signal test interne */
#define MUX_RLD_DRP         6     /* mesure RLD positive */
#define MUX_RLD_DRM         7     /* mesure RLD négative */
#define MUX_RLD_DRPM        8     /* mesure RLD différentielle */
#define MUX_RSP_IN3P        9     /* voie respiration IN3P/IN3N */

/* ---------------------------------------------------------------------------
 * RLD_SENS — Registre 0x06
 * --------------------------------------------------------------------------- */
#define PDB_RLD_OFF         0     /* buffer RLD éteint (défaut) */
#define PDB_RLD_ON          1     /* buffer RLD allumé */
#define RLD_LOFF_SENSE_OFF  0     /* détection déconnexion RLD désactivée (défaut) */
#define RLD_LOFF_SENSE_ON   1     /* détection déconnexion RLD activée */
#define RLD_CANNLE_OFF      0     /* canal RLD désactivé (défaut) */
#define RLD_CANNLE_ON       1     /* canal RLD activé */

/* ---------------------------------------------------------------------------
 * LOFF_SENS — Registre 0x07
 * --------------------------------------------------------------------------- */
#define FLIP2_OFF           0     /* direction courant lead-off ch2 normale (défaut) */
#define FLIP2_ON            1     /* direction courant lead-off ch2 inversée */
#define FLIP1_OFF           0     /* direction courant lead-off ch1 normale (défaut) */
#define FLIP1_ON            1     /* direction courant lead-off ch1 inversée */
#define LOFF_CANNLE_OFF     0     /* détection lead-off canal désactivée (défaut) */
#define LOFF_CANNLE_ON      1     /* détection lead-off canal activée */

/* ---------------------------------------------------------------------------
 * RESP1 — Registre 0x09
 * --------------------------------------------------------------------------- */
#define RESP_DEMOD_OFF      0     /* démodulation respiration désactivée (défaut) */
#define RESP_DEMOD_ON       1     /* démodulation respiration activée */
#define RESP_MOD_OFF        0     /* modulation respiration désactivée (défaut) */
#define RESP_MOD_ON         1     /* modulation respiration activée */
#define RESP_CTRL_CLOCK_INTERNAL 0 /* horloge respiration interne */
#define RESP_CTRL_CLOCK_EXTERNAL 1 /* horloge respiration externe */

/* ---------------------------------------------------------------------------
 * RESP2 — Registre 0x0A
 * --------------------------------------------------------------------------- */
#define CALIB_OFF           0     /* calibration offset désactivée (défaut) */
#define CALIB_ON            1     /* calibration offset activée */
#define FREQ_32K            0     /* fréquence respiration 32kHz (défaut) */
#define FREQ_64K            1     /* fréquence respiration 64kHz */
#define RLDREF_INT_EXTERN   0     /* RLDREF signal externe */
#define RLDREF_INT_INTERNALLY 1   /* RLDREF généré en interne (AVDD+AVSS)/2 */

/* ---------------------------------------------------------------------------
 * STRUCTURES DE CONFIGURATION
 * --------------------------------------------------------------------------- */

typedef struct {
    uint8_t Data_Rate;        /* fréquence d'échantillonnage ADC */
} ADS1292_CONFIG1;

typedef struct {
    uint8_t Pdb_Loff_Comp;   /* alimentation comparateur lead-off */
    uint8_t Pdb_Refbuf;      /* alimentation buffer référence */
    uint8_t Vref;             /* tension de référence interne */
    uint8_t Clk_EN;           /* sortie horloge */
    uint8_t Int_Test;         /* signal test interne */
    uint8_t Test_Freq;        /* fréquence signal test */
} ADS1292_CONFIG2;

typedef struct {
    uint8_t PD;               /* mise en veille du canal */
    uint8_t GAIN;             /* gain PGA */
    uint8_t MUX;              /* sélection entrée */
} ADS1292_CHSET;

typedef struct {
    uint8_t Pdb_Rld;          /* alimentation buffer RLD */
    uint8_t Rld_Loff_Sense;   /* détection déconnexion RLD */
    uint8_t Rld2N;            /* entrée négative CH2 RLD */
    uint8_t Rld2P;            /* entrée positive CH2 RLD */
    uint8_t Rld1N;            /* entrée négative CH1 RLD */
    uint8_t Rld1P;            /* entrée positive CH1 RLD */
} ADS1292_RLD_SENS;

typedef struct {
    uint8_t Flip2;            /* direction courant lead-off CH2 */
    uint8_t Flip1;            /* direction courant lead-off CH1 */
    uint8_t Loff2N;           /* détection lead-off négatif CH2 */
    uint8_t Loff2P;           /* détection lead-off positif CH2 */
    uint8_t Loff1N;           /* détection lead-off négatif CH1 */
    uint8_t Loff1P;           /* détection lead-off positif CH1 */
} ADS1292_LOFF_SENS;

typedef struct {
    uint8_t RESP_DemodEN;     /* activation démodulation respiration */
    uint8_t RESP_modEN;       /* activation modulation respiration */
    uint8_t RESP_ph;          /* phase du signal de démodulation */
    uint8_t RESP_Ctrl;        /* sélection horloge respiration */
} ADS1292_RESP1;

typedef struct {
    uint8_t Calib;            /* calibration offset */
    uint8_t freq;             /* fréquence contrôle respiration */
    uint8_t Rldref_Int;       /* source signal RLDREF */
} ADS1292_RESP2;

typedef struct {
    int32_t ch1;        // Signal Canal 1 si en veux mesuré deux groupes de muscle
    int32_t ch2;        // Signal Canal 2
    uint32_t timestamp; // Temps exact du sample (TIM2)
}ADS1292R_Sample_t;
/* ---------------------------------------------------------------------------
 * PROTOTYPES DES FONCTIONS PUBLIQUES
 * --------------------------------------------------------------------------- */
void    ADS1292R_SetBuffer(void);
uint8_t ADS1292R_SPI_TransmitReceive(uint8_t tx_data);
uint8_t ADS1292R_SendCmd(uint8_t cmd);
uint8_t ADS1292R_RwReg(uint8_t cmd, uint8_t data);
void    ADS1292R_WriteMultiRegs(uint8_t reg, uint8_t *data, uint8_t len);
void    ADS1292R_ReadMultiRegs(uint8_t reg, uint8_t *data, uint8_t len);
void    ADS1292R_GetValue(void);
uint8_t ADS1292R_Init(void);          /* retourne 1=succès · 0=capteur non détecté */
void    ADS1292R_ADCStartNormal(void);
void    ADS1292R_ADCStartTest(void);
void ADS1292R_Decode(uint8_t *raw_buf, ADS1292R_Sample_t *sample, uint32_t time);

#endif /* __ADS1292R_H */
