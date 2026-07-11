#ifndef MAX86141_H
#define MAX86141_H

/*
 * ===========================================================================
 * Fichier  : max86141.h
 *
 *
 *
 * Description :
 *   Driver simplifié pour le MAX86141 sur STM32 HAL SPI.
 *   Ce driver permet de :
 *     1. Initialiser le capteur (configuration de base pour SpO2)
 *     2. Lire les données PPG depuis la FIFO interne du capteur
 *     3. Récupérer le nombre de samples disponibles dans la FIFO
 *
 * Référence : MAX86140/MAX86141 Datasheet Rev.5 - Analog Devices
 * ===========================================================================
 */

#include "main.h"   /* contient SPI_HandleTypeDef, GPIO, HAL */

/* ---------------------------------------------------------------------------
 * ADRESSES DES REGISTRES PRINCIPAUX
 * Source : Datasheet MAX86141 - Table 1 (Register Map)
 * --------------------------------------------------------------------------- */

/* Registres d'état et d'interruption */
#define MAX86141_REG_INT_STAT1      0x00  /* Interrupt Status 1  - indique les interruptions actives */
#define MAX86141_REG_INT_STAT2      0x01  /* Interrupt Status 2  - interruptions supplémentaires    */
#define MAX86141_REG_INT_EN1        0x02  /* Interrupt Enable 1  - activer/désactiver interruptions */
#define MAX86141_REG_INT_EN2        0x03  /* Interrupt Enable 2  - interruptions supplémentaires    */

/* Registres FIFO */
#define MAX86141_REG_FIFO_WR_PTR    0x04  /* FIFO Write Pointer  - où le capteur écrit le prochain sample */
#define MAX86141_REG_FIFO_RD_PTR    0x05  /* FIFO Read Pointer   - où on doit lire le prochain sample    */
#define MAX86141_REG_OVF_COUNTER    0x06  /* Overflow Counter    - compte les samples perdus par dépassement */
#define MAX86141_REG_FIFO_COUNT     0x07  /* FIFO Data Count     - nombre de samples disponibles dans la FIFO */
#define MAX86141_REG_FIFO_DATA      0x08  /* FIFO Data           - registre de lecture des données PPG */
#define MAX86141_REG_FIFO_CONFIG1   0x09  /* FIFO Config 1       - seuil d'interruption FIFO presque pleine */
#define MAX86141_REG_FIFO_CONFIG2   0x0A  /* FIFO Config 2       - options avancées FIFO */

/* Registres de configuration système */
#define MAX86141_REG_SYS_CTRL       0x0D  /* System Control      - reset, shutdown, démarrage mesure */
#define MAX86141_REG_PPG_SYNC_CTRL  0x10  /* PPG Sync Control    - synchronisation des mesures */

/* Registres de configuration PPG */
#define MAX86141_REG_PPG_CONFIG1    0x11  /* PPG Config 1        - durée d'intégration, plage ADC */
#define MAX86141_REG_PPG_CONFIG2    0x12  /* PPG Config 2        - fréquence d'échantillonnage, moyenne */
#define MAX86141_REG_PPG_CONFIG3    0x13  /* PPG Config 3        - temps de stabilisation LED */

/* Registres de configuration photodiode */
#define MAX86141_REG_PD_BIAS        0x15  /* Photodiode Bias     - polarisation des photodiodes */

/* Registres de configuration LED */
#define MAX86141_REG_LED_RANGE      0x1A  /* LED Range           - plage de courant LED (31mA/62mA/93mA/124mA) */
#define MAX86141_REG_LED1_PA        0x23  /* LED1 Pulse Ampl.    - intensité LED1 (infrarouge) */
#define MAX86141_REG_LED2_PA        0x24  /* LED2 Pulse Ampl.    - intensité LED2 (rouge) */

/* Registres de configuration des séquences de mesure */
#define MAX86141_REG_LED_SEQ1       0x20  /* LED Sequence 1      - séquence de mesure 1 et 2 */
#define MAX86141_REG_LED_SEQ2       0x21  /* LED Sequence 2      - séquence de mesure 3 et 4 */
#define MAX86141_REG_LED_SEQ3       0x22  /* LED Sequence 3      - séquence de mesure 5 et 6 */

/* Registre d'identification */
#define MAX86141_REG_PART_ID        0xFF  /* Part ID             - ID du capteur, doit valoir 0x25 pour MAX86141 */

/* ---------------------------------------------------------------------------
 * VALEURS DE CONFIGURATION
 * Source : Datasheet MAX86141 - Sections 6 et 7
 * --------------------------------------------------------------------------- */

/* Valeur attendue du registre Part ID pour MAX86141 */
#define MAX86141_PART_ID            0x25

/* Commandes SPI
 * Le bit D7 de l'octet adresse indique lecture (1) ou écriture (0)
 * Source : Datasheet Section 7.1 - SPI Interface */
#define MAX86141_SPI_READ           0x80  /* OR avec adresse pour lire  : 0x80 | adresse */
#define MAX86141_SPI_WRITE          0x00  /* OR avec adresse pour écrire: 0x00 | adresse */

/* Configuration PPG_CONFIG1 (registre 0x11)
 * Bits [1:0] PPG_TINT : durée d'intégration
 *   0x3 = 123.8 µs (valeur maximale = meilleure sensibilité)
 * Bits [3:2] PPG_ADC_RGE : plage ADC
 *   0x2 = 16 µA (plage recommandée pour SpO2)
 * Valeur combinée = 0x0E = 0b00001110 */
#define MAX86141_PPG_CONFIG1_VAL    0x0E

/* Configuration PPG_CONFIG2 (registre 0x12)
 * Bits [7:3] PPG_SR : fréquence d'échantillonnage
 *   0x01 = 400 sps (400 samples par seconde - notre objectif)
 * Bits [2:0] SMP_AVE : nombre de moyennages
 *   0x00 = pas de moyenne (1 sample direct)
 * Valeur = (0x01 << 3) | 0x00 = 0x08 */
#define MAX86141_PPG_CONFIG2_VAL    0x08

/* Configuration LED Range (registre 0x1A)
 * Bits [1:0] LED1_RGE : plage courant LED1 (IR)
 * Bits [3:2] LED2_RGE : plage courant LED2 (Rouge)
 *   0x1 = 62 mA pour les deux LEDs
 * Valeur = 0x05 = 0b00000101 */
#define MAX86141_LED_RANGE_VAL      0x05

/* Intensité des LEDs (registres 0x23 et 0x24)
 * Valeur sur 8 bits : 0x7F = environ 50% de la plage choisie
 * Avec plage 62mA : 0x7F ≈ 31mA par LED
 * A ajuster lors de la calibration sur le prototype */
#define MAX86141_LED1_PA_VAL        0x7F  /* LED1 infrarouge  ~31mA */
#define MAX86141_LED2_PA_VAL        0x7F  /* LED2 rouge       ~31mA */

/* Configuration séquence LED (registre 0x20)
 * Slot 1 : LED1 (infrarouge) → bits [3:0] = 0x1
 * Slot 2 : LED2 (rouge)      → bits [7:4] = 0x2
 * Valeur = 0x21 */
#define MAX86141_LED_SEQ1_VAL       0x21

/* Seuil FIFO presque pleine (registre 0x09)
 * FIFO_A_FULL = 64 - seuil = nombre de samples avant interruption A_FULL
 * On choisit seuil = 0 → interruption quand 64 samples sont dans la FIFO
 * Valeur = 0x00 */
#define MAX86141_FIFO_CONFIG1_VAL   0x00

/* Configuration FIFO (registre 0x0A)
 * Bit 1 FIFO_STAT_CLR = 1 : vider automatiquement les flags à la lecture
 * Valeur = 0x02 */
#define MAX86141_FIFO_CONFIG2_VAL   0x02

/* Activation de l'interruption A_FULL (FIFO presque pleine)
 * Registre INT_EN1 bit 7 = 1
 * Valeur = 0x80 */
#define MAX86141_INT_EN1_VAL        0x80

/* Taille maximale de la FIFO interne du MAX86141
 * Source : Datasheet Section 5 - FIFO Description
 * La FIFO peut stocker jusqu'à 128 mots de 3 octets chacun
 * Avec 2 slots actifs (IR + Rouge) = 64 paires de samples */
#define MAX86141_FIFO_MAX_SAMPLES   64

/* Taille d'un sample en octets
 * Chaque mesure = 3 octets (19 bits utiles + padding)
 * Source : Datasheet Section 5.3 */
#define MAX86141_SAMPLE_SIZE_BYTES  3

#define PPG_SAMPLE_PERIOD_US  2502.44f

/* ---------------------------------------------------------------------------
 * STRUCTURE DE RÉSULTAT
 * Un sample PPG contient une mesure infrarouge et une mesure rouge
 * --------------------------------------------------------------------------- */
typedef struct {
    uint32_t ir;     /* Valeur infrarouge (LED1) - 19 bits utiles sur 32 bits  */
    uint32_t red;    /* Valeur rouge      (LED2) - 19 bits utiles sur 32 bits  */
    uint32_t timestamp; // Temps exact en microsecondes (µs)*/
} MAX86141_Sample_t;

/* ---------------------------------------------------------------------------
 * PROTOTYPES DES FONCTIONS PUBLIQUES
 * --------------------------------------------------------------------------- */

/* Initialisation du capteur */
uint8_t  MAX86141_Init(SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef      *cs_port,
                       uint16_t           cs_pin);

/* Lecture du nombre de samples disponibles dans la FIFO */
uint8_t  MAX86141_GetFIFOCount(SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef      *cs_port,
                               uint16_t           cs_pin);

/* Lecture de N samples depuis la FIFO */
uint8_t  MAX86141_ReadFIFO(SPI_HandleTypeDef *hspi,
                           GPIO_TypeDef      *cs_port,
                           uint16_t           cs_pin,
                           MAX86141_Sample_t *samples,
                           uint8_t            count);

/* Lecture d'un registre (fonction interne rendue accessible pour debug) */
uint8_t  MAX86141_ReadReg(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef      *cs_port,
                          uint16_t           cs_pin,
                          uint8_t            reg_addr);
void MAX86141_WriteReg(SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin,
                        uint8_t reg,
                        uint8_t value);
void MAX86141_DecodeFIFO(uint8_t *rx_buf,
                          MAX86141_Sample_t *samples,
                          uint8_t count,
						  uint32_t trigger_time);




#endif /* MAX86141_H */
