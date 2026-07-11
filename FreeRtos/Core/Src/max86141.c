/*
 * ===========================================================================
 * Fichier  : max86141.c
 * Capteur  : MAX86141 - Capteur optique PPG (Pulse Oximeter)
 * Auteur   : Etudiante - Projet Biosignal STM32H7A3
 * Date     : 2026
 *
 * Description :
 *   Implémentation du driver simplifié MAX86141.
 *   Séquence d'initialisation basée sur le pseudocode du datasheet
 *   Section 8 "Device Open Start" (page 25).
 *
 * Communication SPI :
 *   - Le MAX86141 fonctionne en SPI Mode 0 (CPOL=0, CPHA=0)
 *   - Chaque transaction = CS bas + [adresse + R/W] + [données] + CS haut
 *   - Lecture  : envoyer (0x80 | adresse) puis lire 1 octet
 *   - Ecriture : envoyer (0x00 | adresse) puis écrire 1 octet
 *
 * Référence : MAX86140/MAX86141 Datasheet Rev.5 - Analog Devices
 * ===========================================================================
 */

#include "max86141.h"

/* ---------------------------------------------------------------------------
 * FONCTIONS PRIVÉES (statiques - utilisées uniquement dans ce fichier)
 * --------------------------------------------------------------------------- */

/*
 * CS_LOW - Sélectionner le capteur (début de transaction SPI)
 * CS = 0 (Low) signifie "je parle à ce capteur"
 */
static void CS_LOW(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

/*
 * CS_HIGH - Désélectionner le capteur (fin de transaction SPI)
 * CS = 1 (High) signifie "fin de communication"
 */
static void CS_HIGH(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}



/* ---------------------------------------------------------------------------
 * FONCTIONS PUBLIQUES
 * --------------------------------------------------------------------------- */

/*
 * MAX86141_ReadReg - Lire la valeur d'un registre du capteur
 *
 * Protocole SPI (Datasheet Section 7.1) :
 *   1. CS bas
 *   2. Envoyer adresse avec bit R/W = 1 (lecture) → 0x80 | adresse
 *   3. Envoyer un octet dummy (0x00) pour générer les cycles d'horloge
 *   4. Récupérer l'octet reçu = valeur du registre
 *   5. CS haut
 *
 * Paramètres :
 *   hspi     - pointeur vers le handle SPI HAL
 *   cs_port  - port GPIO du Chip Select
 *   cs_pin   - pin GPIO du Chip Select
 *   reg_addr - adresse du registre à lire
 *
 * Retourne : valeur lue dans le registre (1 octet)
 */




/*
 * MAX86141_WriteReg - Ecrire une valeur dans un registre du capteur
 *
 * Protocole SPI (Datasheet Section 7.1) :
 *   1. CS bas
 *   2. Envoyer adresse avec bit R/W = 0 (écriture)
 *   3. Envoyer la valeur à écrire
 *   4. CS haut
 *
 * Paramètres :
 *   hspi     - pointeur vers le handle SPI HAL
 *   cs_port  - port GPIO du Chip Select
 *   cs_pin   - pin GPIO du Chip Select
 *   reg_addr - adresse du registre (ex: 0x11 pour PPG_CONFIG1)
 *   value    - valeur à écrire dans le registre
 */

void MAX86141_WriteReg(SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef      *cs_port,
                               uint16_t           cs_pin,
                               uint8_t            reg_addr,
                               uint8_t            value)
{
    uint8_t tx[2];
    tx[0] = MAX86141_SPI_WRITE | reg_addr; /* bit R/W = 0 → écriture */
    tx[1] = value;                          /* valeur à écrire         */

    CS_LOW(cs_port, cs_pin);
    HAL_SPI_Transmit(hspi, tx, 2, 10);     /* envoyer 2 octets, timeout 10ms */
    CS_HIGH(cs_port, cs_pin);
}






uint8_t MAX86141_ReadReg(SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef      *cs_port,
                          uint16_t           cs_pin,
                          uint8_t            reg_addr)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = MAX86141_SPI_READ | reg_addr; /* bit R/W = 1 → lecture      */
    tx[1] = 0x00;                          /* dummy byte pour l'horloge  */

    CS_LOW(cs_port, cs_pin);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 10);
    CS_HIGH(cs_port, cs_pin);

    return rx[1]; /* rx[0] = réponse pendant l'envoi de l'adresse (ignorée) */
                  /* rx[1] = vraie valeur du registre demandé               */
}

/*
 * MAX86141_Init - Initialiser le MAX86141 pour mesure SpO2 à 400 Hz
 *
 * Séquence d'initialisation (Datasheet Section 8 "Device Open Start") :
 *   1. Soft Reset       → remet à zéro tous les registres
 *   2. Attendre 1ms     → temps de reset (Datasheet : 1ms minimum)
 *   3. Shutdown         → désactive les LEDs pendant la configuration
 *   4. Vider interruptions → lire les registres d'état pour les effacer
 *   5. Configurer PPG   → durée d'intégration, plage ADC
 *   6. Configurer SR    → fréquence d'échantillonnage 400 Hz
 *   7. Configurer LEDs  → plage de courant et intensité IR + Rouge
 *   8. Configurer FIFO  → seuil d'interruption à 64 samples
 *   9. Activer interrupt → A_FULL pour réveiller la tâche FreeRTOS
 *  10. Démarrer mesure  → sortir du mode shutdown
 *
 * Paramètres :
 *   hspi    - pointeur vers le handle SPI (ex: &hspi2)
 *   cs_port - port GPIO du CS (ex: GPIOG)
 *   cs_pin  - pin du CS (ex: GPIO_PIN_9 pour CS_PPG sur PG9)
 *
 * Retourne :
 *   1 si le capteur répond correctement (Part ID = 0x25)
 *   0 si erreur (capteur absent ou mal connecté)
 */
uint8_t MAX86141_Init(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef      *cs_port,
                      uint16_t           cs_pin)
{
    uint8_t part_id;

    /* --- Etape 0 : Vérifier que le capteur est présent ---
     * On lit le registre Part ID (0xFF)
     * Pour le MAX86141, la valeur attendue est 0x25
     * Si la valeur est différente → problème de connexion SPI */
    part_id = MAX86141_ReadReg(hspi, cs_port, cs_pin, MAX86141_REG_PART_ID);
    if (part_id != MAX86141_PART_ID) {
        return 0; /* capteur non détecté - vérifier le câblage SPI */
    }

    /* --- Etape 1 : Soft Reset ---
     * Registre SYS_CTRL (0x0D) bit 0 = RESET = 1
     * Remet tous les registres à leurs valeurs par défaut
     * Source : Datasheet Table Register 0x0D */
    MAX86141_WriteReg(hspi, cs_port, cs_pin, MAX86141_REG_SYS_CTRL, 0x01);

    /* --- Etape 2 : Attendre 1ms après le reset ---
     * Le datasheet dit explicitement : après le reset attendre minimum 1ms avant
     * d'envoyer toute commande SPI. Sans ce délai les premières commandes peuvent
     * être ignorées car le capteur n'a pas fini sa réinitialisation interne.
     * Source : Datasheet Section 8 */
    HAL_Delay(1);

    /* --- Etape 3 : Mode Shutdown ---
     * Registre SYS_CTRL (0x0D) bit 1 = SHDN = 1
     * On désactive les LEDs pendant la configuration pour économiser
     * de l'énergie et éviter des mesures incohérentes
     * Source : Datasheet Section 8 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin, MAX86141_REG_SYS_CTRL, 0x02);

    /* --- Etape 4 : Vider les registres d'interruption ---
     * La lecture de INT_STAT1 et INT_STAT2 efface automatiquement
     * les flags d'interruption (lecture = clear)
     * Le datasheet dit que les registres INT_STAT1 (0x00) et INT_STAT2 (0x01)
     * sont Read-to-Clear — lire ces registres efface automatiquement tous les
     * flags d'interruption. Si on ne fait pas ça, une ancienne interruption résiduelle
     * du démarrage pourrait réveiller vTaskPPGAcq immédiatement.
     * Source : Datasheet Section 4.1 */
    MAX86141_ReadReg(hspi, cs_port, cs_pin, MAX86141_REG_INT_STAT1);
    MAX86141_ReadReg(hspi, cs_port, cs_pin, MAX86141_REG_INT_STAT2);


    /* --- Etape 5 : Configurer PPG_CONFIG1 (0x11) ---
     *Bit 1-0 TINT : PPG_TINT = 0x3 → 117.3µs Durée pendant laquelle l'ADC intègre le courant du photodiode
     * Plage ADC : Bit 3-2 ADC_RG: 16 µA    (PPG_ADC_RGE = 0x2)
     * Valeur = 0x0E
     * PPG_ADC_RGE = 0x2 → 16µA Plage de courant max que le photodiode peut envoyer
     * à l'ADC sans saturation
     * Source : Datasheet Table Register 0x11 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_PPG_CONFIG1, MAX86141_PPG_CONFIG1_VAL);

    /* --- Etape 6 : Configurer PPG_CONFIG2 (0x12) ---
     * Fréquence d'échantillonnage : 400 sps (PPG_SR = 0x01)
     * Pas de moyenne              : SMP_AVE = 0x00 (1 sample direct)
     * Valeur = 0x08
     * Source : Datasheet Table Register 0x12 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_PPG_CONFIG2, MAX86141_PPG_CONFIG2_VAL);

    /* --- Etape 7a : Configurer la plage des LEDs (0x1A) ---
     * LED1 (IR) et LED2 (Rouge) en plage 62mA
     * Valeur = 0x05
     * Source : Datasheet Table Register 0x1A */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_LED_RANGE, MAX86141_LED_RANGE_VAL);

    /* --- Etape 7b : Intensité LED1 infrarouge (0x23) ---
     * Valeur = 0x7F ≈ 31mA (50% de la plage 62mA)
     * A calibrer selon les conditions de mesure
     * Source : Datasheet Table Register 0x23 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_LED1_PA, MAX86141_LED1_PA_VAL);

    /* --- Etape 7c : Intensité LED2 rouge (0x24) ---
     * LED_RANGE définit le max · LED_PA définit la valeur réelle
     * 31mA est une valeur de départ recommandée · pas une valeur définitiv
     * Valeur = 0x7F ≈ 31mA (50% de la plage 62mA)
     * Source : Datasheet Table Register 0x24 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_LED2_PA, MAX86141_LED2_PA_VAL);

    /* --- Etape 7d : Séquence des mesures (0x20) ---
     * Slot 1 = LED1 (infrarouge) et Slot 2 = LED2 (rouge)
     * Valeur = 0x21
     * bits [3:0] = LEDC1 = 0x1 → slot1 = LED1 IR · bits [7:4] = LEDC2 = 0x2 → slot2 = LED2 Rouge
     * Source : Datasheet Table Register 0x20 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_LED_SEQ1, MAX86141_LED_SEQ1_VAL);

    /* --- Etape 8 : Configurer la FIFO ---
     * Seuil A_FULL = 0 → interruption quand 64 samples dans FIFO
     * 0x09 FIFO_CONFIG1	0x00	Seuil = 128 mots = 64 paires
     * Source : Datasheet Table Register 0x09 */
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_FIFO_CONFIG1, MAX86141_FIFO_CONFIG1_VAL);
    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_FIFO_CONFIG2, MAX86141_FIFO_CONFIG2_VAL);

    /* --- Etape 9 : Activer l'interruption A_FULL ---
     * INT_EN1 bit 7 = 1 → interruption quand FIFO atteint le seuil
     * déclenche INT=0 sur la broche physique → PE11 → ISR → vTaskPPGAcq
     * Cette interruption réveillera vTaskPPGAcq via PE11 (INT_PPG)
     * Source : Datasheet Table Register 0x02 */

    MAX86141_WriteReg(hspi, cs_port, cs_pin,
                      MAX86141_REG_INT_EN1, MAX86141_INT_EN1_VAL);

     /* --- Etape 10 : Sortir du mode Shutdown → démarrer les mesures ---
      * Registre SYS_CTRL (0x0D) bit 1 = SHDN = 0
     // Laissez le capteur en sommeil (Shutdown)*/

     MAX86141_WriteReg(hspi, cs_port, cs_pin, MAX86141_REG_SYS_CTRL, 0x02);

    return 1; /* initialisation réussie */
}

/*
 * MAX86141_GetFIFOCount - Lire le nombre de samples disponibles dans la FIFO
 *
 * La FIFO du MAX86141 peut contenir jusqu'à 128 mots (64 paires IR+Rouge).
 * Le registre FIFO_DATA_COUNT (0x07) indique combien de mots sont disponibles.
 * Avec 2 slots actifs (IR + Rouge), 1 paire = 2 mots → diviser par 2.
 *
 * Paramètres :
 *   hspi    - handle SPI
 *   cs_port - port GPIO CS
 *   cs_pin  - pin GPIO CS
 *
 * Retourne : nombre de paires (IR + Rouge) disponibles dans la FIFO
 */
uint8_t MAX86141_GetFIFOCount(SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef      *cs_port,
                               uint16_t           cs_pin)
{
    uint8_t count = MAX86141_ReadReg(hspi, cs_port, cs_pin,
                                      MAX86141_REG_FIFO_COUNT);
    /* count = nombre de mots (3 octets chacun) dans la FIFO
     * Avec 2 slots actifs, 1 paire = 2 mots
     * Donc nombre de paires = count / 2 */
    return count / 2;
}

/*
 * MAX86141_ReadFIFO - Lire N paires de samples (IR + Rouge) depuis la FIFO
 *
 * Protocole de lecture FIFO (Datasheet Section 5.3) :
 *   1. CS bas
 *   2. Envoyer adresse FIFO_DATA (0x08) avec bit lecture (0x80)
 *   3. Pour chaque sample : lire 3 octets IR puis 3 octets Rouge
 *   4. CS haut
 *
 * Format d'un sample (3 octets = 24 bits) :
 *   Octet 0 : bits [23:16] - MSB
 *   Octet 1 : bits [15:8]  - milieu
 *   Octet 2 : bits [7:0]   - LSB
 *   Les bits [23:19] sont à 0 → 19 bits utiles
 *   Source : Datasheet Section 5.3
 *
 * Paramètres :
 *   hspi    - handle SPI
 *   cs_port - port GPIO CS
 *   cs_pin  - pin GPIO CS
 *   samples - tableau de résultats à remplir
 *   count   - nombre de paires à lire (max 64)
 *
 * Retourne : nombre de paires effectivement lues
 */
uint8_t MAX86141_ReadFIFO(SPI_HandleTypeDef *hspi,
                           GPIO_TypeDef      *cs_port,
                           uint16_t           cs_pin,
                           MAX86141_Sample_t *samples,
                           uint8_t            count)
{
    /* Sécurité : ne pas lire plus que la capacité maximale */
    if (count > MAX86141_FIFO_MAX_SAMPLES) {
        count = MAX86141_FIFO_MAX_SAMPLES;
    }

    /* Nombre total d'octets à lire :
     * count paires × 2 mesures par paire × 3 octets par mesure */
    uint16_t total_bytes = (uint16_t)count * 2 * MAX86141_SAMPLE_SIZE_BYTES;

    /* Buffer de réception SPI
     * +1 pour l'octet d'adresse envoyé au début */
    uint8_t  tx[1 + 128 * 6]; /* taille max : 1 addr + 64 paires × 6 octets */
    uint8_t  rx[1 + 128 * 6];

    /* Préparer la commande de lecture FIFO */
    tx[0] = MAX86141_SPI_READ | MAX86141_REG_FIFO_DATA;

    /* Remplir le reste avec des bytes dummy pour générer l'horloge SPI */
    for (uint16_t i = 1; i <= total_bytes; i++) {
        tx[i] = 0x00;
    }

    /* Transaction SPI : envoyer adresse + dummy, recevoir données */
    CS_LOW(cs_port, cs_pin);
    HAL_SPI_TransmitReceive(hspi, tx, rx, total_bytes + 1, 50);
    CS_HIGH(cs_port, cs_pin);

    /* Décoder les données reçues
     * rx[0] = réponse pendant l'envoi de l'adresse (ignorée)
     * rx[1..N] = données FIFO utiles
     *
     * Pour chaque paire (IR + Rouge) :
     *   - 3 octets pour IR  : rx[base + 0], rx[base + 1], rx[base + 2]
     *   - 3 octets pour ROG : rx[base + 3], rx[base + 4], rx[base + 5] */
    for (uint8_t i = 0; i < count; i++) {
        uint16_t base = 1 + (uint16_t)i * 6; /* position dans le buffer rx */

        /* Reconstruire la valeur IR (19 bits utiles sur 24 bits) */
        samples[i].ir  = ((uint32_t)rx[base + 0] << 16)
                        | ((uint32_t)rx[base + 1] << 8)
                        |  (uint32_t)rx[base + 2];
        samples[i].ir &= 0x0007FFFF; /* masque 19 bits utiles */

        /* Reconstruire la valeur Rouge (19 bits utiles sur 24 bits) */
        samples[i].red  = ((uint32_t)rx[base + 3] << 16)
                         | ((uint32_t)rx[base + 4] << 8)
                         |  (uint32_t)rx[base + 5];
        samples[i].red &= 0x0007FFFF; /* masque 19 bits utiles */
    }

    return count;
}
/**
 * @brief Décode les données brutes reçues par le bus SPI (via DMA ou Polling)
 * @param rx_buf : Le buffer brut reçu (doit contenir l'octet de dummy au début)
 * @param samples : Tableau de structures où stocker les résultats
 * @param count : Nombre de paires (IR + Rouge) à décoder
 */
void MAX86141_DecodeFIFO(uint8_t *rx_buf, MAX86141_Sample_t *samples, uint8_t count, uint32_t trigger_time)
{
	float period = PPG_SAMPLE_PERIOD_US;

    for (uint8_t i = 0; i < count; i++)
    {
        /* Position dans le buffer rx :
         * +1 pour ignorer l'octet de réponse à l'adresse
         * + i * 6 car chaque paire (IR+Red) fait 6 octets (3+3) */
        uint16_t base = 1 + (uint16_t)i * 6;

        /* Reconstruire la valeur IR (19 bits utiles sur 24 bits) */
        samples[i].ir  = ((uint32_t)rx_buf[base + 0] << 16)
                        | ((uint32_t)rx_buf[base + 1] << 8)
                        |  (uint32_t)rx_buf[base + 2];
        samples[i].ir &= 0x0007FFFF; /* Masque 19 bits */

        /* Reconstruire la valeur Rouge (19 bits utiles sur 24 bits) */
        samples[i].red  = ((uint32_t)rx_buf[base + 3] << 16)
                         | ((uint32_t)rx_buf[base + 4] << 8)
                         |  (uint32_t)rx_buf[base + 5];
        samples[i].red &= 0x0007FFFF; /* Masque 19 bits */



              // Calcul du Timestamp ultra-précis
              // On recule dans le temps depuis le trigger_time
              // i = (count-1) est le dernier échantillon capté
              float offset = (float)(count - 1 - i) * period;

              samples[i].timestamp = trigger_time - (uint32_t)offset;
    }
}
