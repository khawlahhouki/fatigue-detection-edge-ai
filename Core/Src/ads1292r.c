/*
 * ===========================================================================
 * Fichier  : ads1292r.c
 * Capteur  : ADS1292R - Capteur ECG/EMG 2 canaux 24 bits
 * Auteur   : Etudiante - Projet Biosignal STM32H7A3
 * Date     : 2026
 *
 * Description :
 *   Implémentation du driver ADS1292R pour STM32H7A3 HAL SPI2.
 *   Basé sur sjZhu1020/STM32-HAL-ADS1292R — adapté pour notre projet.
 *
 * Modifications par rapport à l'original :
 *   1. hspi1 → hspi2 (notre bus SPI)
 *   2. Suppression de hmi_uart.h et HMI_printf()
 *   3. ADS1292R_Init() retourne uint8_t au lieu de void
 *   4. Boucle while() sans fin → remplacée par timeout 5 tentatives
 *   5. Commentaires traduits et enrichis
 *
 * Mode de fonctionnement dans notre projet FreeRTOS :
 *   - ADS1292R_Init() appelé au démarrage dans main.c
 *   - ADS1292R_ADCStartNormal() démarre le mode continu RDATAC à 500Hz
 *   - Chaque nouveau sample déclenche DRDY=0 sur PE12 → ISR → notification vTaskEMGAcq
 *   - vTaskEMGAcq appelle ADS1292R_GetValue() pour lire 9 octets
 *
 * Référence : ADS1292R Datasheet Rev.C - Texas Instruments SBAS502C
 * ===========================================================================
 */

#include "ads1292r.h"

/* ---------------------------------------------------------------------------
 * VARIABLES GLOBALES
 * --------------------------------------------------------------------------- */

/* Buffer de réception des données RDATAC — 9 octets par sample :
 * Octets [0..2] : 24 bits status (1100 + LOFF_STAT[4:0] + GPIO[1:0] + 13×0)
 * Octets [3..5] : 24 bits canal 1 (CH1) — complément à 2 · MSB en premier
 * Octets [6..8] : 24 bits canal 2 (CH2) — complément à 2 · MSB en premier */
uint8_t ADS1292R_data_buf[9] = {0};

/* Buffer d'émission dummy pour HAL_SPI_TransmitReceive en mode RDATAC
 * Le capteur envoie les données automatiquement — on envoie 9 octets nuls */
volatile uint8_t ADS1292R_tmp[9] = {0};

/* Flag indiquant qu'un nouveau sample est disponible dans ADS1292R_data_buf
 * Mis à 1 par l'ISR DRDY · remis à 0 par vTaskEMGAcq après lecture */
uint8_t ADS1292R_receive_flag = 0;

/* Tableau des valeurs de registres à envoyer lors de l'initialisation
 * Index correspond à l'adresse du registre (0x00 à 0x0B) */
volatile uint8_t ADS1292R_reg[12];

/* ---------------------------------------------------------------------------
 * CONFIGURATION PAR DÉFAUT
 * Ces valeurs sont optimisées pour mesure EMG à 500 Hz dans notre projet
 * --------------------------------------------------------------------------- */

/* CONFIG1 : fréquence d'échantillonnage 500 SPS
 * Datasheet : 1000 SPS correspond à DATA_RATE_1000SPS = 0x03*/
ADS1292_CONFIG1 ADS1292R_config1 = {
		 DATA_RATE_1kSPS
};

/* CONFIG2 : configuration de la référence et des signaux de test
 * PDB_LOFF_COMP_ON  : comparateurs lead-off activés — détecte électrode déconnectée
 * PDB_REFBUF_ON     : buffer référence interne activé — nécessaire pour mesure précise
 * VREF_242V         : référence 2.42V (défaut recommandé datasheet)
 * CLK_EN_OFF        : sortie horloge désactivée — on utilise l'horloge interne
 * INT_TEST_OFF      : signal test désactivé — mode mesure réel
 * TEST_FREQ_DC      : fréquence test DC (sans importance car test désactivé) */
ADS1292_CONFIG2 ADS1292R_config2 = {
    PDB_LOFF_COMP_ON,
    PDB_REFBUF_ON,
    VREF_242V,
    CLK_EN_OFF,
    INT_TEST_OFF,
    TEST_FREQ_DC
};

/* CH1SET : canal 1 — mesure ECG / signal faible
 * PD_ON      : canal 1 actif
 * GAIN_2     : gain PGA = 2 — adapté pour signal ECG (amplitude ~1mV)
 * MUX_Normal : entrée électrode normale */
ADS1292_CHSET ADS1292R_ch1set = {
		 PD_ON,
		 GAIN_12,
		 MUX_Normal_input
};

/* CH2SET : canal 2 — mesure EMG musculaire / signal plus fort
 * PD_ON      : canal 2 actif
 * GAIN_6     : gain PGA = 6 — adapté pour signal EMG (amplitude ~10mV)
 * MUX_Normal : entrée électrode normale */
ADS1292_CHSET ADS1292R_ch2set = {
    PD_ON,
    GAIN_12,
    MUX_Normal_input
};

/* RLD_SENS : configuration du circuit RLD (Right Leg Drive)
 * PDB_RLD_ON         : buffer RLD activé — nécessaire pour réduction bruit commun
 * RLD_LOFF_SENSE_OFF : détection déconnexion RLD désactivée
 * Les 4 derniers champs : canaux RLD — activés sur CH1 et CH2 · désactivés sur CH3 */
ADS1292_RLD_SENS ADS1292R_rld_sens = {
    PDB_RLD_ON,
    RLD_LOFF_SENSE_OFF,
    RLD_CANNLE_ON,
    RLD_CANNLE_ON,
    RLD_CANNLE_OFF,
    RLD_CANNLE_OFF
};

/* LOFF_SENS : configuration de la détection lead-off (électrode déconnectée)
 * FLIP2_OFF / FLIP1_OFF  : direction courant normale
 * RLD_CANNLE_ON×2        : détection activée sur CH1 et CH2
 * RLD_CANNLE_OFF×2       : détection désactivée sur voie respiration */
ADS1292_LOFF_SENS ADS1292R_loff_sens = {
    FLIP2_OFF,
    FLIP1_OFF,
    RLD_CANNLE_ON,
    RLD_CANNLE_ON,
    RLD_CANNLE_OFF,
    RLD_CANNLE_OFF
};

/* RESP1 : configuration respiration
 * RESP_DEMOD_ON              : démodulation canal 1 activée
 * RESP_MOD_ON                : modulation canal 1 activée
 * 0x0A                       : phase démodulation = 112.5 degrés
 * RESP_CTRL_CLOCK_INTERNAL   : horloge respiration interne */
ADS1292_RESP1 ADS1292R_resp1 = {
    RESP_DEMOD_ON,
    RESP_MOD_ON,
    0x0A,
    RESP_CTRL_CLOCK_INTERNAL
};

/* RESP2 : configuration respiration 2
 * CALIB_OFF             : calibration offset désactivée
 * FREQ_32K              : fréquence contrôle respiration 32kHz
 * RLDREF_INT_INTERNALLY : RLDREF généré en interne = (AVDD+AVSS)/2 */
ADS1292_RESP2 ADS1292R_resp2 = {
    CALIB_OFF,
    FREQ_32K,
    RLDREF_INT_INTERNALLY
};

/* ---------------------------------------------------------------------------
 * FONCTIONS PRIVÉES ET PUBLIQUES
 * --------------------------------------------------------------------------- */

/*
 * ADS1292R_SetBuffer — Remplir le tableau ADS1292R_reg avec les valeurs de config
 *
 * Cette fonction construit les valeurs des registres bit par bit
 * à partir des structures de configuration définies ci-dessus.
 * Le tableau est ensuite envoyé en une seule transaction SPI dans Init().
 */
void ADS1292R_SetBuffer(void)
{
    /* Registre ID (0x00) — lecture seule · on met la valeur attendue */
    ADS1292R_reg[ADS1292R_ID] = DEVICE_ID_ADS1292R;

    /* CONFIG1 (0x01) — fréquence d'échantillonnage */
    ADS1292R_reg[ADS1292R_CONFIG1] = 0x00;
    ADS1292R_reg[ADS1292R_CONFIG1] |= ADS1292R_config1.Data_Rate;

    /* CONFIG2 (0x02) — référence + test signal
     * Bit 7 = 1 obligatoire (valeur par défaut du datasheet) */
    ADS1292R_reg[ADS1292R_CONFIG2] = 0x00;
    ADS1292R_reg[ADS1292R_CONFIG2] |= ADS1292R_config2.Test_Freq;
    ADS1292R_reg[ADS1292R_CONFIG2] |= ADS1292R_config2.Int_Test   << 1;
    ADS1292R_reg[ADS1292R_CONFIG2] |= ADS1292R_config2.Clk_EN     << 3;
    ADS1292R_reg[ADS1292R_CONFIG2] |= ADS1292R_config2.Vref       << 4;
    ADS1292R_reg[ADS1292R_CONFIG2] |= ADS1292R_config2.Pdb_Refbuf << 5;
    ADS1292R_reg[ADS1292R_CONFIG2] |= ADS1292R_config2.Pdb_Loff_Comp << 6;
    ADS1292R_reg[ADS1292R_CONFIG2] |= 0x80; /* bit 7 = 1 obligatoire */

    /* LOFF (0x03) — seuil détection lead-off
     * 0xF0 = valeur par défaut recommandée par le datasheet */
    ADS1292R_reg[ADS1292R_LOFF] = 0xF0;

    /* CH1SET (0x04) — configuration canal 1 */
    ADS1292R_reg[ADS1292R_CH1SET] = 0x00;
    ADS1292R_reg[ADS1292R_CH1SET] |= ADS1292R_ch1set.MUX;
    ADS1292R_reg[ADS1292R_CH1SET] |= ADS1292R_ch1set.GAIN << 4;
    ADS1292R_reg[ADS1292R_CH1SET] |= ADS1292R_ch1set.PD   << 7;

    /* CH2SET (0x05) — configuration canal 2 */
    ADS1292R_reg[ADS1292R_CH2SET] = 0x00;
    ADS1292R_reg[ADS1292R_CH2SET] |= ADS1292R_ch2set.MUX;
    ADS1292R_reg[ADS1292R_CH2SET] |= ADS1292R_ch2set.GAIN << 4;
    ADS1292R_reg[ADS1292R_CH2SET] |= ADS1292R_ch2set.PD   << 7;

    /* RLD_SENS (0x06) — configuration RLD
     * Bits [7:6] = 11 obligatoires (valeur par défaut) */
    ADS1292R_reg[ADS1292R_RLD_SENS] = 0x00;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= ADS1292R_rld_sens.Rld1P;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= ADS1292R_rld_sens.Rld1N          << 1;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= ADS1292R_rld_sens.Rld2P          << 2;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= ADS1292R_rld_sens.Rld2N          << 3;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= ADS1292R_rld_sens.Rld_Loff_Sense << 4;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= ADS1292R_rld_sens.Pdb_Rld        << 5;
    ADS1292R_reg[ADS1292R_RLD_SENS] |= 0xC0; /* bits [7:6] = 11 obligatoires */

    /* LOFF_SENS (0x07) — détection lead-off */
    ADS1292R_reg[ADS1292R_LOFF_SENS] = 0x00;
    ADS1292R_reg[ADS1292R_LOFF_SENS] |= ADS1292R_loff_sens.Loff1P;
    ADS1292R_reg[ADS1292R_LOFF_SENS] |= ADS1292R_loff_sens.Loff1N << 1;
    ADS1292R_reg[ADS1292R_LOFF_SENS] |= ADS1292R_loff_sens.Loff2P << 2;
    ADS1292R_reg[ADS1292R_LOFF_SENS] |= ADS1292R_loff_sens.Loff2N << 3;
    ADS1292R_reg[ADS1292R_LOFF_SENS] |= ADS1292R_loff_sens.Flip1  << 4;
    ADS1292R_reg[ADS1292R_LOFF_SENS] |= ADS1292R_loff_sens.Flip2  << 5;

    /* LOFF_STAT (0x08) — statut lead-off (lecture seule · on écrit 0) */
    ADS1292R_reg[ADS1292R_LOFF_STAT] = 0x00;

    /* RESP1 (0x09) — contrôle respiration 1
     * 0x02 = valeur par défaut recommandée datasheet pour notre config */
    ADS1292R_reg[ADS1292R_RESP1] = 0x00;
    ADS1292R_reg[ADS1292R_RESP1] |= ADS1292R_resp1.RESP_Ctrl;
    ADS1292R_reg[ADS1292R_RESP1] |= ADS1292R_resp1.RESP_ph      << 2;
    ADS1292R_reg[ADS1292R_RESP1] |= ADS1292R_resp1.RESP_modEN   << 6;
    ADS1292R_reg[ADS1292R_RESP1] |= ADS1292R_resp1.RESP_DemodEN << 7;
    ADS1292R_reg[ADS1292R_RESP1]  = 0x02; /* forcer valeur par défaut */

    /* RESP2 (0x0A) — contrôle respiration 2
     * 0x01 = valeur par défaut recommandée datasheet */
    ADS1292R_reg[ADS1292R_RESP2] = 0x00;
    ADS1292R_reg[ADS1292R_RESP2] |= ADS1292R_resp2.Rldref_Int << 1;
    ADS1292R_reg[ADS1292R_RESP2] |= ADS1292R_resp2.freq       << 2;
    ADS1292R_reg[ADS1292R_RESP2] |= ADS1292R_resp2.Calib      << 7;
    ADS1292R_reg[ADS1292R_RESP2]  = 0x01; /* forcer valeur par défaut */

    /* GPIO (0x0B) — configuration des broches GPIO internes du capteur
     * bits [7:4] = 0000 obligatoire
     * bits [3:2] = 11   → GPIO configurées en entrée
     * bits [1:0] = statut GPIO en lecture */
    ADS1292R_reg[ADS1292R_GPIO] = 0x0C;
}

/*
 * ADS1292R_SPI_TransmitReceive — Envoyer et recevoir 1 octet via SPI2
 *
 * Envoie tx_data sur MOSI et retourne simultanément l'octet reçu sur MISO.
 * Utilise hspi2 (SPI2 de notre STM32H7A3 — CPOL=0 · CPHA=0 · 8.75 MHz).
 *
 * Note : le mutex MTX_SPI2 doit être pris par la tâche appelante AVANT
 *        d'appeler cette fonction.
 */
uint8_t ADS1292R_SPI_TransmitReceive(uint8_t tx_data)
{
    uint8_t rx_data = 0;
    /* hspi2 = notre SPI2 configuré dans CubeMX
     * timeout 0xFFFF = 65535ms — suffisant pour tout transfert SPI */
    HAL_SPI_TransmitReceive(&hspi2, &tx_data, &rx_data, 1, 0xFFFF);
    return rx_data;
}

/*
 * ADS1292R_SendCmd — Envoyer une commande simple au capteur
 *
 * Protocole : CS bas · délai 1ms · envoyer 1 octet · délai 1ms · CS haut
 * Le délai 1ms est requis par le datasheet pour les commandes système.
 *
 * Retourne la réponse SPI (généralement non significatif pour les commandes).
 */
uint8_t ADS1292R_SendCmd(uint8_t cmd)
{
    ADS1292R_CS_LOW;
    HAL_Delay(1);
    uint8_t rx_data = ADS1292R_SPI_TransmitReceive(cmd);
    HAL_Delay(1);
    ADS1292R_CS_HIGH;
    return rx_data;
}

/*
 * ADS1292R_RwReg — Lire ou écrire un registre interne
 *
 * Protocole lecture (cmd = ADS1292R_RREG | adresse) :
 *   CS bas · envoyer cmd · envoyer 0x00 (nb reg -1) · lire 1 octet · CS haut
 *
 * Protocole écriture (cmd = ADS1292R_WREG | adresse) :
 *   CS bas · envoyer cmd · envoyer 0x00 (nb reg -1) · envoyer data · CS haut
 */
uint8_t ADS1292R_RwReg(uint8_t cmd, uint8_t data)
{
    uint8_t rx_data = 0;
    ADS1292R_CS_LOW;
    HAL_Delay(1);
    ADS1292R_SPI_TransmitReceive(cmd);    /* envoyer commande R/W + adresse */
    ADS1292R_SPI_TransmitReceive(0x00);   /* envoyer nombre de registres - 1 = 0 */
    if ((cmd & ADS1292R_RREG) == ADS1292R_RREG) {
        /* lecture : envoyer dummy · recevoir valeur du registre */
        rx_data = ADS1292R_SPI_TransmitReceive(0x00);
    } else {
        /* écriture : envoyer la valeur à écrire */
        rx_data = ADS1292R_SPI_TransmitReceive(data);
    }
    HAL_Delay(1);
    ADS1292R_CS_HIGH;
    return rx_data;
}

/*
 * ADS1292R_WriteMultiRegs — Écrire plusieurs registres consécutifs
 *
 * Protocole : CS bas · envoyer WREG|adresse · envoyer (len-1) · envoyer len octets · CS haut
 * Utilisé dans Init() pour configurer tous les registres en une seule transaction.
 *
 * param reg  : adresse du premier registre
 * param data : pointeur vers les données à écrire
 * param len  : nombre de registres à écrire
 */
void ADS1292R_WriteMultiRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    ADS1292R_CS_LOW;
    HAL_Delay(1);
    ADS1292R_SPI_TransmitReceive(ADS1292R_WREG | reg); /* commande écriture + adresse */
    HAL_Delay(1);
    ADS1292R_SPI_TransmitReceive(len - 1);             /* nombre de registres - 1 */
    for (i = 0; i < len; i++) {
        HAL_Delay(1);
        ADS1292R_SPI_TransmitReceive(*data);
        data++;
    }
    HAL_Delay(1);
    ADS1292R_CS_HIGH;
}

/*
 * ADS1292R_ReadMultiRegs — Lire plusieurs registres consécutifs
 *
 * Protocole : CS bas · envoyer RREG|adresse · envoyer (len-1) · lire len octets · CS haut
 *
 * param reg  : adresse du premier registre
 * param data : pointeur vers le buffer de réception
 * param len  : nombre de registres à lire
 */
void ADS1292R_ReadMultiRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    ADS1292R_CS_LOW;
    HAL_Delay(1);
    ADS1292R_SPI_TransmitReceive(ADS1292R_RREG | reg); /* commande lecture + adresse */
    HAL_Delay(1);
    ADS1292R_SPI_TransmitReceive(len - 1);             /* nombre de registres - 1 */
    for (i = 0; i < len; i++) {
        HAL_Delay(1);
        *data = ADS1292R_SPI_TransmitReceive(0x00);    /* correction bug original : data= → *data= */
        data++;
    }
    HAL_Delay(1);
    ADS1292R_CS_HIGH;
}

/*
 * ADS1292R_GetValue — Lire 9 octets depuis le capteur en mode RDATAC
 *
 * En mode RDATAC (Read Data Continuously), le capteur envoie automatiquement
 * 9 octets à chaque nouvelle conversion (signalé par DRDY=0).
 *
 * Structure des 9 octets reçus dans ADS1292R_data_buf :
 *   [0..2] : 24 bits status = 1100 + LOFF_STAT[4:0] + GPIO[1:0] + 13×0
 *   [3..5] : 24 bits CH1 en complément à 2 (MSB en premier)
 *   [6..8] : 24 bits CH2 en complément à 2 (MSB en premier)
 *
 * Appel depuis vTaskEMGAcq après notification DRDY :
 *   xSemaphoreTake(MTX_SPI2, pdMS_TO_TICKS(1));
 *   ADS1292R_GetValue();
 *   xSemaphoreGive(MTX_SPI2);
 */
void ADS1292R_GetValue(void)
{
    ADS1292R_CS_LOW;
    /* lire 9 octets d'un coup — ADS1292R_tmp contient 9 octets nuls (dummy TX) */
    HAL_SPI_TransmitReceive(&hspi2,
                             (uint8_t *)ADS1292R_tmp,
                             ADS1292R_data_buf,
                             9,
                             0xFF);
    ADS1292R_CS_HIGH;
}

/*
 * ADS1292R_Init — Initialiser le capteur ADS1292R
 *
 * Séquence d'initialisation basée sur le datasheet ADS1292R Section 9.4 :
 *   1. Configurer le tableau de registres (SetBuffer)
 *   2. Reset matériel via broche RESET
 *   3. Envoyer commande SDATAC (arrêter mode continu si actif)
 *   4. Envoyer commande RESET logiciel
 *   5. Vérifier l'ID du capteur avec timeout (5 tentatives)
 *   6. Écrire tous les registres de configuration
 *
 * MODIFICATION v2 : retourne uint8_t — 1=succès · 0=capteur non détecté
 * MODIFICATION v2 : boucle while sans fin → remplacée par 5 tentatives max
 *
 * Retourne : 1 si capteur détecté et initialisé · 0 si erreur
 */
uint8_t ADS1292R_Init(void)
{
    uint8_t device_id = 0;
    uint8_t retries   = 0;      /* compteur de tentatives — NOUVEAU */

    /* Étape 1 : préparer le tableau des valeurs de registres */
    ADS1292R_SetBuffer();

    /* Étape 2 : séquence de reset matériel via broche RESET
     * Datasheet Section 9.4.1 : RESET doit être bas au minimum 2 cycles d'horloge */
    ADS1292R_CS_HIGH;          /* désélectionner le capteur pendant le reset */
    ADS1292R_START_LOW;        /* arrêter les conversions */
    ADS1292R_RES_HIGH;         /* état initial RESET = haut */
    HAL_Delay(10);
    ADS1292R_RES_LOW;          /* pulse reset : mettre bas */
    HAL_Delay(10);
    ADS1292R_RES_HIGH;         /* relâcher reset : remettre haut */
    HAL_Delay(10);
    ADS1292R_RES_LOW;          /* second pulse reset pour s'assurer */
    HAL_Delay(1);
    ADS1292R_RES_HIGH;
    HAL_Delay(100);            /* attendre stabilisation après reset */

    /* Étape 3 : arrêter le mode lecture continu si actif au démarrage */
    ADS1292R_SendCmd(ADS1292R_SDATAC);
    HAL_Delay(100);

    /* Étape 4 : reset logiciel via commande SPI */
    ADS1292R_SendCmd(ADS1292R_ADSRESET);
    HAL_Delay(1000);           /* attendre 1 seconde après reset logiciel */

    /* Étape 5 : arrêter à nouveau le mode continu après reset logiciel */
    ADS1292R_SendCmd(ADS1292R_SDATAC);
    HAL_Delay(100);

    /* Étape 6 : vérifier l'ID du capteur
     * MODIFICATION : remplace la boucle while() infinie par 5 tentatives max
     * Évite de bloquer le scheduler FreeRTOS si le capteur est absent */
    while (device_id != DEVICE_ID_ADS1292R && retries < 5)
    {
        device_id = ADS1292R_RwReg(ADS1292R_RREG | ADS1292R_ID, 0x00);
        HAL_Delay(200);
        retries++;
    }

    /* Si après 5 tentatives le capteur ne répond pas → retourner échec */
    if (device_id != DEVICE_ID_ADS1292R)
    {
        return 0; /* capteur non détecté — vérifier câblage SPI et alimentation */
    }

    /* Étape 7 : écrire tous les registres de configuration
     * ADS1292R_reg + 1 : on commence à CONFIG1 (adresse 0x01)
     * 11 registres : CONFIG1 à GPIO (0x01 à 0x0B) */
    ADS1292R_WriteMultiRegs(ADS1292R_CONFIG1, (uint8_t *)ADS1292R_reg + 1, 11);

    return 1; /* initialisation réussie */
}

/*
 * ADS1292R_ADCStartNormal — Démarrer les conversions en mode normal
 *
 * Configure le capteur avec les paramètres par défaut de notre projet
 * puis active le mode RDATAC (Read Data Continuously).
 * Après cet appel, le capteur génère une impulsion DRDY toutes les 2ms
 * (500 Hz) → réveille vTaskEMGAcq via l'EXTI PE12.
 */
void ADS1292R_ADCStartNormal(void)
{
    ADS1292R_SendCmd(ADS1292R_SDATAC); /* s'assurer que le mode continu est arrêté */
    HAL_Delay(100);
    ADS1292R_SetBuffer();
    ADS1292R_WriteMultiRegs(ADS1292R_CONFIG1, (uint8_t *)ADS1292R_reg + 1, 11);
    HAL_Delay(10);
    ADS1292R_SendCmd(ADS1292R_RDATAC); /* activer le mode lecture continue */
    HAL_Delay(10);
    ADS1292R_SendCmd(ADS1292R_START);  /* démarrer les conversions ADC */
    HAL_Delay(10);
}


void ADS1292R_Decode(uint8_t *raw_buf, ADS1292R_Sample_t *sample, uint32_t time)
{
/* uint8_t *raw_buf : C'est le tableau ADS1292R_data_buf[9].
	   Il vient directement du capteur via la fonction ADS1292R_GetValue().
       Les octets [0, 1, 2] sont le statut (on les ignore ici).
       Les octets [3, 4, 5] sont les données du Muscle A (24 bits).
       Les octets [6, 7, 8] sont les données du Muscle B (24 bits).
*/




    // Canal 1 (octets 3, 4, 5)
    int32_t val1 = (int32_t)((raw_buf[3] << 16) | (raw_buf[4] << 8) | raw_buf[5]);
    if (val1 & 0x800000) val1 |= 0xFF000000; // Extension de signe si négatif
    sample->ch1 = val1;

    // Canal 2 (octets 6, 7, 8)
    int32_t val2 = (int32_t)((raw_buf[6] << 16) | (raw_buf[7] << 8) | raw_buf[8]);
    if (val2 & 0x800000) val2 |= 0xFF000000; // Extension de signe si négatif
    sample->ch2 = val2;

    /*Sur 24 bits, le bit de signe est le bit 23.
Sur 32 bits (le STM32), le bit de signe est le bit 31.
Si le nombre est négatif sur 24 bits, nous devons "remplir"
les 8 bits vides (de 24 à 31) avec des 1 pour que le STM32 comprenne que le nombre est toujours négatif.*/


    sample->timestamp = time;
}

/*
 * ADS1292R_ADCStartTest — Démarrer en mode test interne
 *
 * Configure le capteur pour générer un signal carré 1Hz en interne.
 * Utile pour vérifier que le bus SPI et les calculs DSP fonctionnent
 * correctement avant de connecter les électrodes.
 */
void ADS1292R_ADCStartTest(void)
{
    /* activer le signal test interne 1Hz carré */
    ADS1292R_config2.Int_Test  = INT_TEST_ON;
    ADS1292R_config2.Test_Freq = TEST_FREQ_AC;

    /* configurer les deux canaux en mode test */
    ADS1292R_ch1set.PD   = PD_ON;
    ADS1292R_ch1set.GAIN = GAIN_6;
    ADS1292R_ch1set.MUX  = MUX_Test_signal;
    ADS1292R_ch2set.PD   = PD_ON;
    ADS1292R_ch2set.GAIN = GAIN_6;
    ADS1292R_ch2set.MUX  = MUX_Test_signal;

    /* désactiver la respiration en mode test */
    ADS1292R_resp1.RESP_DemodEN = RESP_DEMOD_OFF;
    ADS1292R_resp1.RESP_modEN   = RESP_MOD_OFF;
    ADS1292R_resp1.RESP_ph      = 0x00;
    ADS1292R_resp1.RESP_Ctrl    = RESP_CTRL_CLOCK_EXTERNAL;

    ADS1292R_SendCmd(ADS1292R_SDATAC); /* arrêter le mode continu */
    HAL_Delay(100);
    ADS1292R_SetBuffer();
    ADS1292R_WriteMultiRegs(ADS1292R_CONFIG1, (uint8_t *)ADS1292R_reg + 1, 11);
    HAL_Delay(10);
    ADS1292R_SendCmd(ADS1292R_RDATAC); /* activer le mode lecture continue */
    HAL_Delay(10);
    ADS1292R_SendCmd(ADS1292R_START);  /* démarrer les conversions */
    HAL_Delay(10);
}
