/* emg_features.h
 *
 * Features EMG pour detection de fatigue musculaire
 * Fs = 500 Hz (ADS1292R @ DATA_RATE_500SPS)
 *
 * Fenetres :
 *   Courte : 250 samples = 0.5s → RMS, MAV, iEMG
 *   Longue : 500 samples = 1.0s → MDF, MNF (FFT)
 *
 * Indicateurs de fatigue :
 *   MDF et MNF diminuent avec la fatigue musculaire
 *   RMS augmente puis diminue avec la fatigue
 * ============================================================= */

#ifndef EMG_FEATURES_H
#define EMG_FEATURES_H

#include <stdint.h>

/* -------------------------------------------------------------
 * Parametres
 * ------------------------------------------------------------- */
#define EMG_FS              500.0f  /* frequence echantillonnage  */
#define EMG_WIN_SHORT       250     /* fenetre courte 0.5s        */
#define EMG_WIN_LONG        500     /* fenetre longue 1.0s        */
#define EMG_FFT_SIZE        512     /* FFT taille (puissance de 2)*/
#define EMG_EPSILON         1e-6f   /* protection division / zero */
#define EMG_BUF_SIZE    1000

/* -------------------------------------------------------------
 * Fonctions features domaine temporel
 *
 * EMG_Calc_RMS  — Root Mean Square
 *   = sqrt(mean(x²))
 *   → mesure la force de contraction musculaire
 *   → augmente avec l'effort, diminue avec la fatigue extreme
 *
 * EMG_Calc_MAV  — Mean Absolute Value
 *   = mean(|x|)
 *   → similaire au RMS, moins sensible aux pics
 *   → indicateur d'activite musculaire
 *
 * EMG_Calc_iEMG — Integrated EMG
 *   = sum(|x|) / Fs
 *   → energie totale du signal sur la fenetre
 *   → augmente avec la duree de contraction
 * ------------------------------------------------------------- */
float EMG_Calc_RMS  (const float *buf, uint16_t N);
float EMG_Calc_MAV  (const float *buf, uint16_t N);
float EMG_Calc_iEMG (const float *buf, uint16_t N);

/* -------------------------------------------------------------
 * Fonctions features domaine frequentiel
 *
 * EMG_Calc_MDF  — Median Frequency
 *   = frequence qui divise le spectre en deux puissances egales
 *   → DIMINUE avec la fatigue musculaire (indicateur cle !)
 *   → passage de fibres rapides vers fibres lentes
 *
 * EMG_Calc_MNF  — Mean Frequency
 *   = somme(f * PSD(f)) / somme(PSD(f))
 *   → frequence moyenne du spectre
 *   → DIMINUE aussi avec la fatigue
 *
 * Parametres :
 *   buf : buffer signal filtre
 *   N   : nombre de samples (EMG_WIN_LONG = 500)
 *   fs  : frequence echantillonnage (500.0f)
 * ------------------------------------------------------------- */
float EMG_Calc_MDF  (const float *buf, uint16_t N, float fs);
float EMG_Calc_MNF  (const float *buf, uint16_t N, float fs);

#endif /* EMG_FEATURES_H */
