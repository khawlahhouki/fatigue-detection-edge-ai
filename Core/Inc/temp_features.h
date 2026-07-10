/* temp_features.h
 * Features temperature pour detection de fatigue
 *
 * Capteurs :
 *   TMP117 → temperature peau (°C)
 *   SHTC3  → temperature ambiante (°C)
 *
 * Frequence : 2.54 Hz (394ms par cycle)
 * Fenetre   : 30s = 76 samples
 * Buffer    : 31s = 79 samples (marge +3 samples)
 *
 * Features :
 *   temp_mean     → temperature peau moyenne
 *   temp_delta    → ecart peau - ambiante
 *   temp_drift    → derive temporelle (C/s)
 *   temp_variance → variance temperature peau
 * ============================================================= */

#ifndef TEMP_FEATURES_H
#define TEMP_FEATURES_H

#include <stdint.h>
#include <math.h>

/* -------------------------------------------------------------
 * Parametres
 * ------------------------------------------------------------- */
#define TEMP_FS          2.54f   /* frequence echantillonnage Hz */
#define TEMP_WIN_LONG    76      /* fenetre 30s x 2.54Hz         */
#define TEMP_BUF_SIZE    79      /* buffer 31s x 2.54Hz + marge  */
#define TEMP_EPSILON     1e-6f   /* protection division / zero   */

/* -------------------------------------------------------------
 * Fonctions features temperature
 *
 * Temp_Calc_Mean :
 *   Moyenne temperature peau sur 30s
 *   = (1/N) x Σ temp_skin[i]
 *   Augmente avec effort physique (vasodilatation)
 *
 * Temp_Calc_Delta :
 *   Ecart peau - ambiante
 *   = mean(temp_skin) - mean(temp_ambient)
 *   Indicateur stress thermique
 *
 * Temp_Calc_Drift :
 *   Derive temporelle = pente de la temperature
 *   = (temp_skin[N-1] - temp_skin[0]) / duree
 *   Positive = temperature monte (effort)
 *   Negative = temperature descend (recuperation)
 *
 * Temp_Calc_Var :
 *   Variance temperature peau
 *   = (1/N) x Σ (temp[i] - mean)²
 *   Faible = temperature stable
 *   Elevee = temperature fluctuante (instabilite)
 * ------------------------------------------------------------- */
float Temp_Calc_Mean  (const float *skin_buf,    uint8_t N);
float Temp_Calc_Delta (const float *skin_buf,
                       const float *ambient_buf, uint8_t N);
float Temp_Calc_Drift (const float *skin_buf,    uint8_t N,
                       float fs);
float Temp_Calc_Var   (const float *skin_buf,    uint8_t N);

#endif /* TEMP_FEATURES_H */
