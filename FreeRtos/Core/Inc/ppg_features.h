/* =============================================================
 * ppg_features.h
 *
 * Calcul des 8 features PPG pour la détection de fatigue
 *
 * Entrée  : local_ir[8000] + local_red[8000]
 *           ordonnés chronologiquement (index 0 = plus ancien)
 *           Fs = 400 Hz — fenêtre = 20s = 8000 samples
 *
 * Sortie  : 8 valeurs float dans FeatureVector_t
 *
 * Pipeline :
 *   1. PPG_DetectPeaks()   → indices des pics cardiaques
 *   2. PPG_CalcRR()        → intervalles R-R en ms
 *   3. PPG_CalcBPM()       → fréquence cardiaque (bpm)
 *   4. PPG_CalcSDNN()      → variabilité globale (ms)
 *   5. PPG_CalcRMSSD()     → variabilité court terme (ms)
 *   6. PPG_CalcPNN50()     → proportion grandes variations (%)
 *   7. PPG_CalcLFHF()      → balance sympathique/parasympathique
 *   8. PPG_CalcSpO2()      → saturation oxygène (%)
 *   9. PPG_CalcAmplitude() → amplitude crête-à-crête
 *  10. PPG_CalcSkewness()  → asymétrie du signal
 * ============================================================= */

#ifndef PPG_FEATURES_H
#define PPG_FEATURES_H

#include <stdint.h>
#include <stddef.h>

/* =============================================================
 * CONSTANTES
 * ============================================================= */

#define PPG_FS               400.0f   /* Fréquence échantillonnage Hz  */
#define PPG_WINDOW_SAMPLES   8000     /* 20s × 400Hz                   */

/* Détection de pics */
#define PPG_PEAK_MIN_DIST    60       /* Distance minimale entre pics
                                       * = 150ms × 400Hz
                                       * = BPM max 200                 */
#define PPG_PEAK_THRESHOLD   0.3f    /* Fraction au-dessus de la moyenne
                                       * seuil = moy + 0.3×(max-moy)  */

/* Nombre maximum de pics détectables sur 20s
 * BPM max = 200 → 200/60 × 20s = 67 pics max                        */
#define PPG_MAX_PEAKS        70

/* Nombre maximum d'intervalles R-R = pics - 1                        */
#define PPG_MAX_RR           69

/* LF/HF — fenêtre de lissage pour séparation LF/HF
 * Fenêtre = 3 intervalles R-R (≈ 2–3s selon BPM)                    */
#define PPG_LFHF_SMOOTH_WIN  3

/* Protection division par zéro                                        */
#define PPG_EPSILON          1e-6f

/* =============================================================
 * STRUCTURE INTERMÉDIAIRE
 *
 * Stocke les résultats de la détection de pics et le calcul
 * des intervalles R-R. Partagée entre toutes les fonctions
 * pour éviter de recalculer plusieurs fois.
 * ============================================================= */
typedef struct {

    uint16_t peaks[PPG_MAX_PEAKS];  /* Indices des pics dans le signal  */
    uint8_t  n_peaks;               /* Nombre de pics détectés          */

    float    rr_ms[PPG_MAX_RR];    /* Intervalles R-R en millisecondes  */
    uint8_t  n_rr;                  /* Nombre d'intervalles R-R         */

} PPG_RR_t;

/* =============================================================
 * FONCTIONS PUBLIQUES
 * ============================================================= */

/* -------------------------------------------------------------
 * PPG_DetectPeaks
 *
 * Détecte les pics (battements cardiaques) dans le signal PPG.
 *
 * Algorithme :
 *   1. Calculer moyenne et max du signal
 *   2. Calculer seuil = moy + 0.3 × (max - moy)
 *   3. Pour chaque point : si maximum local > seuil
 *      et distance > PPG_PEAK_MIN_DIST → pic détecté
 *
 * Paramètres :
 *   sig    : signal PPG filtré [N samples]
 *   N      : nombre de samples
 *   result : structure PPG_RR_t à remplir
 *
 * Retour : nombre de pics détectés (0 si erreur)
 * ------------------------------------------------------------- */
uint8_t PPG_DetectPeaks(const float *sig,
                         uint16_t     N,
                         PPG_RR_t    *result);

/* -------------------------------------------------------------
 * PPG_CalcRR
 *
 * Calcule les intervalles R-R en ms à partir des indices de pics.
 * Doit être appelée après PPG_DetectPeaks.
 *
 * rr_ms[i] = (peaks[i+1] - peaks[i]) × (1000 / Fs)
 *
 * Retour : nombre d'intervalles calculés
 * ------------------------------------------------------------- */
uint8_t PPG_CalcRR(PPG_RR_t *result);

/* -------------------------------------------------------------
 * PPG_CalcBPM
 *
 * Fréquence cardiaque moyenne en battements par minute.
 * BPM = 60000 / moyenne(rr_ms)
 *
 * Retour : BPM (0.0f si pas assez de pics)
 * ------------------------------------------------------------- */
float PPG_CalcBPM(const PPG_RR_t *result);

/* -------------------------------------------------------------
 * PPG_CalcSDNN
 *
 * Écart-type de tous les intervalles R-R.
 * Mesure la variabilité globale de la fréquence cardiaque.
 * SDNN élevé = bonne variabilité = moins de fatigue.
 *
 * SDNN = √( Σ(RR[i] - moy_RR)² / N )
 *
 * Retour : SDNN en ms (0.0f si pas assez de données)
 * ------------------------------------------------------------- */
float PPG_CalcSDNN(const PPG_RR_t *result);

/* -------------------------------------------------------------
 * PPG_CalcRMSSD
 *
 * Racine carrée de la moyenne des carrés des différences
 * entre intervalles R-R successifs.
 * Mesure la variabilité à court terme (parasympathique).
 *
 * RMSSD = √( Σ(RR[i+1] - RR[i])² / (N-1) )
 *
 * Retour : RMSSD en ms
 * ------------------------------------------------------------- */
float PPG_CalcRMSSD(const PPG_RR_t *result);

/* -------------------------------------------------------------
 * PPG_CalcPNN50
 *
 * Proportion des intervalles successifs dont la différence
 * absolue dépasse 50ms.
 *
 * pNN50 = (count |RR[i+1]-RR[i]| > 50ms) / (N-1) × 100
 *
 * Retour : pNN50 en % (0.0 à 100.0)
 * ------------------------------------------------------------- */
float PPG_CalcPNN50(const PPG_RR_t *result);

/* -------------------------------------------------------------
 * PPG_CalcLFHF
 *
 * Ratio LF/HF estimé par filtrage des intervalles R-R.
 * Sans FFT — méthode par variance.
 *
 * LF (0.04-0.15 Hz) = composante lente (sympathique)
 * HF (0.15-0.4  Hz) = composante rapide (parasympathique)
 *
 * Méthode :
 *   rr_smooth = moyenne mobile (filtre passe-bas)
 *   rr_hf     = rr - rr_smooth (composante haute fréquence)
 *   LF/HF     = var(rr_smooth) / var(rr_hf)
 *
 * Retour : ratio LF/HF (valeur normale : 1.0 à 2.0 au repos)
 * ------------------------------------------------------------- */
float PPG_CalcLFHF(const PPG_RR_t *result);

/* -------------------------------------------------------------
 * PPG_CalcSpO2
 *
 * Saturation en oxygène par ratio AC/DC des deux canaux.
 *
 * R     = (AC_red / DC_red) / (AC_ir / DC_ir)
 * SpO2  = 110.0 - 25.0 × R  [formule empirique]
 *
 * Paramètres :
 *   ir  : signal canal infrarouge  [N samples]
 *   red : signal canal rouge       [N samples]
 *   N   : nombre de samples
 *
 * Retour : SpO2 en % (clampé entre 70.0 et 100.0)
 * ------------------------------------------------------------- */
float PPG_CalcSpO2(const float *ir,
                   const float *red,
                   uint16_t     N);

/* -------------------------------------------------------------
 * PPG_CalcAmplitude
 *
 * Amplitude crête-à-crête du signal PPG.
 * amplitude = max(signal) - min(signal)
 *
 * Retour : amplitude (unité ADC normalisée)
 * ------------------------------------------------------------- */
float PPG_CalcAmplitude(const float *sig, uint16_t N);

/* -------------------------------------------------------------
 * PPG_CalcSkewness
 *
 * Asymétrie du signal (moment statistique d'ordre 3).
 * Signal PPG normal : skewness légèrement positif.
 * Fatigue / artefact : skewness change de signe ou augmente.
 *
 * skewness = Σ( (sig[i]-moy)³ ) / ( N × std³ )
 *
 * Retour : skewness (sans unité)
 * ------------------------------------------------------------- */
float PPG_CalcSkewness(const float *sig, uint16_t N);

#endif /* PPG_FEATURES_H */
