/* emg_features.c */
#include "emg_features.h"
#include <math.h>    /* sqrtf, fabsf */
#include <string.h>  /* memset       */

/* =============================================================
 * EMG_Calc_RMS
 *
 * Root Mean Square = sqrt(mean(x²))
 *
 * Formule :
 *   RMS = sqrt( (1/N) × Σ x[i]² )
 *
 * Interpretation fatigue :
 *   Contraction legere  : RMS faible
 *   Contraction forte   : RMS eleve
 *   Fatigue musculaire  : RMS diminue progressivement
 * ============================================================= */
float EMG_Calc_RMS(const float *buf, uint16_t N)
{
    if(buf == NULL || N == 0) return 0.0f;

    float sum_sq = 0.0f;
    for(uint16_t i = 0; i < N; i++)
    {
        sum_sq += buf[i] * buf[i];
    }
    return sqrtf(sum_sq / (float)N);
}

/* =============================================================
 * EMG_Calc_MAV
 *
 * Mean Absolute Value = mean(|x|)
 *
 * Formule :
 *   MAV = (1/N) × Σ |x[i]|
 *
 * Interpretation fatigue :
 *   Similaire au RMS mais plus robuste aux artefacts
 *   Moins sensible aux pics isoles
 * ============================================================= */
float EMG_Calc_MAV(const float *buf, uint16_t N)
{
    if(buf == NULL || N == 0) return 0.0f;

    float sum_abs = 0.0f;
    for(uint16_t i = 0; i < N; i++)
    {
        sum_abs += fabsf(buf[i]);
    }
    return sum_abs / (float)N;
}

/* =============================================================
 * EMG_Calc_iEMG
 *
 * Integrated EMG = sum(|x|) / Fs
 *
 * Formule :
 *   iEMG = (1/Fs) × Σ |x[i]|
 *
 * Interpretation fatigue :
 *   Energie totale du signal sur la fenetre
 *   Augmente avec la duree et la force de contraction
 * ============================================================= */
float EMG_Calc_iEMG(const float *buf, uint16_t N)
{
    if(buf == NULL || N == 0) return 0.0f;

    float sum_abs = 0.0f;
    for(uint16_t i = 0; i < N; i++)
    {
        sum_abs += fabsf(buf[i]);
    }
    return sum_abs / EMG_FS;
}

/* =============================================================
 * calc_psd_simple
 *
 * Calcule la Densite Spectrale de Puissance (PSD)
 * par methode periodogramme simplifie sans FFT complete.
 *
 * Methode : DFT selective sur les frequences EMG utiles
 *   → calcule uniquement les bins 20Hz a 200Hz
 *   → evite la FFT complete (trop lourde pour STM32)
 *   → complexite O(N × K) ou K = nombre de bins utiles
 *
 * K = (200-20) × N / Fs = 180 × 500 / 500 = 180 bins
 * ============================================================= */
static void calc_psd_simple(const float *buf, uint16_t N, float fs,
                              float *psd, float *freqs, uint16_t *n_bins)
{
    /* Calculer uniquement les bins entre 20Hz et 200Hz */
    float f_min = 20.0f;
    float f_max = 200.0f;
    float df    = fs / (float)N;  /* resolution frequentielle */

    uint16_t k_min = (uint16_t)(f_min / df);
    uint16_t k_max = (uint16_t)(f_max / df);
    if(k_max > N/2) k_max = N/2;

    *n_bins = k_max - k_min;

    for(uint16_t k = k_min; k < k_max; k++)
    {
        float re = 0.0f, im = 0.0f;
        float omega = 2.0f * 3.14159265f * (float)k / (float)N;

        for(uint16_t n = 0; n < N; n++)
        {
            re += buf[n] * cosf(omega * (float)n);
            im += buf[n] * sinf(omega * (float)n);
        }

        /* PSD = |X(k)|² / N */
        psd[k - k_min]   = (re*re + im*im) / (float)N;
        freqs[k - k_min] = (float)k * df;
    }
}

/* =============================================================
 * EMG_Calc_MDF
 *
 * Median Frequency = frequence qui divise le spectre en
 * deux puissances egales
 *
 * Algorithme :
 *   1. Calculer PSD(f) pour f dans [20Hz, 200Hz]
 *   2. Calculer puissance totale = Σ PSD(f)
 *   3. Trouver MDF tel que :
 *      Σ PSD(f≤MDF) = puissance_totale / 2
 *
 * Interpretation fatigue :
 *   Repos/contraction legere : MDF ≈ 80-120 Hz
 *   Fatigue musculaire       : MDF descend vers 40-60 Hz
 *   → glissement vers les basses frequences
 * ============================================================= */
float EMG_Calc_MDF(const float *buf, uint16_t N, float fs)
{
    if(buf == NULL || N < 10) return 0.0f;

    /* Buffers locaux — taille maximale 200 bins */
    float psd[200];
    float freqs[200];
    uint16_t n_bins = 0;

    calc_psd_simple(buf, N, fs, psd, freqs, &n_bins);
    if(n_bins == 0) return 0.0f;

    /* Puissance totale */
    float total_power = 0.0f;
    for(uint16_t i = 0; i < n_bins; i++)
        total_power += psd[i];

    if(total_power < EMG_EPSILON) return 0.0f;

    /* Trouver MDF : frequence ou puissance cumulee = total/2 */
    float half_power  = total_power / 2.0f;
    float cumul_power = 0.0f;

    for(uint16_t i = 0; i < n_bins; i++)
    {
        cumul_power += psd[i];
        if(cumul_power >= half_power)
            return freqs[i];
    }

    return freqs[n_bins - 1];
}

/* =============================================================
 * EMG_Calc_MNF
 *
 * Mean Frequency = frequence moyenne ponderee par la PSD
 *
 * Formule :
 *   MNF = Σ(f × PSD(f)) / Σ(PSD(f))
 *
 * Interpretation fatigue :
 *   Similaire a MDF mais plus sensible aux changements
 *   Diminue avec la fatigue musculaire
 *   MNF > MDF en general (asymetrie spectrale)
 * ============================================================= */
float EMG_Calc_MNF(const float *buf, uint16_t N, float fs)
{
    if(buf == NULL || N < 10) return 0.0f;

    float psd[200];
    float freqs[200];
    uint16_t n_bins = 0;

    calc_psd_simple(buf, N, fs, psd, freqs, &n_bins);
    if(n_bins == 0) return 0.0f;

    float sum_f_psd = 0.0f;
    float sum_psd   = 0.0f;

    for(uint16_t i = 0; i < n_bins; i++)
    {
        sum_f_psd += freqs[i] * psd[i];
        sum_psd   += psd[i];
    }

    if(sum_psd < EMG_EPSILON) return 0.0f;

    return sum_f_psd / sum_psd;
}
