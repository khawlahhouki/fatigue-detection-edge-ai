/* =============================================================
 * ppg_features.c
 *
 * Implémentation des 8 features PPG
 * Voir ppg_features.h pour la documentation complète
 * ============================================================= */

#include "ppg_features.h"
#include <math.h>     /* sqrtf, fabsf, powf */
#include <string.h>   /* memset             */

/* =============================================================
 * FONCTIONS UTILITAIRES PRIVÉES (static)
 * ============================================================= */

/* -------------------------------------------------------------
 * calc_mean — moyenne d'un tableau float
 * ------------------------------------------------------------- */
static float calc_mean(const float *arr, uint8_t N)
{
    if (N == 0) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < N; i++) sum += arr[i];
    return sum / (float)N;
}

/* -------------------------------------------------------------
 * calc_variance — variance d'un tableau float
 * ------------------------------------------------------------- */
static float calc_variance(const float *arr, uint8_t N)
{
    if (N < 2) return 0.0f;
    float moy = calc_mean(arr, N);
    float var = 0.0f;
    for (uint8_t i = 0; i < N; i++)
    {
        float diff = arr[i] - moy;
        var += diff * diff;
    }
    return var / (float)N;
}

/* -------------------------------------------------------------
 * calc_std — écart-type d'un tableau float
 * ------------------------------------------------------------- */
static float calc_std(const float *arr, uint8_t N)
{
    return sqrtf(calc_variance(arr, N));
}

/* =============================================================
 * PPG_DetectPeaks
 * ============================================================= */
uint8_t PPG_DetectPeaks(const float *sig,
                         uint16_t     N,
                         PPG_RR_t    *result)
{
    if (sig == NULL || result == NULL || N < 10)
        return 0;

    /* Initialiser la structure                                  */
    memset(result, 0, sizeof(PPG_RR_t));

    /* --- Étape 1 : calculer moyenne et max du signal --- */
    float sum = 0.0f;
    float max_val = sig[0];
    for (uint16_t i = 0; i < N; i++)
    {
        sum += sig[i];
        if (sig[i] > max_val) max_val = sig[i];
    }
    float mean_val = sum / (float)N;

    /* --- Étape 2 : calculer le seuil adaptatif ---
     * seuil = moy + PPG_PEAK_THRESHOLD × (max - moy)
     * Avec PPG_PEAK_THRESHOLD = 0.3 :
     *   Si max=1.0, moy=0.0 → seuil=0.30
     *   Si max=0.5, moy=0.1 → seuil=0.22
     * Le seuil s'adapte à chaque patient                        */
    float threshold = mean_val
                    + PPG_PEAK_THRESHOLD * (max_val - mean_val);

    /* --- Étape 3 : détecter les pics ---
     * Un pic est valide si :
     *   - c'est un maximum local (plus grand que ses 2 voisins)
     *   - il dépasse le seuil
     *   - il est à au moins PPG_PEAK_MIN_DIST samples du dernier
     * On commence à i=2 et finit à N-2 pour avoir des voisins  */
    uint16_t last_peak = 0;
    uint8_t  n = 0;

    for (uint16_t i = 2; i < N - 2; i++)
    {
        /* Maximum local : plus grand que les 2 voisins de chaque côté */
        if (sig[i] <= sig[i-1]) continue;
        if (sig[i] <= sig[i+1]) continue;
        if (sig[i] <= sig[i-2]) continue;
        if (sig[i] <= sig[i+2]) continue;

        /* Au-dessus du seuil                                    */
        if (sig[i] < threshold) continue;

        /* Distance minimale depuis le dernier pic               */
        if (n > 0 && (i - last_peak) < PPG_PEAK_MIN_DIST) continue;

        /* Pic valide — stocker                                  */
        if (n < PPG_MAX_PEAKS)
        {
            result->peaks[n] = i;
            last_peak = i;
            n++;
        }
    }

    result->n_peaks = n;
    return n;
}

/* =============================================================
 * PPG_CalcRR
 * ============================================================= */
uint8_t PPG_CalcRR(PPG_RR_t *result)
{
    if (result == NULL || result->n_peaks < 2)
        return 0;

    uint8_t n = result->n_peaks - 1;

    /* rr_ms[i] = (peaks[i+1] - peaks[i]) × (1000ms / 400Hz)
     * = différence en samples × 2.5ms                          */
    for (uint8_t i = 0; i < n; i++)
    {
        float diff_samples = (float)(result->peaks[i+1]
                                   - result->peaks[i]);
        result->rr_ms[i] = diff_samples * (1000.0f / PPG_FS);
    }

    /* Filtrer les intervalles physiologiquement impossibles
     * BPM min = 30  → RR max = 2000ms
     * BPM max = 200 → RR min = 300ms                           */
    uint8_t n_valid = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        if (result->rr_ms[i] >= 300.0f &&
            result->rr_ms[i] <= 2000.0f)
        {
            result->rr_ms[n_valid] = result->rr_ms[i];
            n_valid++;
        }
    }

    result->n_rr = n_valid;
    return n_valid;
}

/* =============================================================
 * PPG_CalcBPM
 * ============================================================= */
float PPG_CalcBPM(const PPG_RR_t *result)
{
    /* Besoin d'au moins 2 intervalles pour un BPM fiable        */
    if (result == NULL || result->n_rr < 2)
        return 0.0f;

    float moy = calc_mean(result->rr_ms, result->n_rr);

    if (moy < PPG_EPSILON) return 0.0f;

    /* BPM = 60000ms / intervalle_moyen_ms                      */
    float bpm = 60000.0f / moy;

    /* Clamp physiologique : 30 à 220 BPM                       */
    if (bpm < 30.0f)  bpm = 30.0f;
    if (bpm > 220.0f) bpm = 220.0f;

    return bpm;
}

/* =============================================================
 * PPG_CalcSDNN
 * ============================================================= */
float PPG_CalcSDNN(const PPG_RR_t *result)
{
    /* SDNN nécessite au moins 3 intervalles                     */
    if (result == NULL || result->n_rr < 3)
        return 0.0f;

    /* SDNN = écart-type des intervalles R-R
     * σ = √( Σ(RR[i] - moy)² / N )                            */
    return calc_std(result->rr_ms, result->n_rr);
}

/* =============================================================
 * PPG_CalcRMSSD
 * ============================================================= */
float PPG_CalcRMSSD(const PPG_RR_t *result)
{
    /* RMSSD nécessite au moins 2 intervalles                    */
    if (result == NULL || result->n_rr < 2)
        return 0.0f;

    float sum_sq = 0.0f;
    uint8_t n = result->n_rr - 1;

    /* RMSSD = √( Σ(RR[i+1] - RR[i])² / (N-1) )               */
    for (uint8_t i = 0; i < n; i++)
    {
        float diff = result->rr_ms[i+1] - result->rr_ms[i];
        sum_sq += diff * diff;
    }

    return sqrtf(sum_sq / (float)n);
}

/* =============================================================
 * PPG_CalcPNN50
 * ============================================================= */
float PPG_CalcPNN50(const PPG_RR_t *result)
{
    if (result == NULL || result->n_rr < 2)
        return 0.0f;

    uint8_t count = 0;
    uint8_t n = result->n_rr - 1;

    /* Compter les différences successives > 50ms               */
    for (uint8_t i = 0; i < n; i++)
    {
        float diff = fabsf(result->rr_ms[i+1] - result->rr_ms[i]);
        if (diff > 50.0f) count++;
    }

    /* pNN50 en pourcentage                                      */
    return (float)count / (float)n * 100.0f;
}

/* =============================================================
 * PPG_CalcLFHF
 * ============================================================= */
float PPG_CalcLFHF(const PPG_RR_t *result)
{
    /* LF/HF nécessite au moins 2×PPG_LFHF_SMOOTH_WIN+1 intervalles */
    uint8_t min_rr = 2 * PPG_LFHF_SMOOTH_WIN + 1;
    if (result == NULL || result->n_rr < min_rr)
        return 1.0f; /* valeur neutre si pas assez de données    */

    uint8_t N = result->n_rr;

    /* --- Étape 1 : lisser les RR (filtre passe-bas = composante LF) ---
     * Moyenne mobile de fenêtre PPG_LFHF_SMOOTH_WIN de chaque côté */
    float rr_smooth[PPG_MAX_RR];
    uint8_t k = PPG_LFHF_SMOOTH_WIN;

    for (uint8_t i = 0; i < N; i++)
    {
        float sum = 0.0f;
        uint8_t count = 0;
        /* Fenêtre centrée [i-k, i+k] avec bord clipé           */
        int16_t start = (int16_t)i - k;
        int16_t end   = (int16_t)i + k;
        if (start < 0)    start = 0;
        if (end   >= N)   end   = N - 1;

        for (int16_t j = start; j <= end; j++)
        {
            sum += result->rr_ms[j];
            count++;
        }
        rr_smooth[i] = (count > 0) ? sum / (float)count : result->rr_ms[i];
    }

    /* --- Étape 2 : composante HF = RR brut - RR lissé ---      */
    float rr_hf[PPG_MAX_RR];
    for (uint8_t i = 0; i < N; i++)
    {
        rr_hf[i] = result->rr_ms[i] - rr_smooth[i];
    }

    /* --- Étape 3 : ratio des variances ---                     */
    float var_lf = calc_variance(rr_smooth, N);
    float var_hf = calc_variance(rr_hf,     N);

    if (var_hf < PPG_EPSILON) return 2.0f; /* HF quasi-nulle → LF dominant */

    float lf_hf = var_lf / var_hf;

    /* Clamp : valeur normale 0.5 à 10.0                         */
    if (lf_hf < 0.1f)  lf_hf = 0.1f;
    if (lf_hf > 20.0f) lf_hf = 20.0f;

    return lf_hf;
}

/* =============================================================
 * PPG_CalcSpO2
 * ============================================================= */
float PPG_CalcSpO2(const float *ir,
                   const float *red,
                   uint16_t     N)
{
    if (ir == NULL || red == NULL || N < 10)
        return 0.0f;

    /* --- Calculer AC et DC pour les deux canaux ---
     * DC = moyenne du signal (composante continue)
     * AC = écart-type × 2 ≈ amplitude pulsatile               */
    float dc_ir  = 0.0f, dc_red = 0.0f;
    float max_ir = ir[0],  min_ir = ir[0];
    float max_red = red[0], min_red = red[0];

    for (uint16_t i = 0; i < N; i++)
    {
        dc_ir  += ir[i];
        dc_red += red[i];
        if (ir[i]  > max_ir)  max_ir  = ir[i];
        if (ir[i]  < min_ir)  min_ir  = ir[i];
        if (red[i] > max_red) max_red = red[i];
        if (red[i] < min_red) min_red = red[i];
    }
    dc_ir  /= (float)N;
    dc_red /= (float)N;

    float ac_ir  = max_ir  - min_ir;
    float ac_red = max_red - min_red;

    /* Protection division par zéro                              */
    if (dc_ir  < PPG_EPSILON) return 0.0f;
    if (dc_red < PPG_EPSILON) return 0.0f;
    if (ac_ir  < PPG_EPSILON) return 0.0f;

    /* --- Ratio R ---
     * R = (AC_red / DC_red) / (AC_ir / DC_ir)                 */
    float R = (ac_red / dc_red) / (ac_ir / dc_ir);

    /* --- Formule empirique SpO2 ---
     * SpO2 = 110 - 25×R
     * R=0.4 → SpO2=100%  R=1.0 → SpO2=85%                    */
    float spo2 = 110.0f - 25.0f * R;

    /* Clamp physiologique : 70% à 100%                         */
    if (spo2 < 70.0f)  spo2 = 70.0f;
    if (spo2 > 100.0f) spo2 = 100.0f;

    return spo2;
}

/* =============================================================
 * PPG_CalcAmplitude
 * ============================================================= */
float PPG_CalcAmplitude(const float *sig, uint16_t N)
{
    if (sig == NULL || N == 0) return 0.0f;

    float max_v = sig[0];
    float min_v = sig[0];

    for (uint16_t i = 1; i < N; i++)
    {
        if (sig[i] > max_v) max_v = sig[i];
        if (sig[i] < min_v) min_v = sig[i];
    }

    return max_v - min_v;
}

/* =============================================================
 * PPG_CalcSkewness
 * ============================================================= */
float PPG_CalcSkewness(const float *sig, uint16_t N)
{
    if (sig == NULL || N < 3) return 0.0f;

    float moy = 0.0f;
    for (uint16_t i = 0; i < N; i++) moy += sig[i];
    moy /= (float)N;

    /* Calculer écart-type et moment d'ordre 3                   */
    float sum2 = 0.0f;  /* Pour l'écart-type                     */
    float sum3 = 0.0f;  /* Pour le moment d'ordre 3              */

    for (uint16_t i = 0; i < N; i++)
    {
        float diff = sig[i] - moy;
        float diff2 = diff * diff;
        sum2 += diff2;
        sum3 += diff2 * diff;  /* diff³ = diff² × diff           */
    }

    float variance = sum2 / (float)N;
    float std_dev  = sqrtf(variance);

    /* Protection division par zéro                              */
    if (std_dev < PPG_EPSILON) return 0.0f;

    /* skewness = moment_3 / std³                                */
    float std3 = std_dev * std_dev * std_dev;
    float skewness = (sum3 / (float)N) / std3;

    /* Clamp : valeur normale entre -3 et +3                     */
    if (skewness < -3.0f) skewness = -3.0f;
    if (skewness >  3.0f) skewness =  3.0f;

    return skewness;
}
