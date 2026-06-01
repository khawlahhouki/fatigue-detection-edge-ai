/* temp_features.c */
#include "temp_features.h"

/* =============================================================
 * Temp_Calc_Mean
 *
 * Moyenne temperature peau sur la fenetre
 *
 * Formule :
 *   mean = (1/N) x Σ skin[i]
 *
 * Interpretation fatigue :
 *   Effort physique  → vasodilatation → temperature monte
 *   Repos            → temperature stable ~33-35°C
 *   Fatigue intense  → temperature peut baisser
 *                      (vasoconstriction peripherique)
 * ============================================================= */
float Temp_Calc_Mean(const float *skin_buf, uint8_t N)
{
    if(skin_buf == NULL || N == 0) return 0.0f;

    float sum = 0.0f;
    for(uint8_t i = 0; i < N; i++)
        sum += skin_buf[i];

    return sum / (float)N;
}

/* =============================================================
 * Temp_Calc_Delta
 *
 * Ecart temperature peau - temperature ambiante
 *
 * Formule :
 *   delta = mean(skin) - mean(ambient)
 *
 * Interpretation fatigue :
 *   Delta normal au repos    : ~5-8°C
 *   Delta effort physique    : augmente (peau se rechauffe)
 *   Delta fatigue prolongee  : diminue (regulation thermique
 *                              perturbee)
 * ============================================================= */
float Temp_Calc_Delta(const float *skin_buf,
                      const float *ambient_buf,
                      uint8_t N)
{
    if(skin_buf == NULL || ambient_buf == NULL || N == 0)
        return 0.0f;

    float sum_skin    = 0.0f;
    float sum_ambient = 0.0f;

    for(uint8_t i = 0; i < N; i++)
    {
        sum_skin    += skin_buf[i];
        sum_ambient += ambient_buf[i];
    }

    float mean_skin    = sum_skin    / (float)N;
    float mean_ambient = sum_ambient / (float)N;

    return mean_skin - mean_ambient;
}

/* =============================================================
 * Temp_Calc_Drift
 *
 * Derive temporelle de la temperature peau
 *
 * Formule :
 *   drift = (skin[N-1] - skin[0]) / duree
 *   duree = (N-1) / Fs  en secondes
 *
 * Interpretation fatigue :
 *   drift > 0 : temperature monte  → effort en cours
 *   drift < 0 : temperature baisse → recuperation/fatigue
 *   drift ≈ 0 : temperature stable → repos ou etat stable
 *
 * Unite : degres Celsius par seconde (°C/s)
 * ============================================================= */
float Temp_Calc_Drift(const float *skin_buf, uint8_t N, float fs)
{
    if(skin_buf == NULL || N < 2) return 0.0f;
    if(fs < TEMP_EPSILON) return 0.0f;

    /* Duree de la fenetre en secondes */
    float duree = (float)(N - 1) / fs;

    if(duree < TEMP_EPSILON) return 0.0f;

    /* Pente = variation totale / duree */
    return (skin_buf[N-1] - skin_buf[0]) / duree;
}

/* =============================================================
 * Temp_Calc_Var
 *
 * Variance de la temperature peau
 *
 * Formule :
 *   var = (1/N) x Σ (skin[i] - mean)²
 *
 * Interpretation fatigue :
 *   Variance faible  : temperature stable → etat stable
 *   Variance elevee  : temperature fluctuante → instabilite
 *                      thermique (signe de fatigue)
 * ============================================================= */
float Temp_Calc_Var(const float *skin_buf, uint8_t N)
{
    if(skin_buf == NULL || N < 2) return 0.0f;

    /* Calculer moyenne */
    float sum = 0.0f;
    for(uint8_t i = 0; i < N; i++)
        sum += skin_buf[i];
    float mean = sum / (float)N;

    /* Calculer variance */
    float var = 0.0f;
    for(uint8_t i = 0; i < N; i++)
    {
        float diff = skin_buf[i] - mean;
        var += diff * diff;
    }

    return var / (float)N;
}
