/* imu_features.c
 *
 * Voir imu_features.h pour la description de chaque feature.
 * Toutes les fonctions travaillent sur la MAGNITUDE du vecteur
 * (sqrt(x^2+y^2+z^2)), indépendante de l'orientation du capteur.
 */

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "imu_features.h"

/* ---------------------------------------------------------------
 * Fonction interne : calcule la magnitude de chaque échantillon
 * et la range dans out[]. n <= 104 dans notre usage (buffer local
 * static côté appelant, donc pas de VLA ici : taille max fixée
 * par le buffer statique de l'appelant, mais on reste générique
 * avec une petite pile locale bornée à 208 par sécurité).
 * ------------------------------------------------------------- */
#define IMU_FEATURES_MAX_N 208

static uint16_t IMU_Compute_Magnitudes(float data[][3], uint16_t n, float *out)
{
    if (n > IMU_FEATURES_MAX_N)
    {
        n = IMU_FEATURES_MAX_N; /* sécurité : ne jamais déborder out[] */
    }

    for (uint16_t i = 0; i < n; i++)
    {
        float x = data[i][0];
        float y = data[i][1];
        float z = data[i][2];
        out[i] = sqrtf(x * x + y * y + z * z);
    }
    return n;
}

float IMU_Calc_RMS(float accel[][3], uint16_t n)
{
    if (n == 0) return 0.0f;

    static float mag[IMU_FEATURES_MAX_N];
    n = IMU_Compute_Magnitudes(accel, n, mag);

    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        sum_sq += mag[i] * mag[i];
    }

    return sqrtf(sum_sq / (float)n);
}

float IMU_Calc_Var(float accel[][3], uint16_t n)
{
    if (n == 0) return 0.0f;

    static float mag[IMU_FEATURES_MAX_N];
    n = IMU_Compute_Magnitudes(accel, n, mag);

    /* Moyenne */
    float mean = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        mean += mag[i];
    }
    mean /= (float)n;

    /* Variance = mean((x - mean)^2) */
    float var = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        float d = mag[i] - mean;
        var += d * d;
    }
    var /= (float)n;

    return var;
}

float IMU_Calibrate_PeakThreshold(float accel[][3], uint16_t n, float k)
{
    if (n == 0) return 0.0f;

    static float mag[IMU_FEATURES_MAX_N];
    n = IMU_Compute_Magnitudes(accel, n, mag);

    float mean = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        mean += mag[i];
    }
    mean /= (float)n;

    float var = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        float d = mag[i] - mean;
        var += d * d;
    }
    var /= (float)n;
    float std = sqrtf(var);

    /* seuil = moyenne du bruit de repos + k écarts-types de ce
     * même bruit de repos. k=2.0 est un bon point de départ :
     * ~2-3% du bruit pur dépasse cette barre, donc peu de faux
     * positifs au repos, tout en restant sensible aux vrais pics. */
    return mean + (k * std);
}

float IMU_Calc_Peaks(float accel[][3], uint16_t n, float threshold)
{
    if (n < 3) return 0.0f; /* pas assez de points pour un pic local */

    static float mag[IMU_FEATURES_MAX_N];
    n = IMU_Compute_Magnitudes(accel, n, mag);

    /* Seuil FIXE, fourni par l'appelant (voir IMU_Calibrate_PeakThreshold).
     * Contrairement à un seuil recalculé sur chaque fenêtre, celui-ci
     * ne "suit" pas le signal : un vrai mouvement intense reste bien
     * détecté comme plusieurs pics au lieu de voir la barre monter
     * avec lui. */
    uint16_t peak_count = 0;
    for (uint16_t i = 1; i < (n - 1); i++)
    {
        if (mag[i] > threshold &&
            mag[i] > mag[i - 1] &&
            mag[i] > mag[i + 1])
        {
            peak_count++;
        }
    }

    return (float)peak_count;
}

float IMU_Calc_RMSGyro(float gyro[][3], uint16_t n)
{
    /* Même calcul que IMU_Calc_RMS, appliqué au gyroscope. */
    return IMU_Calc_RMS(gyro, n);
}

void IMU_Calc_MeanXYZ(float accel[][3], uint16_t n,
                       float *mean_x, float *mean_y, float *mean_z)
{
    if (n == 0 || mean_x == NULL || mean_y == NULL || mean_z == NULL)
    {
        return;
    }

    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    for (uint16_t i = 0; i < n; i++)
    {
        sx += accel[i][0];
        sy += accel[i][1];
        sz += accel[i][2];
    }
    *mean_x = sx / (float)n;
    *mean_y = sy / (float)n;
    *mean_z = sz / (float)n;
}
