/* nlms_filter.c */
#include "nlms_filter.h"

/* -------------------------------------------------------
 * NLMS_Init
 *
 * Initialise le filtre à zéro.
 * Appeler UNE FOIS avant la boucle de la tâche.
 * ------------------------------------------------------- */
void NLMS_Init(NLMS_Filter_t *f, float mu_val)
{
    f->w  = 0.0f;
    f->mu = mu_val;
}

/* -------------------------------------------------------
 * NLMS_Apply
 *
 * Applique le filtre NLMS sur UN sample.
 *
 * Étape 1 — Estimer et soustraire l'artefact :
 *   artefact_estimé = w × imu_norm
 *   ppg_propre      = ppg_filtré - artefact_estimé
 *
 * Étape 2 — Mettre à jour le poids :
 *   norm = imu_norm² + 1
 *     → +1 évite division par zéro si repos
 *     → normalise le pas d'adaptation
 *   w += mu × ppg_propre × imu_norm / norm
 *
 * Pourquoi diviser par norm ?
 *   LMS  simple : w += mu × erreur × imu
 *   NLMS normalisé : w += mu × erreur × imu / norm
 *   → si imu grand (mouvement fort) :
 *     norm grand → ajustement petit → stable ✓
 *   → si imu petit (repos) :
 *     norm ≈ 1 → ajustement normal ✓
 * ------------------------------------------------------- */
float NLMS_Apply(NLMS_Filter_t *f,
                 float ppg_filtered,
                 float imu_norm)
{
    /* Soustraire l'artefact estimé */
    float ppg_clean = ppg_filtered - (f->w * imu_norm);

    /* Normalisation */
    float norm = (imu_norm * imu_norm) + 1.0f;

    /* Mettre à jour le poids */
    f->w += f->mu * (ppg_clean * imu_norm) / norm;

    return ppg_clean;
}
