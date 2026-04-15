/* nlms_filter.h
 * Filtre adaptatif NLMS (Normalized Least Mean Squares)
 * Pour la correction des artefacts de mouvement PPG
 * par le signal IMU
 *
 * Principe :
 *   signal_propre = signal_filtré - (w × imu_normalisé)
 *   w mis à jour à chaque sample
 * ------------------------------------------------------- */

#ifndef NLMS_FILTER_H
#define NLMS_FILTER_H

#include <stdint.h>

/* -------------------------------------------------------
 * Structure NLMS — un filtre par canal PPG
 *
 * w  : poids adaptatif
 *      → apprend la relation IMU → artefact
 *      → initialisé à 0
 *      → converge en ~500ms à 1s
 *
 * mu : vitesse d'apprentissage
 *      → 0.01 = bon compromis stabilité/vitesse
 * ------------------------------------------------------- */
typedef struct {
    float w;   /* poids adaptatif        */
    float mu;  /* vitesse d'apprentissage */
} NLMS_Filter_t;

/* -------------------------------------------------------
 * Fonctions publiques
 * ------------------------------------------------------- */

/* Initialiser le filtre
 * mu_val : vitesse d'apprentissage (ex: 0.01f) */
void  NLMS_Init  (NLMS_Filter_t *f, float mu_val);

/* Appliquer le filtre sur UN sample
 *
 * ppg_filtered : sample PPG déjà filtré (passe-bande)
 * imu_norm     : magnitude IMU normalisée
 *                = (magnitude - 1000) / 1000
 *                  repos=0.0, mouvement=0.2 à 1.0
 *
 * retour : sample PPG corrigé (artefact soustrait) */
float NLMS_Apply (NLMS_Filter_t *f,
                  float ppg_filtered,
                  float imu_norm);

#endif /* NLMS_FILTER_H */
