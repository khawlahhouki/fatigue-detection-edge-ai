/* ppg_filter.h
 * Filtre passe-bande Butterworth ordre 2
 * Fc_basse = 0.5 Hz  → élimine dérive baseline
 * Fc_haute = 4.0 Hz  → élimine bruit > 4 Hz
 * Fs       = 400 Hz  (MAX86141)
 *
 * Architecture : 2 biquads en cascade
 *   Entrée → [HP 0.5Hz] → [LP 4.0Hz] → Sortie  */

#ifndef PPG_FILTER_H
#define PPG_FILTER_H

#include <stdint.h>

/* -------------------------------------------------------
 * BiquadState_t — mémoire d'une section biquad
 *
 * Le filtre IIR a besoin des 2 entrées et 2 sorties
 * précédentes pour calculer la sortie actuelle.
 *
 * x1, x2 : entrées  retardées de 1 et 2 samples
 * y1, y2 : sorties retardées de 1 et 2 samples
 *
 * static dans la tâche → persistent entre les appels
 * ------------------------------------------------------- */
typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
} BiquadState_t;

/* -------------------------------------------------------
 * BandpassFilter_t — filtre passe-bande complet
 *
 * Deux sections en cascade :
 *   hp → passe-haut 0.5 Hz (élimine dérive)
 *   lp → passe-bas  4.0 Hz (élimine bruit)
 * ------------------------------------------------------- */
typedef struct {
    BiquadState_t hp;
    BiquadState_t lp;
} BandpassFilter_t;

/* -------------------------------------------------------
 * Fonctions publiques
 * ------------------------------------------------------- */

/* Initialise tous les états à zéro
 * Appeler UNE FOIS avant la boucle de la tâche */
void  PPG_Filter_Init  (BandpassFilter_t *f);

/* Applique le filtre sur UN sample
 * x     = sample brut (IR ou RED converti en float)
 * retour = sample filtré                          */
float PPG_Filter_Apply (BandpassFilter_t *f, float x);

#endif /* PPG_FILTER_H */
