
/* emg_filter.h
 * Filtre passe-bande Butterworth ordre 2 + Notch 50Hz
 * Fc_basse  =  20 Hz  → elimine DC + artefacts mouvement
 * Fc_haute  = 200 Hz  → elimine bruit haute frequence
 * Fc_notch  =  50 Hz  → elimine bruit reseau electrique
 * Fs        = 500 Hz  (ADS1292R)
 *
 * Architecture : 3 biquads en cascade
 *   Entree → [HP 20Hz] → [LP 200Hz] → [Notch 50Hz] → Sortie
 *
 * Reutilise BiquadState_t de ppg_filter.h
 * Meme formule biquad, coefficients differents */

#ifndef EMG_FILTER_H
#define EMG_FILTER_H

#include "ppg_filter.h"  /* reutilise BiquadState_t */

/* -------------------------------------------------------
 * Coefficients passe-haut Butterworth ordre 2
 * Fc = 20 Hz, Fs = 500 Hz
 * Calcules avec scipy.signal.butter(2, 20.0/250.0, 'high')
 * ------------------------------------------------------- */
#define EMG_HP_B0  ( 0.83708919f)
#define EMG_HP_B1  (-1.67417838f)
#define EMG_HP_B2  ( 0.83708919f)
#define EMG_HP_A1  (-1.64745998f)
#define EMG_HP_A2  ( 0.70089678f)

/* -------------------------------------------------------
 * Coefficients passe-bas Butterworth ordre 2
 * Fc = 200 Hz, Fs = 500 Hz
 * Calcules avec scipy.signal.butter(2, 200.0/250.0, 'low')
 * ------------------------------------------------------- */
#define EMG_LP_B0  ( 0.63894553f)
#define EMG_LP_B1  ( 1.27789105f)
#define EMG_LP_B2  ( 0.63894553f)
#define EMG_LP_A1  ( 1.14298050f)
#define EMG_LP_A2  ( 0.41280160f)

/* -------------------------------------------------------
 * Coefficients Notch 50 Hz
 * Fc = 50 Hz, Fs = 500 Hz, Q = 30
 * Calcules avec scipy.signal.iirnotch(50.0, 30.0, 500)
 *
 * Q=30 → filtre tres etroit autour de 50Hz
 *   → supprime uniquement 50Hz
 *   → preserve signal EMG 20-200Hz
 * ------------------------------------------------------- */
#define EMG_NOTCH_B0  ( 0.98963618f)
#define EMG_NOTCH_B1  (-1.60126497f)
#define EMG_NOTCH_B2  ( 0.98963618f)
#define EMG_NOTCH_A1  (-1.60126497f)
#define EMG_NOTCH_A2  ( 0.97927235f)

/* -------------------------------------------------------
 * EMG_Filter_t — filtre EMG complet
 *
 * Trois sections en cascade :
 *   hp    → passe-haut  20 Hz
 *   lp    → passe-bas  200 Hz
 *   notch → notch       50 Hz
 *
 * Meme structure BiquadState_t que PPG
 * Une instance par canal (ch1 et ch2 independants)
 * ------------------------------------------------------- */
typedef struct {
    BiquadState_t hp;    /* passe-haut  20Hz */
    BiquadState_t lp;    /* passe-bas  200Hz */
    BiquadState_t notch; /* notch       50Hz */
} EMG_Filter_t;

/* -------------------------------------------------------
 * Fonctions publiques
 * ------------------------------------------------------- */

/* Initialise tous les etats a zero
 * Appeler UNE FOIS avant la boucle de la tache */
void  EMG_Filter_Init  (EMG_Filter_t *f);

/* Applique le filtre sur UN sample
 * x      = sample brut (ch1 ou ch2 converti en float)
 * retour = sample filtre                               */
float EMG_Filter_Apply (EMG_Filter_t *f, float x);

#endif /* EMG_FILTER_H */
