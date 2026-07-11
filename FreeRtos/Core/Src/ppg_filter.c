/* ppg_filter.c */
#include "ppg_filter.h"

/* -------------------------------------------------------
 * Coefficients passe-haut Butterworth ordre 2
 * Fc = 0.5 Hz, Fs = 400 Hz
 *
 * Calculés avec scipy.signal.butter(2, 0.5/200, 'high')
 * ------------------------------------------------------- */
#define HP_B0  ( 0.99377006f)
#define HP_B1  (-1.98754012f)
#define HP_B2  ( 0.99377006f)
#define HP_A1  (-1.98740580f)
#define HP_A2  ( 0.98767444f)

/* -------------------------------------------------------
 * Coefficients passe-bas Butterworth ordre 2
 * Fc = 4.0 Hz, Fs = 400 Hz
 *
 * Calculés avec scipy.signal.butter(2, 4.0/200, 'low')
 * ------------------------------------------------------- */
#define LP_B0  ( 0.00060479f)
#define LP_B1  ( 0.00120958f)
#define LP_B2  ( 0.00060479f)
#define LP_A1  (-1.98820860f)
#define LP_A2  ( 0.98963675f)

/* -------------------------------------------------------
 * PPG_Filter_Init
 * Initialise tous les états à zéro
 * ------------------------------------------------------- */
void PPG_Filter_Init(BandpassFilter_t *f)
{
    f->hp.x1 = 0.0f;
    f->hp.x2 = 0.0f;
    f->hp.y1 = 0.0f;
    f->hp.y2 = 0.0f;

    f->lp.x1 = 0.0f;
    f->lp.x2 = 0.0f;
    f->lp.y1 = 0.0f;
    f->lp.y2 = 0.0f;
}

/* -------------------------------------------------------
 * PPG_Filter_Apply
 * Applique le filtre passe-bande sur UN sample
 *
 * Séquence :
 *   1. Passe-haut 0.5 Hz → élimine dérive baseline
 *   2. Passe-bas  4.0 Hz → élimine bruit > 4 Hz
 *
 * Formule biquad :
 *   y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2]
 *         - A1*y[n-1] - A2*y[n-2]
 * ------------------------------------------------------- */
float PPG_Filter_Apply(BandpassFilter_t *f, float x)
{
    /* ---------------------------------------------------
     * Section 1 — Passe-haut 0.5 Hz
     *
     * Élimine :
     *   → dérive lente de la ligne de base
     *   → composante respiratoire (0.2-0.3 Hz)
     *   → artefacts DC
     * --------------------------------------------------- */
    float hp_out = HP_B0 * x
                 + HP_B1 * f->hp.x1
                 + HP_B2 * f->hp.x2
                 - HP_A1 * f->hp.y1
                 - HP_A2 * f->hp.y2;

    /* Mettre à jour la mémoire passe-haut */
    f->hp.x2 = f->hp.x1;
    f->hp.x1 = x;
    f->hp.y2 = f->hp.y1;
    f->hp.y1 = hp_out;

    /* ---------------------------------------------------
     * Section 2 — Passe-bas 4.0 Hz
     *
     * Élimine :
     *   → bruit haute fréquence
     *   → harmoniques > 4 Hz
     *   → artefacts électroniques
     *
     * Entrée = sortie du passe-haut (hp_out)
     * PAS le signal brut x
     * --------------------------------------------------- */
    float lp_out = LP_B0 * hp_out
                 + LP_B1 * f->lp.x1
                 + LP_B2 * f->lp.x2
                 - LP_A1 * f->lp.y1
                 - LP_A2 * f->lp.y2;

    /* Mettre à jour la mémoire passe-bas */
    f->lp.x2 = f->lp.x1;
    f->lp.x1 = hp_out;
    f->lp.y2 = f->lp.y1;
    f->lp.y1 = lp_out;

    return lp_out;
}
