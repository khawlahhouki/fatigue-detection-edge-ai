/* emg_filter.c */
#include "emg_filter.h"

/* -------------------------------------------------------
 * EMG_Filter_Init
 * Initialise tous les etats a zero
 * Meme logique que PPG_Filter_Init
 * ------------------------------------------------------- */
void EMG_Filter_Init(EMG_Filter_t *f)
{
    /* Passe-haut 20Hz */
    f->hp.x1 = 0.0f;  f->hp.x2 = 0.0f;
    f->hp.y1 = 0.0f;  f->hp.y2 = 0.0f;

    /* Passe-bas 200Hz */
    f->lp.x1 = 0.0f;  f->lp.x2 = 0.0f;
    f->lp.y1 = 0.0f;  f->lp.y2 = 0.0f;

    /* Notch 50Hz */
    f->notch.x1 = 0.0f;  f->notch.x2 = 0.0f;
    f->notch.y1 = 0.0f;  f->notch.y2 = 0.0f;
}

/* -------------------------------------------------------
 * EMG_Filter_Apply
 * Applique le filtre sur UN sample EMG
 *
 * Pipeline :
 *   x → [HP 20Hz] → [LP 200Hz] → [Notch 50Hz] → y
 *
 * Formule biquad (identique PPG) :
 *   y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2]
 *                  - A1*y[n-1] - A2*y[n-2]
 * ------------------------------------------------------- */
float EMG_Filter_Apply(EMG_Filter_t *f, float x)
{
    /* ---------------------------------------------------
     * Section 1 — Passe-haut 20 Hz
     *
     * Elimine :
     *   → composante DC (offset electrode)
     *   → artefacts de mouvement (0-20 Hz)
     *   → derive lente de la ligne de base
     * --------------------------------------------------- */
    float hp_out = EMG_HP_B0 * x
                 + EMG_HP_B1 * f->hp.x1
                 + EMG_HP_B2 * f->hp.x2
                 - EMG_HP_A1 * f->hp.y1
                 - EMG_HP_A2 * f->hp.y2;

    f->hp.x2 = f->hp.x1;  f->hp.x1 = x;
    f->hp.y2 = f->hp.y1;  f->hp.y1 = hp_out;

    /* ---------------------------------------------------
     * Section 2 — Passe-bas 200 Hz
     *
     * Elimine :
     *   → bruit haute frequence (> 200 Hz)
     *   → bruit electronique ADS1292R
     *
     * Entree = sortie du passe-haut (hp_out)
     * PAS le signal brut x
     * --------------------------------------------------- */
    float lp_out = EMG_LP_B0 * hp_out
                 + EMG_LP_B1 * f->lp.x1
                 + EMG_LP_B2 * f->lp.x2
                 - EMG_LP_A1 * f->lp.y1
                 - EMG_LP_A2 * f->lp.y2;

    f->lp.x2 = f->lp.x1;  f->lp.x1 = hp_out;
    f->lp.y2 = f->lp.y1;  f->lp.y1 = lp_out;

    /* ---------------------------------------------------
     * Section 3 — Notch 50 Hz
     *
     * Elimine :
     *   → bruit reseau electrique exactement a 50Hz
     *   → Q=30 : filtre tres etroit
     *   → preserve tout le signal EMG 20-200Hz
     *
     * Entree = sortie du passe-bas (lp_out)
     * --------------------------------------------------- */
    float notch_out = EMG_NOTCH_B0 * lp_out
                    + EMG_NOTCH_B1 * f->notch.x1
                    + EMG_NOTCH_B2 * f->notch.x2
                    - EMG_NOTCH_A1 * f->notch.y1
                    - EMG_NOTCH_A2 * f->notch.y2;

    f->notch.x2 = f->notch.x1;  f->notch.x1 = lp_out;
    f->notch.y2 = f->notch.y1;  f->notch.y1 = notch_out;

    return notch_out;
}
