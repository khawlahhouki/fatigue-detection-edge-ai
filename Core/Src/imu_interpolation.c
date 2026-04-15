/* imu_interpolation.c */
#include "imu_interpolation.h"

/* -------------------------------------------------------
 * IMU_Interp_Init
 * ------------------------------------------------------- */
void IMU_Interp_Init(IMU_Interp_t *ctx)
{
    /* Initialiser imu_prev_end à repos
     * ax=0, ay=0, az=1000mg (gravité)
     * timestamp=0 (TIM2 démarre à 0)      */
    ctx->imu_prev_end.accel_mg[0] = 0.0f;
    ctx->imu_prev_end.accel_mg[1] = 0.0f;
    ctx->imu_prev_end.accel_mg[2] = 1000.0f;
    ctx->imu_prev_end.timestamp   = 0;

    ctx->imu_pts_count = 0;
    ctx->imu_ptr       = 0;
    ctx->first_run     = 1;
}

/* -------------------------------------------------------
 * IMU_Interp_SetBatch
 *
 * Prépare le tableau imu_pts[] pour le lot PPG courant.
 *
 * Structure de imu_pts[] :
 *   [0]      = imu_prev_end (dernier slot batch précédent)
 *   [1..17]  = slots du batch courant
 *
 * imu_ptr remis à 0 → recommence depuis le début
 * pour chaque nouveau lot PPG
 * ------------------------------------------------------- */
void IMU_Interp_SetBatch(IMU_Interp_t *ctx,
                          IMU_Batch_t  *batch,
                          uint8_t       received)
{
    /* Remettre imu_ptr à 0
     * → recommencer depuis le début du tableau
     *   pour chaque nouveau lot PPG              */
    ctx->imu_ptr = 0;

    if (received == 1 && batch != NULL)
    {
        /* Premier passage → initialiser imu_prev_end
         * avec le vrai premier slot IMU             */
        if (ctx->first_run == 1)
        {
            ctx->imu_prev_end = batch->slots[0];
            ctx->first_run    = 0;
        }

        /* imu_pts[0] = dernier slot batch précédent */
        ctx->imu_pts[0]    = ctx->imu_prev_end;
        ctx->imu_pts_count = 1;

        /* imu_pts[1..N] = slots batch courant */
        for (uint8_t s = 0;
             s < batch->count &&
             ctx->imu_pts_count < 18;
             s++)
        {
            ctx->imu_pts[ctx->imu_pts_count] =
                batch->slots[s];
            ctx->imu_pts_count++;
        }

        /* Sauvegarder dernier slot
         * → servira de imu_prev_end au prochain lot */
        ctx->imu_prev_end =
            batch->slots[batch->count - 1];
    }
    else
    {
        /* Batch absent → 1 seul point connu
         * → interpolation dégradée mais pas de crash */
        ctx->imu_pts[0]    = ctx->imu_prev_end;
        ctx->imu_pts_count = 1;
    }
}

/* -------------------------------------------------------
 * IMU_Interp_GetMag
 *
 * Retourne la magnitude IMU interpolée à t_ppg.
 *
 * Algorithme :
 *   1. Avancer imu_ptr jusqu'aux deux slots
 *      qui encadrent t_ppg
 *   2. Interpoler linéairement entre eux
 *
 * imu_ptr est progressif :
 *   → n'avance que quand t_ppg dépasse
 *     le slot IMU suivant
 *   → toutes les ~4 samples PPG
 * ------------------------------------------------------- */
float IMU_Interp_GetMag(IMU_Interp_t *ctx,
                         uint32_t      t_ppg)
{
    /* Avancer imu_ptr pour trouver
     * les deux slots qui encadrent t_ppg */
    while (ctx->imu_ptr < ctx->imu_pts_count - 2 &&
           ctx->imu_pts[ctx->imu_ptr + 1].timestamp
           < t_ppg)
    {
        ctx->imu_ptr++;
    }

    /* Calculer magnitude des deux points encadrants */
    float ax1 = ctx->imu_pts[ctx->imu_ptr].accel_mg[0];
    float ay1 = ctx->imu_pts[ctx->imu_ptr].accel_mg[1];
    float az1 = ctx->imu_pts[ctx->imu_ptr].accel_mg[2];
    float mag1 = sqrtf(ax1*ax1 + ay1*ay1 + az1*az1);

    float ax2 =
        ctx->imu_pts[ctx->imu_ptr + 1].accel_mg[0];
    float ay2 =
        ctx->imu_pts[ctx->imu_ptr + 1].accel_mg[1];
    float az2 =
        ctx->imu_pts[ctx->imu_ptr + 1].accel_mg[2];
    float mag2 = sqrtf(ax2*ax2 + ay2*ay2 + az2*az2);

    /* Interpolation linéaire */
    float mag_interp = mag1;

    float diff_t =
        (float)(ctx->imu_pts[ctx->imu_ptr + 1].timestamp
              - ctx->imu_pts[ctx->imu_ptr].timestamp);

    if (diff_t > 0.0f)
    {
        float alpha =
            (float)(t_ppg
                  - ctx->imu_pts[ctx->imu_ptr].timestamp)
            / diff_t;

        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        mag_interp = mag1 + alpha * (mag2 - mag1);
    }

    return mag_interp;
}

/* -------------------------------------------------------
 * IMU_Interp_Normalize
 *
 * Normalise la magnitude autour de 0 :
 *   repos   = 1000mg → 0.0
 *   marche  = 1200mg → 0.2
 *   intense = 2000mg → 1.0
 *
 * Nécessaire pour NLMS :
 *   PPG en unités ADC (0 à 524287)
 *   IMU en mg (1000 à 2000)
 *   → sans normalisation les poids w
 *     deviendraient énormes → instabilité ✗
 * ------------------------------------------------------- */
float IMU_Interp_Normalize(float mag)
{
    return (mag - 1000.0f) / 1000.0f;
}
