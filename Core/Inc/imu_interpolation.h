/* imu_interpolation.h
 * Interpolation linéaire du signal IMU
 * pour synchronisation avec le signal PPG
 *
 * But :
 *   IMU à 104 Hz → 1 sample toutes les 9.6ms
 *   PPG à 400 Hz → 1 sample toutes les 2.5ms
 *   → créer 1 valeur IMU par sample PPG
 *     par interpolation linéaire entre
 *     les deux slots IMU encadrants
 * ------------------------------------------------------- */

#ifndef IMU_INTERPOLATION_H
#define IMU_INTERPOLATION_H

#include <stdint.h>
#include <math.h>
#include "lsm6dso_imu.h"  /* IMU_Raw_t, IMU_Batch_t */

/* -------------------------------------------------------
 * IMU_Interp_t — contexte d'interpolation
 *
 * imu_pts[]    : tableau des points IMU disponibles
 *                imu_pts[0]    = dernier slot batch précédent
 *                imu_pts[1..17] = slots batch courant
 *
 * imu_pts_count : nombre de points valides
 *
 * imu_ptr      : index courant dans imu_pts[]
 *                avance progressivement au fil
 *                des samples PPG traités
 *
 * imu_prev_end : dernier slot du batch précédent
 *                → point de départ de l'interpolation
 *                  pour le prochain lot PPG
 * ------------------------------------------------------- */
typedef struct {
    IMU_Raw_t imu_pts[18];
    uint8_t   imu_pts_count;
    int8_t    imu_ptr;
    IMU_Raw_t imu_prev_end;
    uint8_t   first_run;
} IMU_Interp_t;

/* -------------------------------------------------------
 * Fonctions publiques
 * ------------------------------------------------------- */

/* Initialiser le contexte d'interpolation
 * Appeler UNE FOIS avant la boucle         */
void  IMU_Interp_Init   (IMU_Interp_t *ctx);

/* Préparer un nouveau batch IMU
 * Appeler UNE FOIS par lot PPG reçu
 *
 * batch    : nouveau batch IMU reçu
 * received : 1 = batch valide, 0 = batch absent */
void  IMU_Interp_SetBatch(IMU_Interp_t *ctx,
                           IMU_Batch_t  *batch,
                           uint8_t       received);

/* Obtenir la magnitude IMU interpolée
 * à l'instant t_ppg
 *
 * Appeler pour chaque sample PPG
 * dans l'ordre chronologique
 *
 * retour : magnitude en mg
 *          repos ≈ 1000mg
 *          mouvement > 1200mg               */
float IMU_Interp_GetMag  (IMU_Interp_t *ctx,
                           uint32_t      t_ppg);

/* Normaliser la magnitude
 * repos   = 1000mg → 0.0
 * mouvement = 1500mg → 0.5
 * intense = 2000mg → 1.0  */
float IMU_Interp_Normalize(float mag);

#endif /* IMU_INTERPOLATION_H */
