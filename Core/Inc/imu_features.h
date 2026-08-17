/* imu_features.h
 *
 * Calcul des features IMU (accéléromètre + gyroscope) sur une fenêtre
 * de N échantillons, utilisées comme entrées du modèle IA (StartTaskFeatures).
 *
 * Entrée attendue : float[N][3] où [i][0]=X, [i][1]=Y, [i][2]=Z
 *   - accel en mg
 *   - gyro  en mdps
 */

#ifndef IMU_FEATURES_H
#define IMU_FEATURES_H

#include <stdint.h>

/* RMS de la magnitude du vecteur accélération sur la fenêtre.
 * magnitude[i] = sqrt(x[i]^2 + y[i]^2 + z[i]^2)
 * RMS = sqrt( mean( magnitude[i]^2 ) )
 * Unité : mg. Indique l'intensité globale du mouvement. */
float IMU_Calc_RMS(float accel[][3], uint16_t n);

/* Variance de la magnitude du vecteur accélération sur la fenêtre.
 * Unité : mg². Indique la dispersion/variabilité du mouvement
 * (une variance élevée = mouvement irrégulier ; faible = stable). */
float IMU_Calc_Var(float accel[][3], uint16_t n);

/* Calibre un seuil FIXE de détection de pics, à partir d'une fenêtre
 * de référence — idéalement mesurée capteur immobile, au démarrage.
 * seuil = moyenne(magnitude) + k * écart-type(magnitude)
 * k=2.0 recommandé. À appeler UNE SEULE FOIS ; réutiliser la valeur
 * retournée dans tous les appels suivants à IMU_Calc_Peaks(). */
float IMU_Calibrate_PeakThreshold(float accel[][3], uint16_t n, float k);

/* Nombre de pics locaux détectés sur la magnitude du vecteur
 * accélération. Un pic = échantillon strictement supérieur à ses
 * deux voisins immédiats ET dépassant le seuil FIXE fourni en
 * paramètre (voir IMU_Calibrate_PeakThreshold ci-dessus) — ce seuil
 * ne se recalcule pas à chaque fenêtre, pour rester fiable aussi
 * bien au repos (peu de faux positifs) qu'en mouvement intense
 * (le seuil ne "monte" pas avec le signal).
 * Retourne un float pour rester homogène avec les autres features
 * (le nombre de pics est toujours un entier positif ou nul). */
float IMU_Calc_Peaks(float accel[][3], uint16_t n, float threshold);

/* RMS de la magnitude du vecteur gyroscope sur la fenêtre.
 * Unité : mdps. Indique l'intensité globale de rotation. */
float IMU_Calc_RMSGyro(float gyro[][3], uint16_t n);

#endif /* IMU_FEATURES_H */
