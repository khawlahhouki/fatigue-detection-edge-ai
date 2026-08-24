/* user_profile.h
 *
 * Données démographiques/personnalité saisies UNE SEULE FOIS par
 * utilisateur, via une commande terminal série au démarrage (avant
 * osKernelStart()). Contexte statique pour le modèle IA — aide la
 * généralisation inter-participants.
 */

#ifndef USER_PROFILE_H
#define USER_PROFILE_H

#include <stdint.h>
#include "main.h"   /* UART_HandleTypeDef */

typedef struct {
    float age;
    float gender;
    float reserved;
    float generally_trusting;
    float lazy;
    float relaxed_handless_stress;
    float few_artistic_interests;
    float sociable;
    float criticize_others;
    float thorough_job;
    float easily_nervous;
    float active_magination;
    float phisical_fitness;
    float cardiorespiratory_fitness;
    float muscular_strength;
    float agility_speed;
    float flexibility;
    float cardiovascular_health_conditions;
} UserProfile_t;

extern UserProfile_t user_profile;
extern uint8_t        user_profile_ready;   /* 1 = saisie valide reçue */

/* Bloque en attente d'une ligne CSV (18 valeurs, ordre fixe — voir
 * .c) sur l'UART console, la parse, remplit user_profile.
 * À appeler UNE FOIS, dans main.c, USER CODE BEGIN 2, AVANT
 * osKernelStart(). Si rien n'est saisi avant timeout_ms,
 * user_profile_ready reste à 0 et les champs valent 0.0f. */
void UserProfile_PromptAndRead(UART_HandleTypeDef *huart, uint32_t timeout_ms);

#endif /* USER_PROFILE_H */
