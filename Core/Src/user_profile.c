/* user_profile.c — voir user_profile.h */

#include "user_profile.h"
#include "safe_printf.h"
#include <stdlib.h>
#include <string.h>

UserProfile_t user_profile = {0};
uint8_t        user_profile_ready = 0;

#define PROFILE_LINE_MAX 200
#define PROFILE_N_FIELDS 18

void UserProfile_PromptAndRead(UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    static char line[PROFILE_LINE_MAX];
    uint16_t idx = 0;
    uint8_t  ch;
    uint32_t start = HAL_GetTick();

    SafePrintf("\r\n=== Profil utilisateur (saisie unique) ===\r\n");
    SafePrintf("Entrez 18 valeurs separees par des virgules, dans cet ordre :\r\n");
    SafePrintf("age,gender,reserved,generally_trusting,lazy,relaxed_handless_stress,\r\n");
    SafePrintf("few_artistic_interests,sociable,criticize_others,thorough_job,\r\n");
    SafePrintf("easily_nervous,active_magination,phisical_fitness,\r\n");
    SafePrintf("cardiorespiratory_fitness,muscular_strength,agility_speed,\r\n");
    SafePrintf("flexibility,cardiovascular_health_conditions\r\n");
    SafePrintf("Exemple : 25,1,3,5,2,6,3,7,4,2,5,6,4,5,3,4,5,1\r\n");
    SafePrintf("Timeout %lu ms -> valeurs par defaut (0) si rien saisi.\r\n", timeout_ms);
    SafePrintf("> ");

    /* Lecture caractère par caractère jusqu'à '\n'/'\r' ou timeout
     * global. Écho local pour que l'utilisateur voie ce qu'il tape
     * (le terminal série ne le fait pas forcément tout seul). */
    while ((HAL_GetTick() - start) < timeout_ms && idx < (PROFILE_LINE_MAX - 1))
    {
        if (HAL_UART_Receive(huart, &ch, 1, 50) == HAL_OK)
        {
            if (ch == '\r' || ch == '\n')
            {
                if (idx == 0) continue; /* ignore CR/LF isolé en début de ligne */
                break;
            }
            line[idx++] = (char)ch;
            HAL_UART_Transmit(huart, &ch, 1, 10); /* écho */
        }
    }
    line[idx] = '\0';

    if (idx == 0)
    {
        SafePrintf("\r\n[PROFILE] Aucune saisie recue, valeurs par defaut (0) utilisees.\r\n");
        user_profile_ready = 0;
        return;
    }

    /* Tableau de pointeurs, dans le même ordre que le CSV attendu,
     * pour remplir la structure via une seule boucle strtok(). */
    float *fields[PROFILE_N_FIELDS] = {
        &user_profile.age,
        &user_profile.gender,
        &user_profile.reserved,
        &user_profile.generally_trusting,
        &user_profile.lazy,
        &user_profile.relaxed_handless_stress,
        &user_profile.few_artistic_interests,
        &user_profile.sociable,
        &user_profile.criticize_others,
        &user_profile.thorough_job,
        &user_profile.easily_nervous,
        &user_profile.active_magination,
        &user_profile.phisical_fitness,
        &user_profile.cardiorespiratory_fitness,
        &user_profile.muscular_strength,
        &user_profile.agility_speed,
        &user_profile.flexibility,
        &user_profile.cardiovascular_health_conditions
    };

    char   *token = strtok(line, ",");
    uint8_t count = 0;
    while (token != NULL && count < PROFILE_N_FIELDS)
    {
        *(fields[count]) = strtof(token, NULL);
        count++;
        token = strtok(NULL, ",");
    }

    if (count == PROFILE_N_FIELDS)
    {
        user_profile_ready = 1;
        SafePrintf("\r\n[PROFILE] OK, %d valeurs recues et enregistrees.\r\n", count);
    }
    else
    {
        user_profile_ready = 0;
        SafePrintf("\r\n[PROFILE] ECHEC : %d valeurs recues, %d attendues. Valeurs par defaut (0).\r\n",
                   count, PROFILE_N_FIELDS);
    }
}
