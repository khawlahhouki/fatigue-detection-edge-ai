/* safe_printf.h
 *
 * Wrapper thread-safe autour de printf(), pour éviter que plusieurs
 * tâches FreeRTOS n'entrelacent leurs caractères sur l'UART quand
 * elles impriment en même temps (ex: [TEMP]PROCESS] Batch reçu,[EMG] tick).
 *
 * Utilisation : remplacer tous les printf(...) du projet par
 * SafePrintf(...), même syntaxe, mêmes arguments.
 */

#ifndef SAFE_PRINTF_H
#define SAFE_PRINTF_H

/* À appeler UNE SEULE FOIS, dans main.c, entre USER CODE BEGIN 2 et
 * USER CODE END 2, AVANT osKernelStart(). Crée le mutex partagé. */
void SafePrintf_Init(void);

/* Remplace printf() partout dans le projet. Même signature, même
 * comportement, mais protégé par mutex : une seule tâche à la fois
 * peut écrire sur l'UART, les autres attendent leur tour. */
int SafePrintf(const char *fmt, ...);

#endif /* SAFE_PRINTF_H */
