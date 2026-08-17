#ifndef LORA_INTERFACE_H
#define LORA_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Initialise le module LoRa. Retourne 0 si succès, code d'erreur RadioLib sinon.
int LoRa_Init(void);

// Envoie un paquet. Retourne 0 si succès, code d'erreur RadioLib sinon.
int LoRa_Send(const uint8_t* data, uint16_t len);

// À appeler depuis TaskLoRaTX après un envoi : bloque jusqu'à notification ou timeout
// Retourne 1 si notifié (TxDone reçu), 0 si timeout
int LoRa_WaitDone(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
