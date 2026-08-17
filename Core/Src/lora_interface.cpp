#include "lora_interface.h"

#include "STM32Hal.h"
#include "RadioLib.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

// Macro pour coder une broche STM32 en un seul uint32_t (port + numéro)
#define STM32_PIN(port, pin) (((uint32_t)(port) << 16) | (pin))

// Broches réelles de votre module LoRa-C1 (SX1276)
#define LORA_NSS    STM32_PIN(GPIOC, GPIO_PIN_9)   // NSS
#define LORA_DIO0   STM32_PIN(GPIOB, GPIO_PIN_1)   // DIO0
#define LORA_RESET  STM32_PIN(GPIOC, GPIO_PIN_8)   // NRESET

// Instances globales (créées une seule fois)
static STM32Hal hal(&hspi3);
static Module mod(&hal, LORA_NSS, LORA_DIO0, LORA_RESET);
static SX1276 radio(&mod);

int LoRa_Init(void) {
  int state = radio.begin(868.0, 125.0, 7, 5, RADIOLIB_SX127X_SYNC_WORD, 17);
  return state;
}

int LoRa_Send(const uint8_t* data, uint16_t len) {
  int state = radio.transmit((uint8_t*)data, len);
  return state;
}

int LoRa_WaitDone(uint32_t timeout_ms) {
  // Attend une notification directe sur la tâche appelante (TaskLoRaTX)
  uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
  return (notified > 0) ? 1 : 0;
}
