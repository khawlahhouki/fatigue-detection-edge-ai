#include "STM32Hal.h"

STM32Hal::STM32Hal(SPI_HandleTypeDef* spi)
  : RadioLibHal(0, 1, 0, 1, 0, 1),
    _spi(spi) {}

void STM32Hal::init() {
  // Rien à faire : CubeMX a déjà initialisé SPI3 et les GPIO
  // via MX_SPI3_Init() et MX_GPIO_Init() dans main.c
}

void STM32Hal::term() {
  // Rien à nettoyer
}

// --- GPIO ---
void STM32Hal::pinMode(uint32_t pin, uint32_t mode) {
  // Non utilisé : les GPIO sont déjà configurés dans CubeMX
}

void STM32Hal::digitalWrite(uint32_t pin, uint32_t value) {
  HAL_GPIO_WritePin((GPIO_TypeDef*)(pin >> 16), (uint16_t)(pin & 0xFFFF),
                     (GPIO_PinState)value);
}

uint32_t STM32Hal::digitalRead(uint32_t pin) {
  return HAL_GPIO_ReadPin((GPIO_TypeDef*)(pin >> 16), (uint16_t)(pin & 0xFFFF));
}

// --- Interruptions ---
void STM32Hal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
  // Géré manuellement via HAL_GPIO_EXTI_Callback dans stm32h7xx_it.c
}

void STM32Hal::detachInterrupt(uint32_t interruptNum) {
  // Idem, rien à faire ici
}

// --- Délais ---
void STM32Hal::delay(unsigned long ms) {
  HAL_Delay(ms);
}

void STM32Hal::delayMicroseconds(unsigned long us) {
  volatile uint32_t count = us * (SystemCoreClock / 1000000) / 5;
  while (count--);
}

unsigned long STM32Hal::millis() {
  return HAL_GetTick();
}

unsigned long STM32Hal::micros() {
  return HAL_GetTick() * 1000;
}

// --- SPI ---
void STM32Hal::spiBegin() {}
void STM32Hal::spiBeginTransaction() {}

void STM32Hal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
  HAL_SPI_TransmitReceive(_spi, out, in, len, HAL_MAX_DELAY);
}

void STM32Hal::spiEndTransaction() {}
void STM32Hal::spiEnd() {}
long STM32Hal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
  // Non utilisé par le SX1276 dans un usage standard
  return 0;
}
