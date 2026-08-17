#ifndef STM32HAL_H
#define STM32HAL_H

#include "RadioLib.h"
#include "main.h"

class STM32Hal : public RadioLibHal {
  public:
    STM32Hal(SPI_HandleTypeDef* spi);

    void init() override;
    void term() override;

    // GPIO
    void pinMode(uint32_t pin, uint32_t mode) override;
    void digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;

    // Interruptions
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t interruptNum) override;

    // Délais
    void delay(unsigned long ms) override;
    void delayMicroseconds(unsigned long us) override;
    unsigned long millis() override;
    unsigned long micros() override;

    // SPI
    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override;
    void spiEnd() override;
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override;
  private:
    SPI_HandleTypeDef* _spi;
};

#endif
