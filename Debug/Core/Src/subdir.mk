################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ads1292r.c \
../Core/Src/dma.c \
../Core/Src/emg_features.c \
../Core/Src/emg_filter.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/imu_interpolation.c \
../Core/Src/lsm6dso_imu.c \
../Core/Src/main.c \
../Core/Src/max86141.c \
../Core/Src/memorymap.c \
../Core/Src/nlms_filter.c \
../Core/Src/ppg_features.c \
../Core/Src/ppg_filter.c \
../Core/Src/shtc3.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/temp_features.c \
../Core/Src/tim.c \
../Core/Src/tmp117.c 

OBJS += \
./Core/Src/ads1292r.o \
./Core/Src/dma.o \
./Core/Src/emg_features.o \
./Core/Src/emg_filter.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/imu_interpolation.o \
./Core/Src/lsm6dso_imu.o \
./Core/Src/main.o \
./Core/Src/max86141.o \
./Core/Src/memorymap.o \
./Core/Src/nlms_filter.o \
./Core/Src/ppg_features.o \
./Core/Src/ppg_filter.o \
./Core/Src/shtc3.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/temp_features.o \
./Core/Src/tim.o \
./Core/Src/tmp117.o 

C_DEPS += \
./Core/Src/ads1292r.d \
./Core/Src/dma.d \
./Core/Src/emg_features.d \
./Core/Src/emg_filter.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/imu_interpolation.d \
./Core/Src/lsm6dso_imu.d \
./Core/Src/main.d \
./Core/Src/max86141.d \
./Core/Src/memorymap.d \
./Core/Src/nlms_filter.d \
./Core/Src/ppg_features.d \
./Core/Src/ppg_filter.d \
./Core/Src/shtc3.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/temp_features.d \
./Core/Src/tim.d \
./Core/Src/tmp117.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DRENODE_SIMULATION -DUSE_PWR_DIRECT_SMPS_SUPPLY -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/BSP/Components/lsm6dso -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ads1292r.cyclo ./Core/Src/ads1292r.d ./Core/Src/ads1292r.o ./Core/Src/ads1292r.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/emg_features.cyclo ./Core/Src/emg_features.d ./Core/Src/emg_features.o ./Core/Src/emg_features.su ./Core/Src/emg_filter.cyclo ./Core/Src/emg_filter.d ./Core/Src/emg_filter.o ./Core/Src/emg_filter.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/imu_interpolation.cyclo ./Core/Src/imu_interpolation.d ./Core/Src/imu_interpolation.o ./Core/Src/imu_interpolation.su ./Core/Src/lsm6dso_imu.cyclo ./Core/Src/lsm6dso_imu.d ./Core/Src/lsm6dso_imu.o ./Core/Src/lsm6dso_imu.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max86141.cyclo ./Core/Src/max86141.d ./Core/Src/max86141.o ./Core/Src/max86141.su ./Core/Src/memorymap.cyclo ./Core/Src/memorymap.d ./Core/Src/memorymap.o ./Core/Src/memorymap.su ./Core/Src/nlms_filter.cyclo ./Core/Src/nlms_filter.d ./Core/Src/nlms_filter.o ./Core/Src/nlms_filter.su ./Core/Src/ppg_features.cyclo ./Core/Src/ppg_features.d ./Core/Src/ppg_features.o ./Core/Src/ppg_features.su ./Core/Src/ppg_filter.cyclo ./Core/Src/ppg_filter.d ./Core/Src/ppg_filter.o ./Core/Src/ppg_filter.su ./Core/Src/shtc3.cyclo ./Core/Src/shtc3.d ./Core/Src/shtc3.o ./Core/Src/shtc3.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/temp_features.cyclo ./Core/Src/temp_features.d ./Core/Src/temp_features.o ./Core/Src/temp_features.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/tmp117.cyclo ./Core/Src/tmp117.d ./Core/Src/tmp117.o ./Core/Src/tmp117.su

.PHONY: clean-Core-2f-Src

