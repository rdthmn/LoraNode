################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/main.c \
../Src/sensors.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/sx1272.c \
../Src/syscalls.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/main.o \
./Src/sensors.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/sx1272.o \
./Src/syscalls.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/main.d \
./Src/sensors.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/sx1272.d \
./Src/syscalls.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DSTM32 -DARM_MATH_CM4 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/Inc" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/Drivers/CMSIS/Include" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/CMSIS/core" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/CMSIS/device" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/radle/Dropbox/Thesis/LoRaNode2/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


