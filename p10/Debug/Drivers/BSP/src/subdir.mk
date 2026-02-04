################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/src/es_wifi.c \
../Drivers/BSP/src/es_wifi_io.c \
../Drivers/BSP/src/hts221.c \
../Drivers/BSP/src/stm32l475e_iot01.c \
../Drivers/BSP/src/stm32l475e_iot01_hsensor.c \
../Drivers/BSP/src/stm32l475e_iot01_tsensor.c \
../Drivers/BSP/src/wifi.c 

OBJS += \
./Drivers/BSP/src/es_wifi.o \
./Drivers/BSP/src/es_wifi_io.o \
./Drivers/BSP/src/hts221.o \
./Drivers/BSP/src/stm32l475e_iot01.o \
./Drivers/BSP/src/stm32l475e_iot01_hsensor.o \
./Drivers/BSP/src/stm32l475e_iot01_tsensor.o \
./Drivers/BSP/src/wifi.o 

C_DEPS += \
./Drivers/BSP/src/es_wifi.d \
./Drivers/BSP/src/es_wifi_io.d \
./Drivers/BSP/src/hts221.d \
./Drivers/BSP/src/stm32l475e_iot01.d \
./Drivers/BSP/src/stm32l475e_iot01_hsensor.d \
./Drivers/BSP/src/stm32l475e_iot01_tsensor.d \
./Drivers/BSP/src/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/src/%.o Drivers/BSP/src/%.su Drivers/BSP/src/%.cyclo: ../Drivers/BSP/src/%.c Drivers/BSP/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"/media/dani/media/workspace-1.19/SCYSH_proyecto/p10/Drivers/BSP/inc" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-src

clean-Drivers-2f-BSP-2f-src:
	-$(RM) ./Drivers/BSP/src/es_wifi.cyclo ./Drivers/BSP/src/es_wifi.d ./Drivers/BSP/src/es_wifi.o ./Drivers/BSP/src/es_wifi.su ./Drivers/BSP/src/es_wifi_io.cyclo ./Drivers/BSP/src/es_wifi_io.d ./Drivers/BSP/src/es_wifi_io.o ./Drivers/BSP/src/es_wifi_io.su ./Drivers/BSP/src/hts221.cyclo ./Drivers/BSP/src/hts221.d ./Drivers/BSP/src/hts221.o ./Drivers/BSP/src/hts221.su ./Drivers/BSP/src/stm32l475e_iot01.cyclo ./Drivers/BSP/src/stm32l475e_iot01.d ./Drivers/BSP/src/stm32l475e_iot01.o ./Drivers/BSP/src/stm32l475e_iot01.su ./Drivers/BSP/src/stm32l475e_iot01_hsensor.cyclo ./Drivers/BSP/src/stm32l475e_iot01_hsensor.d ./Drivers/BSP/src/stm32l475e_iot01_hsensor.o ./Drivers/BSP/src/stm32l475e_iot01_hsensor.su ./Drivers/BSP/src/stm32l475e_iot01_tsensor.cyclo ./Drivers/BSP/src/stm32l475e_iot01_tsensor.d ./Drivers/BSP/src/stm32l475e_iot01_tsensor.o ./Drivers/BSP/src/stm32l475e_iot01_tsensor.su ./Drivers/BSP/src/wifi.cyclo ./Drivers/BSP/src/wifi.d ./Drivers/BSP/src/wifi.o ./Drivers/BSP/src/wifi.su

.PHONY: clean-Drivers-2f-BSP-2f-src

