################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/freertos.c \
../Src/main.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_hal_timebase_TIM.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/freertos.o \
./Src/main.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_hal_timebase_TIM.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/freertos.d \
./Src/main.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_hal_timebase_TIM.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DUSE_KISS_FFT -DHAVE_CONFIG_H '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xE -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/WS2812B" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/LCD for STM32F103RET" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/LedMatrix" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/include/speex" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/libspeex" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/STM32" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/STM32/include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/STM32/include/speex" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/STM32/libspeex" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/uart" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/voice" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Inc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


