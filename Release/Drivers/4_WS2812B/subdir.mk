################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/4_WS2812B/ws2812b.c 

OBJS += \
./Drivers/4_WS2812B/ws2812b.o 

C_DEPS += \
./Drivers/4_WS2812B/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/4_WS2812B/%.o: ../Drivers/4_WS2812B/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xE -I"/home/user/workspaceSTM32/CupRTC_v4.0/Inc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


