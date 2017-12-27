################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/2_LCD\ for\ STM32F103RET/tm_stm32_hd44780.c 

OBJS += \
./Drivers/2_LCD\ for\ STM32F103RET/tm_stm32_hd44780.o 

C_DEPS += \
./Drivers/2_LCD\ for\ STM32F103RET/tm_stm32_hd44780.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/2_LCD\ for\ STM32F103RET/tm_stm32_hd44780.o: ../Drivers/2_LCD\ for\ STM32F103RET/tm_stm32_hd44780.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xE -I"/home/user/workspaceSTM32/CupRTC_v4.0/Inc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/user/workspaceSTM32/CupRTC_v4.0/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/2_LCD for STM32F103RET/tm_stm32_hd44780.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


