################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/filters_cortexM3.s \
../Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/ltp_cortexM3.s \
../Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/vq_cortexm3.s 

OBJS += \
./Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/filters_cortexM3.o \
./Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/ltp_cortexM3.o \
./Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/vq_cortexm3.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/%.o: ../Drivers/STM32F10x_Speex_Lib/STM32/libspeex/gcc/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


