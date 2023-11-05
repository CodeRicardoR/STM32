################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/Src/LCD_Library.c 

OBJS += \
./Peripheral/Src/LCD_Library.o 

C_DEPS += \
./Peripheral/Src/LCD_Library.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Src/%.o Peripheral/Src/%.su: ../Peripheral/Src/%.c Peripheral/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ricar/Documents/STM32/Clone/STM32/NUCLEO64/EXAMPLE_INTERRUPTS/Peripheral/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripheral-2f-Src

clean-Peripheral-2f-Src:
	-$(RM) ./Peripheral/Src/LCD_Library.d ./Peripheral/Src/LCD_Library.o ./Peripheral/Src/LCD_Library.su

.PHONY: clean-Peripheral-2f-Src

