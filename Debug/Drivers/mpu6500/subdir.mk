################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/mpu6500/mpu6500.c 

OBJS += \
./Drivers/mpu6500/mpu6500.o 

C_DEPS += \
./Drivers/mpu6500/mpu6500.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/mpu6500/%.o Drivers/mpu6500/%.su Drivers/mpu6500/%.cyclo: ../Drivers/mpu6500/%.c Drivers/mpu6500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/VRSwork/VRS_zadanie/Drivers/mpu6500" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-mpu6500

clean-Drivers-2f-mpu6500:
	-$(RM) ./Drivers/mpu6500/mpu6500.cyclo ./Drivers/mpu6500/mpu6500.d ./Drivers/mpu6500/mpu6500.o ./Drivers/mpu6500/mpu6500.su

.PHONY: clean-Drivers-2f-mpu6500

