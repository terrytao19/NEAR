################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../decadriver/src/deca_device.c \
../decadriver/src/deca_params_init.c 

OBJS += \
./decadriver/src/deca_device.o \
./decadriver/src/deca_params_init.o 

C_DEPS += \
./decadriver/src/deca_device.d \
./decadriver/src/deca_params_init.d 


# Each subdirectory must supply rules for building sources it contributes
decadriver/src/%.o decadriver/src/%.su decadriver/src/%.cyclo: ../decadriver/src/%.c decadriver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/terry/STM32CubeIDE/workspace_1.16.0/NEAR_ANCHOR/decadriver/inc" -I"C:/Users/terry/STM32CubeIDE/workspace_1.16.0/NEAR_ANCHOR/platform/inc" -I"C:/Users/terry/STM32CubeIDE/workspace_1.16.0/NEAR_ANCHOR/compiler/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-decadriver-2f-src

clean-decadriver-2f-src:
	-$(RM) ./decadriver/src/deca_device.cyclo ./decadriver/src/deca_device.d ./decadriver/src/deca_device.o ./decadriver/src/deca_device.su ./decadriver/src/deca_params_init.cyclo ./decadriver/src/deca_params_init.d ./decadriver/src/deca_params_init.o ./decadriver/src/deca_params_init.su

.PHONY: clean-decadriver-2f-src

