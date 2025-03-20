################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/ex_05b_main.c 

OBJS += \
./Core/Inc/ex_05b_main.o 

C_DEPS += \
./Core/Inc/ex_05b_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/terry/STM32CubeIDE/workspace_1.16.0/NEAR_ANCHOR/decadriver/inc" -I"C:/Users/terry/STM32CubeIDE/workspace_1.16.0/NEAR_ANCHOR/platform/inc" -I"C:/Users/terry/STM32CubeIDE/workspace_1.16.0/NEAR_ANCHOR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/ex_05b_main.cyclo ./Core/Inc/ex_05b_main.d ./Core/Inc/ex_05b_main.o ./Core/Inc/ex_05b_main.su

.PHONY: clean-Core-2f-Inc

