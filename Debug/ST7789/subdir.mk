################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ST7789/fonts.c \
../ST7789/st7789.c 

OBJS += \
./ST7789/fonts.o \
./ST7789/st7789.o 

C_DEPS += \
./ST7789/fonts.d \
./ST7789/st7789.d 


# Each subdirectory must supply rules for building sources it contributes
ST7789/%.o ST7789/%.su ST7789/%.cyclo: ../ST7789/%.c ST7789/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/ST7789" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/TFT_Library/LCD" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/TFT_Library/UGUI" -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/decadriver/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/platform/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ST7789

clean-ST7789:
	-$(RM) ./ST7789/fonts.cyclo ./ST7789/fonts.d ./ST7789/fonts.o ./ST7789/fonts.su ./ST7789/st7789.cyclo ./ST7789/st7789.d ./ST7789/st7789.o ./ST7789/st7789.su

.PHONY: clean-ST7789

