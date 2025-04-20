################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TFT_Library/LCD/images.c \
../TFT_Library/LCD/lcd.c 

OBJS += \
./TFT_Library/LCD/images.o \
./TFT_Library/LCD/lcd.o 

C_DEPS += \
./TFT_Library/LCD/images.d \
./TFT_Library/LCD/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
TFT_Library/LCD/%.o TFT_Library/LCD/%.su TFT_Library/LCD/%.cyclo: ../TFT_Library/LCD/%.c TFT_Library/LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/ST7789" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/TFT_Library/LCD" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/TFT_Library/UGUI" -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/decadriver/inc" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/platform/inc" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TFT_Library-2f-LCD

clean-TFT_Library-2f-LCD:
	-$(RM) ./TFT_Library/LCD/images.cyclo ./TFT_Library/LCD/images.d ./TFT_Library/LCD/images.o ./TFT_Library/LCD/images.su ./TFT_Library/LCD/lcd.cyclo ./TFT_Library/LCD/lcd.d ./TFT_Library/LCD/lcd.o ./TFT_Library/LCD/lcd.su

.PHONY: clean-TFT_Library-2f-LCD

