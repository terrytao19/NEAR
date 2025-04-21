################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TFT_Library/UGUI/ugui.c \
../TFT_Library/UGUI/ugui_button.c \
../TFT_Library/UGUI/ugui_checkbox.c \
../TFT_Library/UGUI/ugui_image.c \
../TFT_Library/UGUI/ugui_progress.c \
../TFT_Library/UGUI/ugui_textbox.c 

OBJS += \
./TFT_Library/UGUI/ugui.o \
./TFT_Library/UGUI/ugui_button.o \
./TFT_Library/UGUI/ugui_checkbox.o \
./TFT_Library/UGUI/ugui_image.o \
./TFT_Library/UGUI/ugui_progress.o \
./TFT_Library/UGUI/ugui_textbox.o 

C_DEPS += \
./TFT_Library/UGUI/ugui.d \
./TFT_Library/UGUI/ugui_button.d \
./TFT_Library/UGUI/ugui_checkbox.d \
./TFT_Library/UGUI/ugui_image.d \
./TFT_Library/UGUI/ugui_progress.d \
./TFT_Library/UGUI/ugui_textbox.d 


# Each subdirectory must supply rules for building sources it contributes
TFT_Library/UGUI/%.o TFT_Library/UGUI/%.su TFT_Library/UGUI/%.cyclo: ../TFT_Library/UGUI/%.c TFT_Library/UGUI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/ST7789" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/TFT_Library/LCD" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/TFT_Library/UGUI" -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/decadriver/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/platform/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TFT_Library-2f-UGUI

clean-TFT_Library-2f-UGUI:
	-$(RM) ./TFT_Library/UGUI/ugui.cyclo ./TFT_Library/UGUI/ugui.d ./TFT_Library/UGUI/ugui.o ./TFT_Library/UGUI/ugui.su ./TFT_Library/UGUI/ugui_button.cyclo ./TFT_Library/UGUI/ugui_button.d ./TFT_Library/UGUI/ugui_button.o ./TFT_Library/UGUI/ugui_button.su ./TFT_Library/UGUI/ugui_checkbox.cyclo ./TFT_Library/UGUI/ugui_checkbox.d ./TFT_Library/UGUI/ugui_checkbox.o ./TFT_Library/UGUI/ugui_checkbox.su ./TFT_Library/UGUI/ugui_image.cyclo ./TFT_Library/UGUI/ugui_image.d ./TFT_Library/UGUI/ugui_image.o ./TFT_Library/UGUI/ugui_image.su ./TFT_Library/UGUI/ugui_progress.cyclo ./TFT_Library/UGUI/ugui_progress.d ./TFT_Library/UGUI/ugui_progress.o ./TFT_Library/UGUI/ugui_progress.su ./TFT_Library/UGUI/ugui_textbox.cyclo ./TFT_Library/UGUI/ugui_textbox.d ./TFT_Library/UGUI/ugui_textbox.o ./TFT_Library/UGUI/ugui_textbox.su

.PHONY: clean-TFT_Library-2f-UGUI

