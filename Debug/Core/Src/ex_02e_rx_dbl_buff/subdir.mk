################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.c 

OBJS += \
./Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.o 

C_DEPS += \
./Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ex_02e_rx_dbl_buff/%.o Core/Src/ex_02e_rx_dbl_buff/%.su Core/Src/ex_02e_rx_dbl_buff/%.cyclo: ../Core/Src/ex_02e_rx_dbl_buff/%.c Core/Src/ex_02e_rx_dbl_buff/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/decadriver/inc" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/platform/inc" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ex_02e_rx_dbl_buff

clean-Core-2f-Src-2f-ex_02e_rx_dbl_buff:
	-$(RM) ./Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.cyclo ./Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.d ./Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.o ./Core/Src/ex_02e_rx_dbl_buff/ex_02e_main.su

.PHONY: clean-Core-2f-Src-2f-ex_02e_rx_dbl_buff

