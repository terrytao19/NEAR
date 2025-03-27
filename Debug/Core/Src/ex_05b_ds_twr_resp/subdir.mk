################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ex_05b_ds_twr_resp/ex_05b_main.c 

OBJS += \
./Core/Src/ex_05b_ds_twr_resp/ex_05b_main.o 

C_DEPS += \
./Core/Src/ex_05b_ds_twr_resp/ex_05b_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ex_05b_ds_twr_resp/%.o Core/Src/ex_05b_ds_twr_resp/%.su Core/Src/ex_05b_ds_twr_resp/%.cyclo: ../Core/Src/ex_05b_ds_twr_resp/%.c Core/Src/ex_05b_ds_twr_resp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu90 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/decadriver/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/platform/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ex_05b_ds_twr_resp

clean-Core-2f-Src-2f-ex_05b_ds_twr_resp:
	-$(RM) ./Core/Src/ex_05b_ds_twr_resp/ex_05b_main.cyclo ./Core/Src/ex_05b_ds_twr_resp/ex_05b_main.d ./Core/Src/ex_05b_ds_twr_resp/ex_05b_main.o ./Core/Src/ex_05b_ds_twr_resp/ex_05b_main.su

.PHONY: clean-Core-2f-Src-2f-ex_05b_ds_twr_resp

