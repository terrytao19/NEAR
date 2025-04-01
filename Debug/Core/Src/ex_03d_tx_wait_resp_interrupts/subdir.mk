################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.c 

OBJS += \
./Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.o 

C_DEPS += \
./Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ex_03d_tx_wait_resp_interrupts/%.o Core/Src/ex_03d_tx_wait_resp_interrupts/%.su Core/Src/ex_03d_tx_wait_resp_interrupts/%.cyclo: ../Core/Src/ex_03d_tx_wait_resp_interrupts/%.c Core/Src/ex_03d_tx_wait_resp_interrupts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/decadriver/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/platform/inc" -I"C:/Users/twhel/Documents/Rob450/nearfield/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ex_03d_tx_wait_resp_interrupts

clean-Core-2f-Src-2f-ex_03d_tx_wait_resp_interrupts:
	-$(RM) ./Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.cyclo ./Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.d ./Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.o ./Core/Src/ex_03d_tx_wait_resp_interrupts/ex_03d_main.su

.PHONY: clean-Core-2f-Src-2f-ex_03d_tx_wait_resp_interrupts

