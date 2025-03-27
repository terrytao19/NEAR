################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../platform/src/deca_mutex.c \
../platform/src/deca_range_tables.c \
../platform/src/deca_sleep.c \
../platform/src/deca_spi.c \
../platform/src/port.c 

OBJS += \
./platform/src/deca_mutex.o \
./platform/src/deca_range_tables.o \
./platform/src/deca_sleep.o \
./platform/src/deca_spi.o \
./platform/src/port.o 

C_DEPS += \
./platform/src/deca_mutex.d \
./platform/src/deca_range_tables.d \
./platform/src/deca_sleep.d \
./platform/src/deca_spi.d \
./platform/src/port.d 


# Each subdirectory must supply rules for building sources it contributes
platform/src/%.o platform/src/%.su platform/src/%.cyclo: ../platform/src/%.c platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/decadriver/inc" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/platform/inc" -I"C:/Users/terry/MICHIGAN/ROB_450/NEAR/compiler/inc" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-platform-2f-src

clean-platform-2f-src:
	-$(RM) ./platform/src/deca_mutex.cyclo ./platform/src/deca_mutex.d ./platform/src/deca_mutex.o ./platform/src/deca_mutex.su ./platform/src/deca_range_tables.cyclo ./platform/src/deca_range_tables.d ./platform/src/deca_range_tables.o ./platform/src/deca_range_tables.su ./platform/src/deca_sleep.cyclo ./platform/src/deca_sleep.d ./platform/src/deca_sleep.o ./platform/src/deca_sleep.su ./platform/src/deca_spi.cyclo ./platform/src/deca_spi.d ./platform/src/deca_spi.o ./platform/src/deca_spi.su ./platform/src/port.cyclo ./platform/src/port.d ./platform/src/port.o ./platform/src/port.su

.PHONY: clean-platform-2f-src

