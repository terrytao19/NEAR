################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include platform/src/subdir.mk
-include decadriver/src/subdir.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/ex_06b_ss_twr_resp/subdir.mk
-include Core/Src/ex_06a_ss_twr_init/subdir.mk
-include Core/Src/ex_05b_ds_twr_resp/subdir.mk
-include Core/Src/ex_05a_ds_twr_init/subdir.mk
-include Core/Src/ex_03d_tx_wait_resp_interrupts/subdir.mk
-include Core/Src/ex_03b_rx_send_resp/subdir.mk
-include Core/Src/ex_03a_tx_wait_resp/subdir.mk
-include Core/Src/ex_02e_rx_dbl_buff/subdir.mk
-include Core/Src/ex_02a_simple_rx/subdir.mk
-include Core/Src/ex_01a_simple_tx/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := NEAR_ANCHOR
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
NEAR_ANCHOR.elf \

MAP_FILES += \
NEAR_ANCHOR.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NEAR_ANCHOR.list \


# All Target
all: main-build

# Main-build Target
main-build: NEAR_ANCHOR.elf secondary-outputs

# Tool invocations
NEAR_ANCHOR.elf NEAR_ANCHOR.map: $(OBJS) $(USER_OBJS) C:\Users\twhel\Documents\Rob450\nearfield\NEAR\STM32F072CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "NEAR_ANCHOR.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\twhel\Documents\Rob450\nearfield\NEAR\STM32F072CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="NEAR_ANCHOR.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NEAR_ANCHOR.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NEAR_ANCHOR.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) NEAR_ANCHOR.elf NEAR_ANCHOR.list NEAR_ANCHOR.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
