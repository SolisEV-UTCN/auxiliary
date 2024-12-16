################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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


BUILD_ARTIFACT_NAME := SolisEV4_2_Auxiliare_CBT
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SolisEV4_2_Auxiliare_CBT.elf \

MAP_FILES += \
SolisEV4_2_Auxiliare_CBT.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SolisEV4_2_Auxiliare_CBT.list \

OBJCOPY_HEX += \
SolisEV4_2_Auxiliare_CBT.hex \

OBJCOPY_BIN += \
SolisEV4_2_Auxiliare_CBT.bin \


# All Target
all: main-build

# Main-build Target
main-build: SolisEV4_2_Auxiliare_CBT.elf secondary-outputs

# Tool invocations
SolisEV4_2_Auxiliare_CBT.elf SolisEV4_2_Auxiliare_CBT.map: $(OBJS) $(USER_OBJS) D:\SOLIS\SolisEV4_Auxiliare\SolisEV4_2_Auxiliare_CBT6\STM32F303CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "SolisEV4_2_Auxiliare_CBT.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\SOLIS\SolisEV4_Auxiliare\SolisEV4_2_Auxiliare_CBT6\STM32F303CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SolisEV4_2_Auxiliare_CBT.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SolisEV4_2_Auxiliare_CBT.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SolisEV4_2_Auxiliare_CBT.list"
	@echo 'Finished building: $@'
	@echo ' '

SolisEV4_2_Auxiliare_CBT.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "SolisEV4_2_Auxiliare_CBT.hex"
	@echo 'Finished building: $@'
	@echo ' '

SolisEV4_2_Auxiliare_CBT.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SolisEV4_2_Auxiliare_CBT.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) SolisEV4_2_Auxiliare_CBT.bin SolisEV4_2_Auxiliare_CBT.elf SolisEV4_2_Auxiliare_CBT.hex SolisEV4_2_Auxiliare_CBT.list SolisEV4_2_Auxiliare_CBT.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
