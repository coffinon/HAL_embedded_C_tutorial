################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Projekt_TM1_Kacper_Kupiszewski.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Projekt_TM1_Kacper_Kupiszewski.list \

OBJCOPY_BIN += \
Projekt_TM1_Kacper_Kupiszewski.bin \


# All Target
all: Projekt_TM1_Kacper_Kupiszewski.elf secondary-outputs

# Tool invocations
Projekt_TM1_Kacper_Kupiszewski.elf: $(OBJS) $(USER_OBJS) C:\Users\Kacper\STM32CubeIDE\workspace_1.3.0\Projekt_TM1_Kacper_Kupiszewski\STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "Projekt_TM1_Kacper_Kupiszewski.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Kacper\STM32CubeIDE\workspace_1.3.0\Projekt_TM1_Kacper_Kupiszewski\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Projekt_TM1_Kacper_Kupiszewski.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Projekt_TM1_Kacper_Kupiszewski.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Projekt_TM1_Kacper_Kupiszewski.list"
	@echo 'Finished building: $@'
	@echo ' '

Projekt_TM1_Kacper_Kupiszewski.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Projekt_TM1_Kacper_Kupiszewski.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
