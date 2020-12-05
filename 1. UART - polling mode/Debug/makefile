################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
1.\ UART\ -\ polling\ mode.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
1.\ UART\ -\ polling\ mode.list \

OBJCOPY_BIN += \
1.\ UART\ -\ polling\ mode.bin \


# All Target
all: 1.\ UART\ -\ polling\ mode.elf secondary-outputs

# Tool invocations
1.\ UART\ -\ polling\ mode.elf: $(OBJS) $(USER_OBJS) C:\Users\skorp\Desktop\projects\HAL_embedded_C_tutorial\1.\ UART\ -\ polling\ mode\STM32F103RBTX_FLASH.ld
	arm-none-eabi-gcc -o "1. UART - polling mode.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\skorp\Desktop\projects\HAL_embedded_C_tutorial\1. UART - polling mode\STM32F103RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="1. UART - polling mode.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

1.\ UART\ -\ polling\ mode.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "1. UART - polling mode.list"
	@echo 'Finished building: $@'
	@echo ' '

1.\ UART\ -\ polling\ mode.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "1. UART - polling mode.bin"
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
