################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
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
Stepper_Motor.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Stepper_Motor.list \

OBJCOPY_BIN += \
Stepper_Motor.bin \


# All Target
all: Stepper_Motor.elf secondary-outputs

# Tool invocations
Stepper_Motor.elf: $(OBJS) $(USER_OBJS) C:\Users\mukun\Desktop\stepper\STM32F030R8TX_FLASH.ld
	arm-none-eabi-gcc -o "Stepper_Motor.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\mukun\Desktop\stepper\STM32F030R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Stepper_Motor.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Stepper_Motor.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Stepper_Motor.list"
	@echo 'Finished building: $@'
	@echo ' '

Stepper_Motor.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Stepper_Motor.bin"
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
