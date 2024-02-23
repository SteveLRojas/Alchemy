################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
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
SECONDARY_FLASH += \
Alchemy_TMC5130.hex \

SECONDARY_LIST += \
Alchemy_TMC5130.lst \

SECONDARY_SIZE += \
Alchemy_TMC5130.siz \


# All Target
all: Alchemy_TMC5130.elf secondary-outputs

# Tool invocations
Alchemy_TMC5130.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -T "C:\Users\Steve\Documents\mrs_community_workspace\Alchemy_TMC5130\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"Alchemy_TMC5130.map" --specs=nano.specs --specs=nosys.specs -o "Alchemy_TMC5130.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Alchemy_TMC5130.hex: Alchemy_TMC5130.elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	riscv-none-embed-objcopy -O ihex "Alchemy_TMC5130.elf"  "Alchemy_TMC5130.hex"
	@echo 'Finished building: $@'
	@echo ' '

Alchemy_TMC5130.lst: Alchemy_TMC5130.elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	riscv-none-embed-objdump --all-headers --demangle --disassemble "Alchemy_TMC5130.elf" > "Alchemy_TMC5130.lst"
	@echo 'Finished building: $@'
	@echo ' '

Alchemy_TMC5130.siz: Alchemy_TMC5130.elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	riscv-none-embed-size --format=berkeley "Alchemy_TMC5130.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Alchemy_TMC5130.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
