################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include user_libs/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Debug/subdir.mk
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
Alchemy_TMC5160_Dev.hex \

SECONDARY_LIST += \
Alchemy_TMC5160_Dev.lst \

SECONDARY_SIZE += \
Alchemy_TMC5160_Dev.siz \


# All Target
all: Alchemy_TMC5160_Dev.elf secondary-outputs

# Tool invocations
Alchemy_TMC5160_Dev.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T "C:\Users\Steve\mrs_community_workspace\Alchemy_TMC5160_Dev\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"Alchemy_TMC5160_Dev.map" --specs=nano.specs --specs=nosys.specs -o "Alchemy_TMC5160_Dev.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Alchemy_TMC5160_Dev.hex: Alchemy_TMC5160_Dev.elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	riscv-none-embed-objcopy -O ihex "Alchemy_TMC5160_Dev.elf"  "Alchemy_TMC5160_Dev.hex"
	@echo 'Finished building: $@'
	@echo ' '

Alchemy_TMC5160_Dev.lst: Alchemy_TMC5160_Dev.elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	riscv-none-embed-objdump --all-headers --demangle --disassemble "Alchemy_TMC5160_Dev.elf" > "Alchemy_TMC5160_Dev.lst"
	@echo 'Finished building: $@'
	@echo ' '

Alchemy_TMC5160_Dev.siz: Alchemy_TMC5160_Dev.elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	riscv-none-embed-size --format=berkeley "Alchemy_TMC5160_Dev.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Alchemy_TMC5160_Dev.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
