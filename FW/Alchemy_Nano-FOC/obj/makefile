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
Alchemy_Nano-FOC.hex \

SECONDARY_LIST += \
Alchemy_Nano-FOC.lst \

SECONDARY_SIZE += \
Alchemy_Nano-FOC.siz \


# All Target
all: Alchemy_Nano-FOC.elf secondary-outputs

# Tool invocations
Alchemy_Nano-FOC.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -T "/media/dragomir/lnx_hdd/lnx_home/repos/Alchemy/SW/Alchemy_Nano-FOC/Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"Alchemy_Nano-FOC.map" --specs=nano.specs --specs=nosys.specs -o "Alchemy_Nano-FOC.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Alchemy_Nano-FOC.hex: Alchemy_Nano-FOC.elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	riscv-none-embed-objcopy -O ihex "Alchemy_Nano-FOC.elf"  "Alchemy_Nano-FOC.hex"
	@echo 'Finished building: $@'
	@echo ' '

Alchemy_Nano-FOC.lst: Alchemy_Nano-FOC.elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	riscv-none-embed-objdump --all-headers --demangle --disassemble "Alchemy_Nano-FOC.elf" > "Alchemy_Nano-FOC.lst"
	@echo 'Finished building: $@'
	@echo ' '

Alchemy_Nano-FOC.siz: Alchemy_Nano-FOC.elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	riscv-none-embed-size --format=berkeley "Alchemy_Nano-FOC.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Alchemy_Nano-FOC.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
