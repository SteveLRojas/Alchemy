################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v20x_it.c \
../User/main.c \
../User/support.c 

OBJS += \
./User/ch32v20x_it.o \
./User/main.o \
./User/support.o 

C_DEPS += \
./User/ch32v20x_it.d \
./User/main.d \
./User/support.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -I"C:\Users\Steve\repos\Alchemy\SW\Alchemy_Nano-FOC\user_libs" -I"C:\Users\Steve\repos\Alchemy\SW\Alchemy_Nano-FOC\Debug" -I"C:\Users\Steve\repos\Alchemy\SW\Alchemy_Nano-FOC\User" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


