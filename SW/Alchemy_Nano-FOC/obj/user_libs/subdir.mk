################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user_libs/ch32v203_core.c \
../user_libs/ch32v203_exti.c \
../user_libs/ch32v203_gpio.c \
../user_libs/ch32v203_rcc.c \
../user_libs/ch32v203_spi.c \
../user_libs/ch32v203_uart.c \
../user_libs/ch32v203_usbd.c \
../user_libs/ch32v203_usbd_cdc.c \
../user_libs/fifo.c 

OBJS += \
./user_libs/ch32v203_core.o \
./user_libs/ch32v203_exti.o \
./user_libs/ch32v203_gpio.o \
./user_libs/ch32v203_rcc.o \
./user_libs/ch32v203_spi.o \
./user_libs/ch32v203_uart.o \
./user_libs/ch32v203_usbd.o \
./user_libs/ch32v203_usbd_cdc.o \
./user_libs/fifo.o 

C_DEPS += \
./user_libs/ch32v203_core.d \
./user_libs/ch32v203_exti.d \
./user_libs/ch32v203_gpio.d \
./user_libs/ch32v203_rcc.d \
./user_libs/ch32v203_spi.d \
./user_libs/ch32v203_uart.d \
./user_libs/ch32v203_usbd.d \
./user_libs/ch32v203_usbd_cdc.d \
./user_libs/fifo.d 


# Each subdirectory must supply rules for building sources it contributes
user_libs/%.o: ../user_libs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -I"/media/dragomir/lnx_hdd/lnx_home/repos/Alchemy/SW/Alchemy_Nano-FOC/user_libs" -I"/media/dragomir/lnx_hdd/lnx_home/repos/Alchemy/SW/Alchemy_Nano-FOC/Debug" -I"/media/dragomir/lnx_hdd/lnx_home/repos/Alchemy/SW/Alchemy_Nano-FOC/User" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


