################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user_libs/ch32v203_core.c \
../user_libs/ch32v203_dma.c \
../user_libs/ch32v203_gpio.c \
../user_libs/ch32v203_rcc.c \
../user_libs/ch32v203_spi.c \
../user_libs/ch32v203_uart_dma.c \
../user_libs/ch32v203_usbd.c \
../user_libs/ch32v203_usbd_cdc.c 

OBJS += \
./user_libs/ch32v203_core.o \
./user_libs/ch32v203_dma.o \
./user_libs/ch32v203_gpio.o \
./user_libs/ch32v203_rcc.o \
./user_libs/ch32v203_spi.o \
./user_libs/ch32v203_uart_dma.o \
./user_libs/ch32v203_usbd.o \
./user_libs/ch32v203_usbd_cdc.o 

C_DEPS += \
./user_libs/ch32v203_core.d \
./user_libs/ch32v203_dma.d \
./user_libs/ch32v203_gpio.d \
./user_libs/ch32v203_rcc.d \
./user_libs/ch32v203_spi.d \
./user_libs/ch32v203_uart_dma.d \
./user_libs/ch32v203_usbd.d \
./user_libs/ch32v203_usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
user_libs/%.o: ../user_libs/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"C:\Users\elooser\mrs_community_workspace\Alchemy_TM10_DMA\user_libs" -I"C:\Users\elooser\mrs_community_workspace\Alchemy_TM10_DMA\Debug" -I"C:\Users\elooser\mrs_community_workspace\Alchemy_TM10_DMA\User" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

