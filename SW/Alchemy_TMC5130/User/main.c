/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
 串口打印调试例程：
 USART1_Tx(PA9)。
 本例程演示使用 USART1(PA9) 作打印调试口输出。

*/

#include "debug.h"

/* Global typedef */

/* Global define */
#define PORT_5130_CTRL GPIOA
#define PIN_5130_NCS GPIO_Pin_15
#define PIN_5130_DRV_ENN GPIO_Pin_8
#define PIN_5130_CLK GPIO_Pin_1	//Hack! requires hardware mod

/* Global Variable */
//TODO: drive DRV_ENN
void platform_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure = {0};
	SPI_InitTypeDef SPI_InitStructure={0};

	//SPI stuff
	GPIO_InitStructure.GPIO_Pin = (PIN_5130_NCS | PIN_5130_DRV_ENN | PIN_5130_CLK);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_5130_CTRL, &GPIO_InitStructure);

	//SCK, MOSI pin
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_3 | GPIO_Pin_5);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//MISO pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//TODO: adjust
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_SET);
	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_DRV_ENN, Bit_SET);
	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_CLK, Bit_RESET);	//Try to use internal oscillator, may require TMC5130 to be power cycled

	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

void tmc5130_write_reg(uint8_t addr, uint32_t data)
{
	uint8_t* data_bytes = (uint8_t*)((void*)(&data));

	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_RESET);
	spi_transfer(SPI1, (uint16_t)addr);
	spi_transfer(SPI1, (uint16_t)(data_bytes[3]));
	spi_transfer(SPI1, (uint16_t)(data_bytes[2]));
	spi_transfer(SPI1, (uint16_t)(data_bytes[1]));
	spi_transfer(SPI1, (uint16_t)(data_bytes[0]));
	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_SET);
	return;
}

uint32_t tmc5130_read_reg(uint8_t addr)
{
	uint32_t read_val;
	uint8_t* data_bytes = (uint8_t*)((void*)(&read_val));

	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_RESET);
	spi_transfer(SPI1, (uint16_t)addr);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_SET);
	Delay_Us(1);

	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_RESET);
	spi_transfer(SPI1, (uint16_t)addr);
	data_bytes[3] = (uint8_t)spi_transfer(SPI1, 0x0000);
	data_bytes[2] = (uint8_t)spi_transfer(SPI1, 0x0000);
	data_bytes[1] = (uint8_t)spi_transfer(SPI1, 0x0000);
	data_bytes[0] = (uint8_t)spi_transfer(SPI1, 0x0000);
	GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_NCS, Bit_SET);

	return read_val;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    platform_init();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("Alchemy - TMC5130\n");

    uint8_t addr;
    uint32_t reg_data;
    uint8_t* reg_bytes = (uint8_t*)((void*)(&reg_data));
    while(1)
    {
    	addr = uart_get_byte();

    	if(addr == 0xFF)	//write control bits
		{
    		reg_bytes[3] = uart_get_byte();
			reg_bytes[2] = uart_get_byte();
			reg_bytes[1] = uart_get_byte();
			reg_bytes[0] = uart_get_byte();

			GPIO_WriteBit(PORT_5130_CTRL, PIN_5130_DRV_ENN, reg_bytes[0]);
		}
    	else if(addr == 0x7F)	//read control bits
    	{
    		reg_data = 0;
    		reg_bytes[0] = GPIO_ReadOutputDataBit(PORT_5130_CTRL, PIN_5130_DRV_ENN);

    		uart_send_byte(reg_bytes[3]);
			uart_send_byte(reg_bytes[2]);
			uart_send_byte(reg_bytes[1]);
			uart_send_byte(reg_bytes[0]);
    	}
    	else if(addr & 0x80)	//handle write
    	{
    		reg_bytes[3] = uart_get_byte();
    		reg_bytes[2] = uart_get_byte();
    		reg_bytes[1] = uart_get_byte();
    		reg_bytes[0] = uart_get_byte();

    		tmc5130_write_reg(addr, reg_data);
    	}
    	else	//handle read
    	{
    		reg_data = tmc5130_read_reg(addr);

    		uart_send_byte(reg_bytes[3]);
    		uart_send_byte(reg_bytes[2]);
    		uart_send_byte(reg_bytes[1]);
    		uart_send_byte(reg_bytes[0]);
    	}
    }
}
