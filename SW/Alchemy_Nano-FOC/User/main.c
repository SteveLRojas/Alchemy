//                     /\         /\__
//                   // \       (  0 )_____/\            __
//                  // \ \     (vv          o|          /^v\
//                //    \ \   (vvvv  ___-----^        /^^/\vv\
//              //  /     \ \ |vvvvv/               /^^/    \v\
//             //  /       (\\/vvvv/              /^^/       \v\
//            //  /  /  \ (  /vvvv/              /^^/---(     \v\
//           //  /  /    \( /vvvv/----(O        /^^/           \v\
//          //  /  /  \  (/vvvv/               /^^/             \v|
//        //  /  /    \( vvvv/                /^^/               ||
//       //  /  /    (  vvvv/                 |^^|              //
//      //  / /    (  |vvvv|                  /^^/            //
//     //  / /   (    \vvvvv\          )-----/^^/           //
//    // / / (          \vvvvv\            /^^^/          //
//   /// /(               \vvvvv\        /^^^^/          //
//  ///(              )-----\vvvvv\    /^^^^/-----(      \\
// //(                        \vvvvv\/^^^^/               \\
///(                            \vvvv^^^/                 //
//                                \vv^/         /        //
//                                             /<______//
//                                            <<<------/
//                                             \<
//                                              \
//**************************************************
//* main.c                        SOURCE FILE      *
//* Copyright (C) 2024 Esteban Looser-Rojas.       *
//* Contains Alchemy firmware for the Nano-FOC     *
//* project. Currently using UART1 for the host,   *
//* and SPI1 for the Nano-FOC target.              *
//**************************************************

#include "debug.h"
#include "ch32v20x.h"
#include "uart.h"

/* Global typedef */

/* Global define */
#define PORT_UART1 GPIOA
#define PORT_UART2 GPIOA
#define PORT_SPI1 GPIOB
#define PORT_SPI1_NCS GPIOA

#define PIN_UART1_TX GPIO_Pin_9
#define PIN_UART1_RX GPIO_Pin_10
#define PIN_UART2_TX GPIO_Pin_2
#define PIN_UART2_RX GPIO_Pin_3
#define PIN_SPI1_MOSI GPIO_Pin_5
#define PIN_SPI1_MISO GPIO_Pin_4
#define PIN_SPI1_SCK GPIO_Pin_3
#define PIN_SPI1_NCS GPIO_Pin_15

/* Global Variable */

void platform_init()
{
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure = {0};
	SPI_InitTypeDef SPI_InitStructure={0};
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	//GPIOA
	GPIO_InitStructure.GPIO_Pin = (PIN_SPI1_NCS);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(PORT_SPI1_NCS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = (PIN_UART1_TX);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(PORT_UART1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = (PIN_UART1_RX);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(PORT_UART1, &GPIO_InitStructure);

	//GPIOB
	GPIO_InitStructure.GPIO_Pin = (PIN_SPI1_SCK | PIN_SPI1_MOSI);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(PORT_SPI1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_SPI1_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(PORT_SPI1, &GPIO_InitStructure);

	//GPIOC
	//GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_Init(GPIOC, &GPIO_InitStructure);

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

	//GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);

	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_SET);
}

void nano_write_reg(uint8_t addr, uint16_t data)
{
	uint8_t* data_bytes = (uint8_t*)((void*)(&data));

	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_RESET);
	spi_transfer(SPI1, (uint16_t)addr);
	spi_transfer(SPI1, (uint16_t)(data_bytes[1]));
	spi_transfer(SPI1, (uint16_t)(data_bytes[0]));
	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_SET);
	return;
}

uint16_t nano_read_reg(uint8_t addr)
{
	uint16_t read_val;
	uint8_t* data_bytes = (uint8_t*)((void*)(&read_val));

	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_RESET);
	spi_transfer(SPI1, (uint16_t)addr);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_SET);
	Delay_Us(1);

	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_RESET);
	spi_transfer(SPI1, (uint16_t)addr);
	data_bytes[1] = (uint8_t)spi_transfer(SPI1, 0x0000);
	data_bytes[0] = (uint8_t)spi_transfer(SPI1, 0x0000);
	GPIO_WriteBit(PORT_SPI1_NCS, PIN_SPI1_NCS, Bit_SET);

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
    uart_init(115200, 0, 0);

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("Alchemy - Nano-FOC\n");

    if(uart_bytes_available(USART1))
    {
    	printf("There is something fishy in the RX buffer...\n");
    }

    uint8_t addr;
	uint16_t reg_data;
	uint8_t* reg_bytes = (uint8_t*)((void*)(&reg_data));
	while(1)
	{
		addr = uart_read_byte(USART1);

		if(addr & 0x80)	//handle write
		{
			reg_bytes[1] = uart_read_byte(USART1);
			reg_bytes[0] = uart_read_byte(USART1);

			nano_write_reg(addr, reg_data);
		}
		else	//handle read
		{
			reg_data = nano_read_reg(addr);

			uart_write_byte(USART1, reg_bytes[1]);
			uart_write_byte(USART1, reg_bytes[0]);
		}
	}
}
