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
//* project. Currently using CDC for the host,     *
//* and SPI1 for the Nano-FOC target.              *
//**************************************************

#include "ch32v20x.h"
#include "ch32v203_core.h"
#include "debug.h"
#include "fifo.h"
#include "ch32v203_uart.h"
#include "ch32v203_gpio.h"
#include "ch32v203_spi.h"
#include "ch32v203_rcc.h"
#include "ch32v203_afio.h"
#include "ch32v203_usbd_cdc.h"

//Pins:
// USART1_TX = PA9
// USART1_RX = PA10
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5


void nano_write_reg(uint8_t addr, uint16_t data)
{
	gpio_clear_pin(GPIOA, GPIO_PIN_15);

	spi_transfer(SPI1, addr);
	spi_transfer(SPI1, ((uint8_t*)&data)[1]);
	spi_transfer(SPI1, ((uint8_t*)&data)[0]);

	gpio_set_pin(GPIOA, GPIO_PIN_15);
	return;
}

uint16_t nano_read_reg(uint8_t addr)
{
	uint32_t read_val;

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	spi_transfer(SPI1, addr);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);
	Delay_Us(1);

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	(void)spi_transfer(SPI1, addr);
	((uint8_t*)&read_val)[1] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[0] = (uint8_t)spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);

	return read_val;
}

int main(void)
{
	rcc_apb2_clk_enable(RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN | RCC_TIM1EN | RCC_SPI1EN | RCC_USART1EN);
	rcc_apb1_clk_enable(RCC_TIM2EN | RCC_USBEN | RCC_USART2EN);

	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_15);	//SPI1_NCS
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_9);	//USART1_TX
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PULL_IN, GPIO_PIN_10);	//USART1_RX
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5);	//SPI1_SCK, SPI1_MOSI
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4);			//SPI1_MISO

	gpio_set_pin(GPIOA, GPIO_PIN_15 | GPIO_PIN_10);	//SPI1_NCS high, pull-up USART1_RX
	gpio_set_pin(GPIOB, GPIO_PIN_4);	//pull-up SPI1_MISO

	Delay_Init();
	uart_init(USART1, 115200);
	core_enable_irq(USART1_IRQn);

	cdc_init();
	cdc_set_serial_state(0x03);
	uint8_t prev_control_line_state = cdc_control_line_state;
	while(!cdc_config);	//Wait for host to configure the CDC interface
    printf("Alchemy - Nano-FOC\n");

    printf("SYSCLK: %u\n", rcc_compute_sysclk_freq());
	printf("HCLK: %u\n", rcc_compute_hclk_freq());
	printf("PCLK1: %u\n", rcc_compute_pclk1_freq());
	printf("PCLK2: %u\n", rcc_compute_pclk2_freq());
	printf("ADCCLK: %u\n", rcc_compute_adcclk());

	spi_init(SPI1, SPI_8_BIT | SPI_CLK_DIV_8 | SPI_MODE_1);
	afio_pcfr1_remap(AFIO_PCFR1_SPI1_REMAP);

	if(uart_bytes_available(uart1_rx_fifo))
	{
		printf("There is something fishy in the UART1 RX buffer...\n");
	}

	if(cdc_bytes_available())
	{
		printf("There is something fishy in the CDC RX buffer...\n");
	}

	uint8_t datagram[4];
	uint8_t address;
	uint16_t data;
	while(1)
	{
		if((cdc_bytes_available() >= 3) && (cdc_peek() & 0x80))	//handle write datagram
		{
			cdc_read_bytes(datagram, 3);

			address = datagram[0];
			((uint8_t*)&data)[1] = datagram[1];
			((uint8_t*)&data)[0] = datagram[2];

			nano_write_reg(address, data);
		}

		if((cdc_bytes_available() >= 1) && !(cdc_peek() & 0x80))	//handle read datagram
		{
			address = cdc_read_byte();

			data = nano_read_reg(address);
			datagram[0] = ((uint8_t*)&data)[1];
			datagram[1] = ((uint8_t*)&data)[0];
			cdc_write_bytes(datagram, 2);
		}

		if(prev_control_line_state != cdc_control_line_state)
		{
			cdc_set_serial_state(cdc_control_line_state & 3);
			prev_control_line_state = cdc_control_line_state;
		}
	}
}
