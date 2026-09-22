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
// CLK16 = PA1
// USART2_TX = PA2
// USART2_RX = PA3
// IREF_R3 = PA5
// NDRV_EN = PA8
// USART1_TX = PA9
// USART1_RX = PA10
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// UART_EN = PB0
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5
// IREF_R2 = PB10
// NSLEEP = PC15

//Control flags:
// 0: NSLEEP
// 1: NDRV_EN
// 2: UART_EN
// 3: IREF_R2
// 4: IREF_R3
// 5: CRC_OK

uint32_t option_flags = 0;

uint8_t crc8_poly = 0x07;
uint8_t crc8_lfsr;
void crc8_calc_lsb_first(uint8_t* data, uint16_t len)
{
	uint8_t current_byte;
	for(uint8_t d = 0; d < len; ++d)
	{
		current_byte = data[d];
		for(uint8_t i = 0; i < 8; ++i)
		{
			if((crc8_lfsr >> 7) ^ (current_byte & 0x01))
			{
				crc8_lfsr = (crc8_lfsr << 1) ^ crc8_poly;
			}
			else
			{
				crc8_lfsr = crc8_lfsr << 1;
			}
			current_byte = current_byte >> 1;
		}
	}
}

void tm11_write_option_flags(uint32_t data)
{
	option_flags = data;
	gpio_write_pin(GPIOC, GPIO_PIN_15, data & 0x01);	//NSLEEP
	gpio_write_pin(GPIOA, GPIO_PIN_8, data & 0x02);		//NDRV_EN
	gpio_write_pin(GPIOB, GPIO_PIN_0, data & 0x04);		//UART_EN
	gpio_write_pin(GPIOB, GPIO_PIN_10, data & 0x08);	//IREF_R2
	gpio_write_pin(GPIOA, GPIO_PIN_5, data & 0x10);		//IREF_R3
	(void)spi_transfer(SPI1, 0x00);	//make sure MISO is low
}

uint32_t tm11_read_option_flags(void)
{
	uint32_t data;

	data = option_flags & 0xFFFFFFE0;
	data |= gpio_read_pin(GPIOC, GPIO_PIN_15);		//NSLEEP
	data |= gpio_read_pin(GPIOA, GPIO_PIN_8) << 1;	//NDRV_EN
	data |= gpio_read_pin(GPIOB, GPIO_PIN_0) << 2;	//UART_EN
	data |= gpio_read_pin(GPIOB, GPIO_PIN_10) << 3;	//IREF_R2
	data |= gpio_read_pin(GPIOA, GPIO_PIN_5) << 4;	//IREF_R3
	return data;
}

void tm11_write_reg_spi(uint8_t addr, uint32_t data)
{
	gpio_clear_pin(GPIOA, GPIO_PIN_15);

	spi_transfer(SPI1, addr);
	spi_transfer(SPI1, ((uint8_t*)&data)[3]);
	spi_transfer(SPI1, ((uint8_t*)&data)[2]);
	spi_transfer(SPI1, ((uint8_t*)&data)[1]);
	spi_transfer(SPI1, ((uint8_t*)&data)[0]);

	gpio_set_pin(GPIOA, GPIO_PIN_15);
	return;
}

uint32_t tm11_read_reg_spi(uint8_t addr)
{
	uint32_t read_val;

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	spi_transfer(SPI1, addr);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);
	Delay_Us(1);

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	(void)spi_transfer(SPI1, addr);
	((uint8_t*)&read_val)[3] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[2] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[1] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[0] = (uint8_t)spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);

	return read_val;
}

void tm11_write_request_uart(uint8_t addr, uint32_t data)
{
	uint8_t datagram[8];

	datagram[0] = 0xA5;
	datagram[1] = ((uint8_t*)&option_flags)[3];	//Node address
	datagram[2] = addr;
	datagram[3] = ((uint8_t*)&data)[3];
	datagram[4] = ((uint8_t*)&data)[2];
	datagram[5] = ((uint8_t*)&data)[1];
	datagram[6] = ((uint8_t*)&data)[0];
	crc8_lfsr = 0;
	crc8_calc_lsb_first(datagram, 7);
	datagram[7] = crc8_lfsr;
	uart_write_bytes(USART2, uart2_tx_fifo, datagram, 8);
}

void tm11_read_request_uart(uint8_t addr)
{
	uint8_t datagram[4];

	datagram[0] = 0x55;
	datagram[1] = ((uint8_t*)&option_flags)[3];	//Node address
	datagram[2] = addr;
	crc8_lfsr = 0;
	crc8_calc_lsb_first(datagram, 3);
	datagram[3] = crc8_lfsr;
	uart_write_bytes(USART2, uart2_tx_fifo, datagram, 4);
}

uint32_t tm11_handle_read_response(void)
{
	uint8_t datagram[8];
	uint32_t data;

	uart_read_bytes(USART2, uart2_rx_fifo, datagram, 8);
	crc8_lfsr = 0;
	crc8_calc_lsb_first(datagram, 7);
	option_flags &= ~(uint16_t)0x20;
	if(datagram[7] == crc8_lfsr)
	{
		option_flags |= 0x20;	//CRC ok
	}

	((uint8_t*)&data)[3] = datagram[3];
	((uint8_t*)&data)[2] = datagram[4];
	((uint8_t*)&data)[1] = datagram[5];
	((uint8_t*)&data)[0] = datagram[6];
	return data;
}

int main(void)
{
	rcc_apb2_clk_enable(RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN | RCC_TIM1EN | RCC_SPI1EN | RCC_USART1EN);
	rcc_apb1_clk_enable(RCC_TIM2EN | RCC_USBEN | RCC_USART2EN);

	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15);	//UART_EN, IREF_R3, NDRV_EN, SPI1_NCS
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_2 | GPIO_PIN_9);	//USART2_TX, USART1_TX
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PULL_IN, GPIO_PIN_3 | GPIO_PIN_10);	//USART2_RX, USART1_RX
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_0 | GPIO_PIN_10);	//CLK16, IREF_R2
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5);	//SPI1_SCK, SPI1_MOSI
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4);			//SPI1_MISO
	gpio_set_mode(GPIOC, GPIO_DIR_SPD_OUT_2MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_15);	//NSLEEP

	gpio_clear_pin(GPIOA, GPIO_PIN_1 | GPIO_PIN_5);	//CLK16, IREF_R3 low
	gpio_clear_pin(GPIOB, GPIO_PIN_0 | GPIO_PIN_10);	//UART_EN, IREF_R2 low
	gpio_set_pin(GPIOA, GPIO_PIN_8 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_10);	//DNRV_EN, SPI1_NCS high, pull-up USART2_RX, USART1_RX
	gpio_set_pin(GPIOB, GPIO_PIN_4);	//pull-up SPI1_MISO
	gpio_set_pin(GPIOC, GPIO_PIN_15);	//NSLEEP high

    Delay_Init();
    uart_init(USART1, 115200);
    core_enable_irq(USART1_IRQn);
    uart_init(USART2, 500000);
    core_enable_irq(USART2_IRQn);

    cdc_init();
	cdc_set_serial_state(0x03);
	uint8_t prev_control_line_state = cdc_control_line_state;
	while(!cdc_config);	//Wait for host to configure the CDC interface
	printf("Unicorn\n");

    printf("SYSCLK: %u\n", rcc_compute_sysclk_freq());
	printf("HCLK: %u\n", rcc_compute_hclk_freq());
	printf("PCLK1: %u\n", rcc_compute_pclk1_freq());
	printf("PCLK2: %u\n", rcc_compute_pclk2_freq());
	printf("ADCCLK: %u\n", rcc_compute_adcclk());

	spi_init(SPI1, SPI_8_BIT | SPI_CLK_DIV_8 | SPI_MODE_3);
	afio_pcfr1_remap(AFIO_PCFR1_SPI1_REMAP);

    if(uart_bytes_available(uart1_rx_fifo))
	{
		printf("There is something fishy in the UART1 RX buffer...\n");
	}

    if(cdc_bytes_available())
	{
		printf("There is something fishy in the CDC RX buffer...\n");
	}

    uint8_t datagram[8];
    uint8_t address;
    uint32_t data;
    uint16_t bytes_available;
	while(1)
	{
		if((cdc_bytes_available() >= 5) && (cdc_peek() & 0x80))	//handle write datagram
		{
			cdc_read_bytes(datagram, 5);

			address = datagram[0];
			((uint8_t*)&data)[3] = datagram[1];
			((uint8_t*)&data)[2] = datagram[2];
			((uint8_t*)&data)[1] = datagram[3];
			((uint8_t*)&data)[0] = datagram[4];

			if(address == 0xFF)	//handle control write
			{
				tm11_write_option_flags(data);
			}
			else
			{
				if(option_flags & 0x04)	//use UART
				{
					tm11_write_request_uart(address, data);
				}
				else	//use SPI
				{
					tm11_write_reg_spi(address, data);
				}
			}
		}

		if((cdc_bytes_available() >= 1) && !(cdc_peek() & 0x80))	//handle read datagram
		{
			address = cdc_read_byte();

			if(address == 0x7F)	//handle control read
			{
				data = tm11_read_option_flags();
				datagram[0] = 0xFF;
				datagram[1] = ((uint8_t*)&data)[3];
				datagram[2] = ((uint8_t*)&data)[2];
				datagram[3] = ((uint8_t*)&data)[1];
				datagram[4] = ((uint8_t*)&data)[0];
				cdc_write_bytes(datagram, 5);
			}
			else
			{
				if(option_flags & 0x04)	//use UART
				{
					tm11_read_request_uart(address);
				}
				else	//use SPI
				{
					data = tm11_read_reg_spi(address);
					datagram[0] = 0x80;
					datagram[1] = ((uint8_t*)&data)[3];
					datagram[2] = ((uint8_t*)&data)[2];
					datagram[3] = ((uint8_t*)&data)[1];
					datagram[4] = ((uint8_t*)&data)[0];
					cdc_write_bytes(datagram, 5);
				}
			}
		}

		bytes_available = uart_bytes_available(uart2_rx_fifo);
		if((bytes_available >= 8) && (uart_peek(uart2_rx_fifo) == 0xA5))	//handle write request echo
		{
			uart_read_bytes(USART2, uart2_rx_fifo, datagram, 8);
		}
		else if((bytes_available >= 4) && (uart_peek(uart2_rx_fifo) == 0x55))	//handle read request echo
		{
			uart_read_bytes(USART2, uart2_rx_fifo, datagram, 4);
		}
		else if((bytes_available >= 8) && (uart_peek(uart2_rx_fifo) == 0x05))	//handle read response
		{
			data = tm11_handle_read_response();
			datagram[0] = 0x40;
			datagram[1] = ((uint8_t*)&data)[3];
			datagram[2] = ((uint8_t*)&data)[2];
			datagram[3] = ((uint8_t*)&data)[1];
			datagram[4] = ((uint8_t*)&data)[0];
			cdc_write_bytes(datagram, 5);
		}
		else if(bytes_available >= 8)	//handle garbage bytes
		{
			(void)uart_read_byte(USART2, uart2_rx_fifo);
		}

		if(prev_control_line_state != cdc_control_line_state)
		{
			cdc_set_serial_state(cdc_control_line_state & 3);
			prev_control_line_state = cdc_control_line_state;
		}
	}
}
