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
// NSLEEP = PA2
// DRV_EN = PA8
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// CLK16 = PB0
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5
// USART1_TX = PB6
// USART1_RX = PB7

//Control flags:
// 0: NSLEEP
// 1: DRV_EN
// 2: USE_UART
// 3: RTMI_NO_REPEAT
// 4: CRC_ENABLE
// 5: RTMI_CRC_ENABLE
// 6: CRC_OK
// 7: RTMI_CRC_OK

uint32_t option_flags = 0;
uint32_t prev_rtmi_responses[8];
uint8_t rtmi_channel_id;
uint8_t read_res_len = 6;
uint8_t rtmi_res_len = 5;

uint8_t crc8_poly = 0x1D;
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

void tm10_write_option_flags(uint32_t data)
{
	option_flags = data;
	gpio_write_pin(GPIOA, GPIO_PIN_2, ((uint8_t*)&data)[0] & 0x01);	//NSLEEP
	gpio_write_pin(GPIOA, GPIO_PIN_8, ((uint8_t*)&data)[0] & 0x02);	//DRV_EN
	read_res_len = 6;
	rtmi_res_len = 5;
	if(option_flags & 0x10)
		read_res_len = 7;
	if(option_flags & 0x20)
		rtmi_res_len = 6;
}

uint32_t tm10_read_option_flags(void)
{
	uint32_t data;

	data = option_flags & 0xFFFFFFFC;
	((uint8_t*)&data)[0] |= gpio_read_pin(GPIOA, GPIO_PIN_2);
	((uint8_t*)&data)[0] |= gpio_read_pin(GPIOA, GPIO_PIN_8) << 1;
	return data;
}

void tm10_write_reg_spi(uint16_t addr, uint32_t data)
{
	gpio_clear_pin(GPIOA, GPIO_PIN_15);

	spi_transfer(SPI1, ((uint8_t*)&addr)[1]);
	spi_transfer(SPI1, ((uint8_t*)&addr)[0]);

	spi_transfer(SPI1, ((uint8_t*)&data)[3]);
	spi_transfer(SPI1, ((uint8_t*)&data)[2]);
	spi_transfer(SPI1, ((uint8_t*)&data)[1]);
	spi_transfer(SPI1, ((uint8_t*)&data)[0]);

	gpio_set_pin(GPIOA, GPIO_PIN_15);
	return;
}

uint32_t tm10_read_reg_spi(uint16_t addr)
{
	uint32_t read_val;

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	spi_transfer(SPI1, ((uint8_t*)&addr)[1]);
	spi_transfer(SPI1, ((uint8_t*)&addr)[0]);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);
	Delay_Us(1);

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	(void)spi_transfer(SPI1, ((uint8_t*)&addr)[1]);
	(void)spi_transfer(SPI1, ((uint8_t*)&addr)[0]);
	((uint8_t*)&read_val)[3] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[2] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[1] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[0] = (uint8_t)spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);

	return read_val;
}

void tm10_write_request_uart(uint16_t addr, uint32_t data)
{
	uint8_t datagram[7];
	uint8_t temp = 0x4A;
	temp |= (uint8_t)(addr >> 4) & 0x30;

	datagram[0] = temp;
	datagram[1] = (uint8_t)addr;
	datagram[2] = ((uint8_t*)&data)[3];
	datagram[3] = ((uint8_t*)&data)[2];
	datagram[4] = ((uint8_t*)&data)[1];
	datagram[5] = ((uint8_t*)&data)[0];
	if(option_flags & 0x10)	//CRC enabled
	{
		crc8_lfsr = 0;
		crc8_calc_lsb_first(datagram, 6);
		datagram[6] = crc8_lfsr;
		uart_write_bytes(USART1, uart1_tx_fifo, datagram, 7);
	}
	else
	{
		uart_write_bytes(USART1, uart1_tx_fifo, datagram, 6);
	}

}

void tm10_read_request_uart(uint16_t addr)
{
	uint8_t datagram[3];
	uint8_t temp = 0x42;
	temp |= (uint8_t)(addr >> 4) & 0x30;

	datagram[0] = temp;
	datagram[1] = (uint8_t)addr;
	if(option_flags & 0x10)	//CRC enabled
	{
		crc8_lfsr = 0;
		crc8_calc_lsb_first(datagram, 2);
		datagram[2] = crc8_lfsr;
		uart_write_bytes(USART1, uart1_tx_fifo, datagram, 3);
	}
	else
	{
		uart_write_bytes(USART1, uart1_tx_fifo, datagram, 2);
	}
}

uint32_t tm10_handle_rtmi_response(void)
{
	uint8_t datagram[6];
	uint32_t data;
	if(option_flags & 0x20)
	{
		uart_read_bytes(USART1, uart1_rx_fifo, datagram, 6);
		crc8_lfsr = 0;
		crc8_calc_lsb_first(datagram, 5);
		option_flags &= ~(uint16_t)0x80;
		if(datagram[5] == crc8_lfsr)
		{
			option_flags |= 0x80;
		}
	}
	else
	{
		uart_read_bytes(USART1, uart1_rx_fifo, datagram, 5);
	}
	rtmi_channel_id = (datagram[0] >> 1) & 0x07;
	((uint8_t*)&data)[3] = datagram[1];
	((uint8_t*)&data)[2] = datagram[2];
	((uint8_t*)&data)[1] = datagram[3];
	((uint8_t*)&data)[0] = datagram[4];
	return data;
}

uint32_t tm10_handle_read_response(void)
{
	uint8_t datagram[7];
	uint32_t data;
	if(option_flags & 0x10)	//CRC enabled
	{
		uart_read_bytes(USART1, uart1_rx_fifo, datagram, 7);
		crc8_lfsr = 0;
		crc8_calc_lsb_first(datagram, 6);
		option_flags &= ~(uint16_t)0x40;
		if(datagram[6] == crc8_lfsr)
		{
			option_flags |= 0x40;
		}
	}
	else
	{
		uart_read_bytes(USART1, uart1_rx_fifo, datagram, 6);
	}
	((uint8_t*)&data)[3] = datagram[2];
	((uint8_t*)&data)[2] = datagram[3];
	((uint8_t*)&data)[1] = datagram[4];
	((uint8_t*)&data)[0] = datagram[5];
	return data;
}

int main(void)
{
	rcc_apb2_clk_enable(RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN | RCC_TIM1EN | RCC_SPI1EN | RCC_USART1EN);
	rcc_apb1_clk_enable(RCC_TIM2EN | RCC_USBEN);

	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_15);	//NSLEEP, DRV_EN, SPI1_NCS
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_0);	//CLK16
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6);	//SPI1_SCK, SPI1_MOSI, USART1_TX
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4 | GPIO_PIN_7);			//SPI1_MISO, USART1_RX
	gpio_clear_pin(GPIOB, GPIO_PIN_0);
	gpio_set_pin(GPIOB, GPIO_PIN_4 | GPIO_PIN_7);

    Delay_Init();
    afio_pcfr1_remap(AFIO_PCFR1_USART1_REMAP);
    uart_init(USART1, 6000000);
    core_enable_irq(USART1_IRQn);

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

	gpio_clear_pin(GPIOA, GPIO_PIN_2);
	gpio_clear_pin(GPIOA, GPIO_PIN_8);
	Delay_Ms(1);
	gpio_set_pin(GPIOA, GPIO_PIN_2);

	spi_init(SPI1, SPI_8_BIT | SPI_CLK_DIV_8 | SPI_MODE_1);
	afio_pcfr1_remap(AFIO_PCFR1_SPI1_REMAP);
	gpio_set_pin(GPIOA, GPIO_PIN_15);

    if(uart_bytes_available(uart1_rx_fifo))
	{
		printf("There is something fishy in the UART1 RX buffer...\n");
	}

    if(cdc_bytes_available())
	{
		printf("There is something fishy in the CDC RX buffer...\n");
	}

    uint8_t datagram[6];
    uint16_t address;
    uint32_t data;
	while(1)
	{
		if((cdc_bytes_available() >= 6) && (cdc_peek() & 0x80))	//handle write datagram
		{
			cdc_read_bytes(datagram, 6);

			((uint8_t*)&address)[1] = datagram[0];
			((uint8_t*)&address)[0] = datagram[1];
			((uint8_t*)&data)[3] = datagram[2];
			((uint8_t*)&data)[2] = datagram[3];
			((uint8_t*)&data)[1] = datagram[4];
			((uint8_t*)&data)[0] = datagram[5];

			if(address == 0xFFFF)	//handle control write
			{
				tm10_write_option_flags(data);
			}
			else
			{
				if(option_flags & 0x04)	//use UART
				{
					tm10_write_request_uart(address, data);
				}
				else	//use SPI
				{
					tm10_write_reg_spi(address, data);
				}
			}
		}

		if((cdc_bytes_available() >= 2) && !(cdc_peek() & 0x80))	//handle read datagram
		{
			cdc_read_bytes(datagram, 2);
			((uint8_t*)&address)[1] = datagram[0];
			((uint8_t*)&address)[0] = datagram[1];

			if(address == 0x7FFF)	//handle control read
			{
				data = tm10_read_option_flags();
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
					tm10_read_request_uart(address);
				}
				else	//use SPI
				{
					data = tm10_read_reg_spi(address);
					datagram[0] = 0x80;
					datagram[1] = ((uint8_t*)&data)[3];
					datagram[2] = ((uint8_t*)&data)[2];
					datagram[3] = ((uint8_t*)&data)[1];
					datagram[4] = ((uint8_t*)&data)[0];
					cdc_write_bytes(datagram, 5);
				}
			}
		}

		/*if(cdc_bytes_available() >= 2)	//CRC test
		{
			cdc_read_bytes(datagram, 2);

			crc8_lfsr = 0;
			crc8_calc_lsb_first(datagram, 2);
			cdc_write_byte(crc8_lfsr);
		}*/

		if((uart_bytes_available(uart1_rx_fifo) >= rtmi_res_len) && (uart_peek(uart1_rx_fifo) & 0x01)) //handle write response or RTMI
		{
			data = tm10_handle_rtmi_response();
			if(!(option_flags & 0x08) || (data != prev_rtmi_responses[rtmi_channel_id]))
			{
				datagram[0] = rtmi_channel_id;
				datagram[1] = ((uint8_t*)&data)[3];
				datagram[2] = ((uint8_t*)&data)[2];
				datagram[3] = ((uint8_t*)&data)[1];
				datagram[4] = ((uint8_t*)&data)[0];
				cdc_write_bytes(datagram, 5);
			}
			prev_rtmi_responses[rtmi_channel_id] = data;
		}

		if((uart_bytes_available(uart1_rx_fifo) >= read_res_len) && !(uart_peek(uart1_rx_fifo) & 0x01)) //handle read response
		{
			data = tm10_handle_read_response();
			datagram[0] = 0x40;
			datagram[1] = ((uint8_t*)&data)[3];
			datagram[2] = ((uint8_t*)&data)[2];
			datagram[3] = ((uint8_t*)&data)[1];
			datagram[4] = ((uint8_t*)&data)[0];
			cdc_write_bytes(datagram, 5);
		}

		if(prev_control_line_state != cdc_control_line_state)
		{
			cdc_set_serial_state(cdc_control_line_state & 3);
			prev_control_line_state = cdc_control_line_state;
		}
	}
}
