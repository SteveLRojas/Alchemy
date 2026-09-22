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
//* Copyright (C) 2025 Esteban Looser-Rojas.       *
//* Contains Alchemy firmware for TM10 development.*
//* Using CDC for the host, and SPI1 and UART1 or  *
//* UART2 for the TM10 target.                     *
//**************************************************
#include "ch32v20x.h"
#include "ch32v203_afio.h"
#include "ch32v203_core.h"
#include "ch32v203_dma.h"
#include "ch32v203_gpio.h"
#include "ch32v203_rcc.h"
#include "ch32v203_spi.h"
#include "ch32v203_uart_dma.h"
#include "ch32v203_usbd_cdc.h"
#include "debug.h"

#define ALCHEMY_TO_LB_V10 1
#define ALCHEMY_TO_LB_V20 2
#define ALCHEMY_TO_LB_V30 3

#define ALC_UART1 1
#define ALC_UART2 2

#define ALCHEMY_UART_SEL ALC_UART1
#define ALCHEMY_TO_LB_VER ALCHEMY_TO_LB_V20

#if ALCHEMY_UART_SEL == ALC_UART1
#define TM10_UART_DMA uart_dma_1
#define TM10_UART_HW USART1
#else
#define TM10_UART_DMA uart_dma_2
#define TM10_UART_HW USART2
#endif

//Pins:
// NSLEEP = PA7 TODO: check if this is the same on both versions
// DRV_EN = PA8
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5
//For Alchemy_to_LB V1.0 only:
// USART1_TX = PB6
// USART1_RX = PB7
//For Alchemy_to_LB V2.0 only:
// USART2_TX = PA2
// USART2_RX = PA3

//FW registers
// 0x7FFF: Device ID
// 0x7FFE: Unique ID
// 0x7FFD:  Option Flags
// 0x7FFC:  Baud Rate
// 0x7FFB:  SPI clk div

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
uint32_t baud_rate = 6000000;
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

void tm10_write_baud_rate(uint32_t data)
{
	baud_rate = data;
	uart_dma_init(TM10_UART_HW, data);
}

void tm10_write_option_flags(uint32_t data)
{
	option_flags = data;
	gpio_write_pin(GPIOA, GPIO_PIN_7, ((uint8_t*)&data)[0] & 0x01);	//NSLEEP
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
	((uint8_t*)&data)[0] |= gpio_read_pin(GPIOA, GPIO_PIN_7);
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
	core_delay_us(1);

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
		uart_dma_write_bytes(TM10_UART_DMA, datagram, 7);
	}
	else
	{
		uart_dma_write_bytes(TM10_UART_DMA, datagram, 6);
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
		uart_dma_write_bytes(TM10_UART_DMA, datagram, 3);
	}
	else
	{
		uart_dma_write_bytes(TM10_UART_DMA, datagram, 2);
	}
}

uint32_t tm10_handle_rtmi_response(void)
{
	uint8_t datagram[6];
	uint32_t data;
	if(option_flags & 0x20)
	{
		uart_dma_read_bytes(TM10_UART_DMA, datagram, 6);
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
		uart_dma_read_bytes(TM10_UART_DMA, datagram, 5);
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
	uint32_t datagram_val;
	if(option_flags & 0x10)	//CRC enabled
	{
		uart_dma_read_bytes(TM10_UART_DMA, datagram, 7);
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
		uart_dma_read_bytes(TM10_UART_DMA, datagram, 6);
	}
	((uint8_t*)&datagram_val)[3] = datagram[2];
	((uint8_t*)&datagram_val)[2] = datagram[3];
	((uint8_t*)&datagram_val)[1] = datagram[4];
	((uint8_t*)&datagram_val)[0] = datagram[5];
	return datagram_val;
}

int main(void)
{
	rcc_apb2_clk_enable(RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN | RCC_TIM1EN | RCC_SPI1EN | RCC_USART1EN);
	rcc_apb1_clk_enable(RCC_TIM2EN | RCC_USART2EN | RCC_USBEN);
	rcc_ahb_clk_enable(RCC_DMA1EN);

	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15);	//NSLEEP, DRV_EN, SPI1_NCS
#if ALCHEMY_UART_SEL == ALC_UART1
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6);	//SPI1_SCK, SPI1_MOSI, USART1_TX
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4 | GPIO_PIN_7);	//SPI1_MISO, USART1_RX
	gpio_set_pin(GPIOB, GPIO_PIN_4 | GPIO_PIN_7);	//pull-up on SPI1_MISO, USART1_RX

	afio_pcfr1_remap(AFIO_PCFR1_USART1_REMAP);
	dma_init();
	uart_dma_init(USART1, baud_rate);
#else
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_2);	//USART2_TX
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_3);		//USART2_RX
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5);	//SPI1_SCK, SPI1_MOSI
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4);		//SPI1_MISO
	gpio_set_pin(GPIOA, GPIO_PIN_3);	//pull-up on USART2_RX
	gpio_set_pin(GPIOB, GPIO_PIN_4);	//pull-up on SPI1_MISO

	dma_init();
	uart_dma_init(USART2, baud_rate);
#endif

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

	core_delay_init();
	gpio_clear_pin(GPIOA, GPIO_PIN_2);
	gpio_clear_pin(GPIOA, GPIO_PIN_8);
	core_delay_ms(1);
	gpio_set_pin(GPIOA, GPIO_PIN_2);

	spi_init(SPI1, SPI_8_BIT | SPI_CLK_DIV_8 | SPI_MODE_1);
	afio_pcfr1_remap(AFIO_PCFR1_SPI1_REMAP);
	gpio_set_pin(GPIOA, GPIO_PIN_15);

    if(uart_dma_bytes_available(TM10_UART_DMA))
	{
		printf("There is something fishy in the UART RX buffer...\n");
	}

    if(cdc_bytes_available())
	{
		printf("There is something fishy in the CDC RX buffer...\n");
	}

    uint8_t datagram[6];
    uint16_t address;
    uint32_t datagram_val;
	while(1)
	{
		if((cdc_bytes_available() >= 6) && (cdc_peek() & 0x80))	//handle write datagram
		{
			cdc_read_bytes(datagram, 6);

			((uint8_t*)&address)[1] = datagram[0];
			((uint8_t*)&address)[0] = datagram[1];
			((uint8_t*)&datagram_val)[3] = datagram[2];
			((uint8_t*)&datagram_val)[2] = datagram[3];
			((uint8_t*)&datagram_val)[1] = datagram[4];
			((uint8_t*)&datagram_val)[0] = datagram[5];

			if(datagram[0] & 0x70)  // Handle access to FW register
			{
			    switch(datagram[1])
			    {
                    case 0xFD:  //option flags
                        tm10_write_option_flags(datagram_val);
                        break;
                    case 0xFC:	//baud rate
                    	tm10_write_baud_rate(datagram_val);
                    	break;
                    case 0xFB:	//SPI clk div
                    	spi_init(SPI1, SPI_8_BIT | (uint16_t)datagram_val | SPI_MODE_1);
                    	spi_transfer(SPI1, 0x0000);
                    	break;
                    default: ;
			    }
			}
			else
			{
				if(option_flags & 0x04)	//use UART
				{
					tm10_write_request_uart(address, datagram_val);
				}
				else	//use SPI
				{
					tm10_write_reg_spi(address, datagram_val);
				}
			}
		}

		if((cdc_bytes_available() >= 2) && !(cdc_peek() & 0x80))	//handle read datagram
		{
			cdc_read_bytes(datagram, 2);
			((uint8_t*)&address)[1] = datagram[0];
			((uint8_t*)&address)[0] = datagram[1];

			if(datagram[0] & 0x70)	//Handle access to FW register
			{
				switch(datagram[1])
				{
					case 0xFF: //device_id
						datagram_val = 0x30314D54;
						break;
					case 0xFE:	//unique_id
						datagram_val = *(volatile uint32_t*)0x1FFFF7E8;
						break;
					case 0xFD:	//option flags
						datagram_val = tm10_read_option_flags();
						break;
					case 0xFC:	//baud rate
						datagram_val = baud_rate;
						break;
					case 0xFB:	//SPI clk div
						datagram_val = SPI1->CTLR1 & SPI_CTLR1_BR;
						break;
					default: datagram_val = 0;
				}

				datagram[0] = 0xFF;
				datagram[1] = ((uint8_t*)&datagram_val)[3];
				datagram[2] = ((uint8_t*)&datagram_val)[2];
				datagram[3] = ((uint8_t*)&datagram_val)[1];
				datagram[4] = ((uint8_t*)&datagram_val)[0];
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
					datagram_val = tm10_read_reg_spi(address);
					datagram[0] = 0x80;
					datagram[1] = ((uint8_t*)&datagram_val)[3];
					datagram[2] = ((uint8_t*)&datagram_val)[2];
					datagram[3] = ((uint8_t*)&datagram_val)[1];
					datagram[4] = ((uint8_t*)&datagram_val)[0];
					cdc_write_bytes(datagram, 5);
				}
			}
		}

		if((uart_dma_bytes_available(TM10_UART_DMA) >= rtmi_res_len) && (uart_dma_peek(TM10_UART_DMA) & 0x01)) //handle write response or RTMI
		{
			datagram_val = tm10_handle_rtmi_response();
			if(!(option_flags & 0x08) || (datagram_val != prev_rtmi_responses[rtmi_channel_id]))
			{
				datagram[0] = rtmi_channel_id;
				datagram[1] = ((uint8_t*)&datagram_val)[3];
				datagram[2] = ((uint8_t*)&datagram_val)[2];
				datagram[3] = ((uint8_t*)&datagram_val)[1];
				datagram[4] = ((uint8_t*)&datagram_val)[0];
				cdc_write_bytes(datagram, 5);
			}
			prev_rtmi_responses[rtmi_channel_id] = datagram_val;
		}

		if((uart_dma_bytes_available(TM10_UART_DMA) >= read_res_len) && !(uart_dma_peek(TM10_UART_DMA) & 0x01)) //handle read response
		{
			datagram_val = tm10_handle_read_response();
			datagram[0] = 0x40;
			datagram[1] = ((uint8_t*)&datagram_val)[3];
			datagram[2] = ((uint8_t*)&datagram_val)[2];
			datagram[3] = ((uint8_t*)&datagram_val)[1];
			datagram[4] = ((uint8_t*)&datagram_val)[0];
			cdc_write_bytes(datagram, 5);
		}

		if(prev_control_line_state != cdc_control_line_state)
		{
			cdc_set_serial_state(cdc_control_line_state & 3);
			prev_control_line_state = cdc_control_line_state;
		}
	}
}
