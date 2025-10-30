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
#include "ch32v203_afio.h"
#include "ch32v203_core.h"
#include "debug.h"
#include "ch32v203_exti.h"
#include "ch32v203_gpio.h"
#include "ch32v203_rcc.h"
#include "ch32v203_spi.h"
#include "fifo.h"
#include "ch32v203_uart.h"
#include "ch32v203_usbd_cdc.h"

uint16_t nano_read_reg(uint8_t addr);

//Pins:
// FOC_INT = PA7
// USART1_TX = PA9
// USART1_RX = PA10
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5

//FW registers
// 0x7FFF to 7FF8:	Reserved
// 0x7FF7:	RTMI Control
// 0x7FF6:	RTMI Num Samples
// 0x7FF5:	RTMI Threshold
// 0x7FF4:	RTMI Channel 0
// 0x7FF3:	RTMI Channel 1
// 0x7FF2:	RTMI Channel 2
// 0x7FF1:	RTMI Channel 3
// 0x7FF0:	RTMI Channel 4
// 0x7FEF:	RTMI Channel 5
// 0x7FEE:	RTMI Channel 6
// 0x7FED:	RTMI Channel 7

//RTMI Control Bitfields
// 3:0:		Trigger Mode
// 7:4:		Trigger Channel
// 11:8:	Num Channels
// 12:		Done
// 13:		Active
// 14:		Continuous Sampling
// 15:		Trigger

//RTMI trigger modes
// 0:	Unsigned greater than
// 1:	Unsigned less than
// 2:	Signed greater than
// 3:	Signed less than
// 4:	Equal to
// 5:	Not equal to
// 6:	Unconditional

uint16_t rtmi_control = 0;
uint16_t rtmi_num_samples = 0;
uint16_t rtmi_threshold = 0;
uint8_t rtmi_channels[8];

uint8_t trigger_mode = 0;
uint8_t trigger_channel = 0;
uint8_t num_channels = 0;
uint16_t rtmi_sample_count = 0;

void on_foc_int(void)
{
	uint16_t read_val;
	uint8_t channel_idx;
	uint8_t datagram[3];

	if(rtmi_control & 0x8000)	//waiting for trigger
	{
		read_val = nano_read_reg(rtmi_channels[trigger_channel]);

		switch(trigger_mode)
		{
			case 0x00:	//unsigned greater than
				if(read_val > rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x01:	//unsigned less than
				if(read_val < rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x02:	//signed greater than
				if((int16_t)read_val > (int16_t)rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x03:	//signed less than
				if((int16_t)read_val < (int16_t)rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x04:	//equal to
				if(read_val == rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x05:	//not equal to
				if(read_val != rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x06:	//unconditional
				rtmi_control |= 0x2000;	//set Active bit
				break;
			default: ;
		}

		if(rtmi_control & 0x2000)	//trigger condition met
		{
			rtmi_control &= 0x7FFF;	//clear Trigger bit
			rtmi_sample_count = 0;
		}
	}

	if(rtmi_control & 0x6000)	//triggered or in continuous sampling mode
	{
		for(channel_idx = 0; channel_idx < num_channels; ++channel_idx)
		{
			read_val = nano_read_reg(rtmi_channels[channel_idx]);
			datagram[0] = channel_idx;
			datagram[1] = ((uint8_t*)&read_val)[1];
			datagram[2] = ((uint8_t*)&read_val)[0];
			cdc_write_bytes(datagram, 3);
		}

		rtmi_sample_count += 1;
		if(rtmi_sample_count == rtmi_num_samples)	//done capturing
		{
			rtmi_control |= 0x1000;		//set Done bit
			rtmi_control &= ~0x2000;	//clear Active bit
		}
	}
}

void fw_write_rtmi_control(uint16_t val)
{
	rtmi_control = val;
	trigger_mode = (uint8_t)val & 0x0F;
	val = val >> 4;
	trigger_channel = (uint8_t)val & 0x0F;
	val = val >> 4;
	num_channels = (uint8_t)val & 0x0F;
}

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
	core_delay_us(1);

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

	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_9);	//USART1_TX
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PULL_IN, GPIO_PIN_10 | GPIO_PIN_7);	//USART1_RX, FOC_INT
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_15);	//SPI1_NCS
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5);	//SPI1_SCK, SPI1_MOSI
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4);			//SPI1_MISO

	gpio_set_pin(GPIOA, GPIO_PIN_15 | GPIO_PIN_10 | GPIO_PIN_7);	//SPI1_NCS high, pull-up USART1_RX, FOC_INT
	gpio_set_pin(GPIOB, GPIO_PIN_4);	//pull-up SPI1_MISO

	core_delay_init();
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

	afio_exti_config(AFIO_PORT_A, AFIO_PIN_7);
	exti_enable_rising_edge(EXTI_CHAN_7);
	exti_enable_falling_edge(EXTI_CHAN_7);
	exti_enable_interrupt(EXTI_CHAN_7);
	exti_clear_interrupt_flag(EXTI_CHAN_7);
	exti7_callback = on_foc_int;
	core_enable_irq(EXTI9_5_IRQn);

	if(uart_bytes_available(uart1_rx_fifo))
	{
		printf("There is something fishy in the UART1 RX buffer...\n");
	}

	if(cdc_bytes_available())
	{
		printf("There is something fishy in the CDC RX buffer...\n");
	}

	uint8_t datagram[4];
	uint16_t datagram_val;
	while(1)
	{
		if((cdc_bytes_available() >= 4) && (cdc_peek() & 0x80))	//Handle write datagram
		{
			cdc_read_bytes(datagram, 4);

			((uint8_t*)&datagram_val)[1] = datagram[2];
			((uint8_t*)&datagram_val)[0] = datagram[3];

			if(datagram[0] & 0x7F)	// Handle access to FW register
			{
				switch(datagram[1])
				{
					case 0xF7:	//RTMI Control
						fw_write_rtmi_control(datagram_val);
						break;
					case 0xF6:	//RTMI Num Samples
						rtmi_num_samples = datagram_val;
						break;
					case 0xF5:	//RTMI Threshold
						rtmi_threshold = datagram_val;
						break;
					case 0xF4:	//RTMI Channel 0
						rtmi_channels[0] = datagram_val;
						break;
					case 0xF3:	//RTMI Channel 1
						rtmi_channels[1] = datagram_val;
						break;
					case 0xF2:	//RTMI Channel 2
						rtmi_channels[2] = datagram_val;
						break;
					case 0xF1:	//RTMI Channel 3
						rtmi_channels[3] = datagram_val;
						break;
					case 0xF0:	//RTMI Channel 4
						rtmi_channels[4] = datagram_val;
						break;
					case 0xEF:	//RTMI Channel 5
						rtmi_channels[5] = datagram_val;
						break;
					case 0xEE:	//RTMI Channel 6
						rtmi_channels[6] = datagram_val;
						break;
					case 0xED:	//RTMI Channel 7
						rtmi_channels[7] = datagram_val;
						break;
					default: ;
				}
			}
			else	//Handle access to device register
			{
				nano_write_reg(datagram[1] | 0x80, datagram_val);
			}
		}

		if((cdc_bytes_available() >= 2) && !(cdc_peek() & 0x80))	//handle read datagram
		{
			cdc_read_bytes(datagram, 2);

			if(datagram[0] & 0x7F)	//Handle access to FW register
			{
				switch(datagram[1])
				{
					case 0xF7:	//RTMI Control
						datagram_val = rtmi_control;
						break;
					case 0xF6:	//RTMI Num Samples
						datagram_val = rtmi_num_samples;
						break;
					case 0xF5:	//RTMI Threshold
						datagram_val = rtmi_threshold;
						break;
					case 0xF4:	//RTMI Channel 0
						datagram_val = rtmi_channels[0];
						break;
					case 0xF3:	//RTMI Channel 1
						datagram_val = rtmi_channels[1];
						break;
					case 0xF2:	//RTMI Channel 2
						datagram_val = rtmi_channels[2];
						break;
					case 0xF1:	//RTMI Channel 3
						datagram_val = rtmi_channels[3];
						break;
					case 0xF0:	//RTMI Channel 4
						datagram_val = rtmi_channels[4];
						break;
					case 0xEF:	//RTMI Channel 5
						datagram_val = rtmi_channels[5];
						break;
					case 0xEE:	//RTMI Channel 6
						datagram_val = rtmi_channels[6];
						break;
					case 0xED:	//RTMI Channel 7
						datagram_val = rtmi_channels[7];
						break;
					default: ;
				}

				datagram[0] = 0xFF;
				datagram[1] = ((uint8_t*)&datagram_val)[1];
				datagram[2] = ((uint8_t*)&datagram_val)[0];
				cdc_write_bytes(datagram, 3);
			}
			else	//Handle access to device register
			{
				datagram_val = nano_read_reg(datagram[1] & 0x7F);
				datagram[0] = 0x80;
				datagram[1] = ((uint8_t*)&datagram_val)[1];
				datagram[2] = ((uint8_t*)&datagram_val)[0];
				cdc_write_bytes(datagram, 3);
			}
		}

		if(prev_control_line_state != cdc_control_line_state)
		{
			cdc_set_serial_state(cdc_control_line_state & 3);
			prev_control_line_state = cdc_control_line_state;
		}
	}
}
