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
//* Contains Alchemy firmware for the TMC5160_Dev  *
//* project. Currently using CDC for the host,     *
//* and SPI1 for the TMC5160 target.               *
//**************************************************

#include "ch32v20x.h"
#include "ch32v203_afio.h"
#include "ch32v203_core.h"
#include "debug.h"
#include "ch32v203_exti.h"
#include "ch32v203_gpio.h"
#include "ch32v203_rcc.h"
#include "ch32v203_spi.h"
#include "ch32v203_timer.h"
#include "fifo.h"
#include "ch32v203_uart.h"
#include "ch32v203_usbd_cdc.h"

uint32_t tmc5130_read_reg(uint8_t addr);

//Pins:
// DIAG0 = PA0
// DIAG1 = PA1
// DRV_EN = PA2
// USART1_TX = PA9
// USART1_RX = PA10
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5
// LED3 = PB6
// LED2 = PB7
// LED1 = PB8
// LED0 = PB9
// ENCN = PB11
// ENCA = PB12
// ENCB = PB13
// DIR = PB14
// STEP = PB15

//FW registers
// 0x7FFF:	Reserved for Interface Mode
// 0x7FFE:	Reserved for UART baud
// 0x7FFD:	Reserved for UART baud
// 0x7FFC:	Reserved for SPI clk div
// 0x7FFB:	Reserved for Pin Mode
// 0x7FFA:	Pin Data
// 0x7FF9:	Pin Set
// 0x7FF8:	Pin Clear
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
// 0x7FEC:	RTMI Period (in microseconds)

//Pin Data bits
// 0:		LED0
// 1:		LED1
// 2:		LED2
// 3:		LED3
// 4:		DRV_EN
// 5:		DIAG0
// 6:		DIAG1

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
uint32_t rtmi_num_samples = 0;
uint32_t rtmi_threshold = 0;
uint8_t rtmi_channels[8];

uint8_t trigger_mode = 0;
uint8_t trigger_channel = 0;
uint8_t num_channels = 0;
uint32_t rtmi_sample_count = 0;

void on_rtmi_int(void)
{
	uint32_t read_val;
	uint8_t channel_idx;
	uint8_t datagram[5];

	if(rtmi_control & 0x8000)	//waiting for trigger
	{
		read_val = tmc5130_read_reg(rtmi_channels[trigger_channel]);

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
				if((int32_t)read_val > (int32_t)rtmi_threshold)
					rtmi_control |= 0x2000;	//set Active bit
				break;
			case 0x03:	//signed less than
				if((int32_t)read_val < (int32_t)rtmi_threshold)
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
			read_val = tmc5130_read_reg(rtmi_channels[channel_idx]);
			datagram[0] = channel_idx;
			datagram[1] = ((uint8_t*)&read_val)[3];
			datagram[2] = ((uint8_t*)&read_val)[2];
			datagram[3] = ((uint8_t*)&read_val)[1];
			datagram[4] = ((uint8_t*)&read_val)[0];
			cdc_write_bytes(datagram, 5);
		}

		rtmi_sample_count += 1;
		if(rtmi_sample_count == rtmi_num_samples)	//done capturing
		{
			rtmi_control |= 0x1000;		//set Done bit
			rtmi_control &= ~0x2000;	//clear Active bit
		}
	}
}

void fw_write_pin_data(uint8_t pin_val)
{
	gpio_write_pin(GPIOB, GPIO_PIN_9, pin_val & 0x01);	//LED0
	pin_val = pin_val >> 1;
	gpio_write_pin(GPIOB, GPIO_PIN_8, pin_val & 0x01);	//LED1
	pin_val = pin_val >> 1;
	gpio_write_pin(GPIOB, GPIO_PIN_7, pin_val & 0x01);	//LED2
	pin_val = pin_val >> 1;
	gpio_write_pin(GPIOB, GPIO_PIN_6, pin_val & 0x01);	//LED3
	pin_val = pin_val >> 1;
	gpio_write_pin(GPIOA, GPIO_PIN_2, pin_val & 0x01);	//DRV_EN
}

uint8_t fw_read_pin_data(void)
{
	uint8_t val;

	val = gpio_read_pin(GPIOA, GPIO_PIN_1);		//DIAG1
	val = val << 1;
	val |= gpio_read_pin(GPIOA, GPIO_PIN_0);	//DIAG0
	val = val << 1;
	val |= gpio_read_pin(GPIOA, GPIO_PIN_2);	//DRV_EN
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_6);	//LED3
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_7);	//LED2
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_8);	//LED1
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_9);	//LED0

	return val;
}

void fw_pin_set(uint8_t pin_val)
{
	if(pin_val & 0x01)
		gpio_set_pin(GPIOB, GPIO_PIN_9);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_set_pin(GPIOB, GPIO_PIN_8);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_set_pin(GPIOB, GPIO_PIN_7);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_set_pin(GPIOB, GPIO_PIN_6);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_set_pin(GPIOA, GPIO_PIN_2);
}

void fw_pin_clear(uint8_t pin_val)
{
	if(pin_val & 0x01)
		gpio_clear_pin(GPIOB, GPIO_PIN_9);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_clear_pin(GPIOB, GPIO_PIN_8);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_clear_pin(GPIOB, GPIO_PIN_7);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_clear_pin(GPIOB, GPIO_PIN_6);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)
		gpio_clear_pin(GPIOA, GPIO_PIN_2);
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

void tmc5130_write_reg(uint8_t addr, uint32_t data)
{
	gpio_clear_pin(GPIOA, GPIO_PIN_15);

	spi_transfer(SPI1, (uint16_t)addr);
	spi_transfer(SPI1, ((uint8_t*)&data)[3]);
	spi_transfer(SPI1, ((uint8_t*)&data)[2]);
	spi_transfer(SPI1, ((uint8_t*)&data)[1]);
	spi_transfer(SPI1, ((uint8_t*)&data)[0]);

	gpio_set_pin(GPIOA, GPIO_PIN_15);
	return;
}

uint32_t tmc5130_read_reg(uint8_t addr)
{
	uint32_t read_val;

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	spi_transfer(SPI1, (uint16_t)addr);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);
	delay_us(1);

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	(void)spi_transfer(SPI1, addr);
	((uint8_t*)&read_val)[3] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[2] = (uint8_t)spi_transfer(SPI1, 0x0000);
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
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PULL_IN, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10);	//DIAG0, DIAG1, USART1_RX
	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_2 | GPIO_PIN_15);	//DRV_EN, SPI1_NCS

	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5);	//SPI1_SCK, SPI1_MOSI
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4);			//SPI1_MISO
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);	//LED3, LED2, LED1, LED0

	gpio_set_pin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_2 | GPIO_PIN_15);	//pull-up DIAG0, DIAG1 USART1_RX, set DRV_EN, SPI1_NCS high
	gpio_set_pin(GPIOB, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);	//pull-up SPI1_MISO, set LED3, LED2, LED1, LED0

	delay_init();
	uart_init(USART1, 115200);
	core_enable_irq(USART1_IRQn);

	cdc_init();
	cdc_set_serial_state(0x03);
	uint8_t prev_control_line_state = cdc_control_line_state;
	while(!cdc_config);	//Wait for host to configure the CDC interface
    printf("Alchemy - TMC5160_Dev\n");

    printf("SYSCLK: %u\n", rcc_compute_sysclk_freq());
	printf("HCLK: %u\n", rcc_compute_hclk_freq());
	printf("PCLK1: %u\n", rcc_compute_pclk1_freq());
	printf("PCLK2: %u\n", rcc_compute_pclk2_freq());
	printf("ADCCLK: %u\n", rcc_compute_adcclk());

	spi_init(SPI1, SPI_8_BIT | SPI_CLK_DIV_16 | SPI_MODE_3);	//6 MHz
	afio_pcfr1_remap(AFIO_PCFR1_SPI1_REMAP);

	timer_init(TIM1, 96000000 / 1000000 - 1, 1000 - 1);	//1us time scale, 1ms period
	timer1_callback = on_rtmi_int;
	timer_enable_interrupt(TIM1);
	core_enable_irq(TIM1_UP_IRQn);
	timer_start(TIM1);

	if(uart_bytes_available(uart1_rx_fifo))
	{
		printf("There is something fishy in the UART1 RX buffer...\n");
	}

	if(cdc_bytes_available())
	{
		printf("There is something fishy in the CDC RX buffer...\n");
	}

	uint8_t datagram[6];
	uint32_t datagram_val;
	while(1)
	{
		if((cdc_bytes_available() >= 6) && (cdc_peek() & 0x80))	//Handle write datagram
		{
			cdc_read_bytes(datagram, 6);

			((uint8_t*)&datagram_val)[3] = datagram[2];
			((uint8_t*)&datagram_val)[2] = datagram[3];
			((uint8_t*)&datagram_val)[1] = datagram[4];
			((uint8_t*)&datagram_val)[0] = datagram[5];

			if(datagram[0] & 0x7F)	// Handle access to FW register
			{
				switch(datagram[1])
				{
					case 0xFA:	//Pin Data
						fw_write_pin_data((uint8_t)datagram_val);
						break;
					case 0xF9:	//Pin Set
						fw_pin_set((uint8_t)datagram_val);
						break;
					case 0xF8:	//Pin Clear
						fw_pin_clear((uint8_t)datagram_val);
						break;
					case 0xF7:	//RTMI Control
						fw_write_rtmi_control((uint16_t)datagram_val);
						break;
					case 0xF6:	//RTMI Num Samples
						rtmi_num_samples = datagram_val;
						break;
					case 0xF5:	//RTMI Threshold
						rtmi_threshold = datagram_val;
						break;
					case 0xF4:	//RTMI Channel 0
						rtmi_channels[0] = (uint8_t)datagram_val;
						break;
					case 0xF3:	//RTMI Channel 1
						rtmi_channels[1] = (uint8_t)datagram_val;
						break;
					case 0xF2:	//RTMI Channel 2
						rtmi_channels[2] = (uint8_t)datagram_val;
						break;
					case 0xF1:	//RTMI Channel 3
						rtmi_channels[3] = (uint8_t)datagram_val;
						break;
					case 0xF0:	//RTMI Channel 4
						rtmi_channels[4] = (uint8_t)datagram_val;
						break;
					case 0xEF:	//RTMI Channel 5
						rtmi_channels[5] = (uint8_t)datagram_val;
						break;
					case 0xEE:	//RTMI Channel 6
						rtmi_channels[6] = (uint8_t)datagram_val;
						break;
					case 0xED:	//RTMI Channel 7
						rtmi_channels[7] = (uint8_t)datagram_val;
						break;
					case 0xEC:	//RTMI Period
						timer_set_period(TIM1, (uint16_t)datagram_val);
						break;
					default: ;
				}
			}
			else	//Handle access to device register
			{
				tmc5130_write_reg(datagram[1] | 0x80, datagram_val);
			}
		}

		if((cdc_bytes_available() >= 2) && !(cdc_peek() & 0x80))	//handle read datagram
		{
			cdc_read_bytes(datagram, 2);

			if(datagram[0] & 0x7F)	//Handle access to FW register
			{
				switch(datagram[1])
				{
					case 0xFA:	//Pin Data
						datagram_val = (uint32_t)fw_read_pin_data();
						break;
					case 0xF7:	//RTMI Control
						datagram_val = (uint32_t)rtmi_control;
						break;
					case 0xF6:	//RTMI Num Samples
						datagram_val = rtmi_num_samples;
						break;
					case 0xF5:	//RTMI Threshold
						datagram_val = rtmi_threshold;
						break;
					case 0xF4:	//RTMI Channel 0
						datagram_val = (uint32_t)rtmi_channels[0];
						break;
					case 0xF3:	//RTMI Channel 1
						datagram_val = (uint32_t)rtmi_channels[1];
						break;
					case 0xF2:	//RTMI Channel 2
						datagram_val = (uint32_t)rtmi_channels[2];
						break;
					case 0xF1:	//RTMI Channel 3
						datagram_val = (uint32_t)rtmi_channels[3];
						break;
					case 0xF0:	//RTMI Channel 4
						datagram_val = (uint32_t)rtmi_channels[4];
						break;
					case 0xEF:	//RTMI Channel 5
						datagram_val = (uint32_t)rtmi_channels[5];
						break;
					case 0xEE:	//RTMI Channel 6
						datagram_val = (uint32_t)rtmi_channels[6];
						break;
					case 0xED:	//RTMI Channel 7
						datagram_val = (uint32_t)rtmi_channels[7];
						break;
					case 0xEC:	//RTMI Period
						datagram_val = (uint32_t)timer_get_period(TIM1);
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
			else	//Handle access to device register
			{
				datagram_val = tmc5130_read_reg(datagram[1] & 0x7F);
				datagram[0] = 0x80;
				datagram[1] = ((uint8_t*)&datagram_val)[3];
				datagram[2] = ((uint8_t*)&datagram_val)[2];
				datagram[3] = ((uint8_t*)&datagram_val)[1];
				datagram[4] = ((uint8_t*)&datagram_val)[0];
				cdc_write_bytes(datagram, 5);
			}
		}

		if(prev_control_line_state != cdc_control_line_state)
		{
			cdc_set_serial_state(cdc_control_line_state & 3);
			prev_control_line_state = cdc_control_line_state;
		}
	}
}
