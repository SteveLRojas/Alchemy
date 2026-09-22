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
//* Contains Alchemy firmware for the TMC5130 eval *
//* board. Currently using USBD CDC for the host   *
//* and SPI1 for the TMC5130 target.               *
//**************************************************
#include "ch32v20x.h"
#include "ch32v203_afio.h"
#include "ch32v203_core.h"
#include "ch32v203_gpio.h"
#include "ch32v203_rcc.h"
#include "ch32v203_spi.h"
#include "ch32v203_timer.h"
#include "ch32v203_usbd_cdc.h"
#include "debug.h"

//Pins:
// DRV_ENN = PA8
// UDM = PA11
// UDP = PA12
// SPI1_NCS = PA15
// REFL = PB0
// REFR = PB1
// SPI1_SCK = PB3
// SPI1_MISO = PB4
// SPI1_MOSI = PB5
// DIAG0 = PB9
// DIAG1 = PB11

//FW registers
// 0x7FFF:	Device ID
// 0x7FFE:	Unique ID
// 0x7FFD:	Pin Data
// 0x7FFC:	Pin Set
// 0x7FFB:	Pin Clear
// 0x7FFA:	RTMI Control
// 0x7FF9:	RTMI Num Samples
// 0x7FF8:	RTMI Threshold
// 0x7FF7:	RTMI Channel 0
// 0x7FF6:	RTMI Channel 1
// 0x7FF5:	RTMI Channel 2
// 0x7FF4:	RTMI Channel 3
// 0x7FF3:	RTMI Channel 4
// 0x7FF2:	RTMI Channel 5
// 0x7FF1:	RTMI Channel 6
// 0x7FF0:	RTMI Channel 7
// 0x7FEF:	RTMI Period (in microseconds)

//Pin Data bits
// 0:		DRV_ENN
// 1:		REFL
// 2:		REFR
// 3:		DIAG0
// 4:		DIAG1

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

void tmc5130_write_reg(uint8_t addr, uint32_t data)
{
	gpio_clear_pin(GPIOA, GPIO_PIN_15);

	spi_transfer(SPI1, (uint16_t)addr);

	spi_transfer(SPI1, ((uint8_t*)&data)[3]);
	spi_transfer(SPI1, ((uint8_t*)&data)[2]);
	spi_transfer(SPI1, ((uint8_t*)&data)[1]);
	spi_transfer(SPI1, ((uint8_t*)&data)[0]);

	gpio_set_pin(GPIOA, GPIO_PIN_15);
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
	core_delay_us(1);

	gpio_clear_pin(GPIOA, GPIO_PIN_15);
	spi_transfer(SPI1, (uint16_t)addr);
	((uint8_t*)&read_val)[3] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[2] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[1] = (uint8_t)spi_transfer(SPI1, 0x0000);
	((uint8_t*)&read_val)[0] = (uint8_t)spi_transfer(SPI1, 0x0000);
	gpio_set_pin(GPIOA, GPIO_PIN_15);

	return read_val;
}

void fw_write_pin_data(uint8_t pin_val)
{
	gpio_write_pin(GPIOA, GPIO_PIN_8, pin_val & 0x01);	//DRV_ENN
	pin_val = pin_val >> 1;
	gpio_write_pin(GPIOB, GPIO_PIN_0, pin_val & 0x01);	//REFL
	pin_val = pin_val >> 1;
	gpio_write_pin(GPIOB, GPIO_PIN_1, pin_val & 0x01);	//REFR
}

uint8_t fw_read_pin_data(void)
{
	uint8_t val;

	val = gpio_read_pin(GPIOB, GPIO_PIN_11);	//DIAG1
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_9);	//DIAG0
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_1);	//REFR
	val = val << 1;
	val |= gpio_read_pin(GPIOB, GPIO_PIN_0);	//REFL
	val = val << 1;
	val |= gpio_read_pin(GPIOA, GPIO_PIN_8);	//DRV_ENN

	return val;
}

void fw_pin_set(uint8_t pin_val)
{
	if(pin_val & 0x01)	//DRV_ENN
		gpio_set_pin(GPIOA, GPIO_PIN_8);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)	//REFL
		gpio_set_pin(GPIOB, GPIO_PIN_0);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)	//REFR
		gpio_set_pin(GPIOB, GPIO_PIN_1);
}

void fw_pin_clear(uint8_t pin_val)
{
	if(pin_val & 0x01)	//DRV_ENN
		gpio_clear_pin(GPIOA, GPIO_PIN_8);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)	//REFL
		gpio_clear_pin(GPIOB, GPIO_PIN_0);
	pin_val = pin_val >> 1;
	if(pin_val & 0x01)	//REFL
		gpio_clear_pin(GPIOB, GPIO_PIN_1);
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

int main(void)
{
	rcc_apb2_clk_enable(RCC_AFIOEN | RCC_IOPAEN | RCC_IOPBEN | RCC_IOPCEN | RCC_TIM1EN | RCC_SPI1EN | RCC_USART1EN);
	rcc_apb1_clk_enable(RCC_TIM2EN | RCC_USBEN);
	rcc_ahb_clk_enable(RCC_DMA1EN);

	gpio_set_mode(GPIOA, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_8 | GPIO_PIN_15);	//DRV_ENN, SPI1_NCS
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_AFIO_PP, GPIO_PIN_3 | GPIO_PIN_5);	//SPI1_SCK, SPI1_MOSI
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_OUT_50MHZ | GPIO_MODE_PP_OUT, GPIO_PIN_0 | GPIO_PIN_1);	//REFL, REFR
	gpio_set_mode(GPIOB, GPIO_DIR_SPD_IN | GPIO_MODE_PULL_IN, GPIO_PIN_4 | GPIO_PIN_9 | GPIO_PIN_11);	//SPI1_MISO, DIAG0, DIAG1
	gpio_set_pin(GPIOA, GPIO_PIN_8 | GPIO_PIN_15);	//set DRV_ENN, SPI1_NCS
	gpio_set_pin(GPIOB, GPIO_PIN_4 | GPIO_PIN_9 | GPIO_PIN_11);	//pull-up SPI1_MISO, DIAG0, DIAG1

	core_delay_init();

	cdc_init();
	cdc_set_serial_state(CDC_SS_TXCARRIER | CDC_SS_RXCARRIER);
	uint8_t prev_control_line_state = cdc_control_line_state;
	while(!cdc_config);	//Wait for host to configure the CDC interface
    printf("Alchemy - TMC5130\n");

    printf("SYSCLK: %u\n", rcc_compute_sysclk_freq());
	printf("HCLK: %u\n", rcc_compute_hclk_freq());
	printf("PCLK1: %u\n", rcc_compute_pclk1_freq());
	printf("PCLK1_TIM: %u\n", rcc_compute_pclk1_tim_freq());
	printf("PCLK2: %u\n", rcc_compute_pclk2_freq());
	printf("PCLK2_TIM: %u\n", rcc_compute_pclk2_tim_freq());
	printf("ADCCLK: %u\n", rcc_compute_adcclk());

	spi_init(SPI1, SPI_8_BIT | SPI_CLK_DIV_32 | SPI_MODE_3);	//3 MHz
	afio_pcfr1_remap(AFIO_PCFR1_SPI1_REMAP);

	timer_init(TIM1, 96000000 / 1000000 - 1, 1000 - 1);	//1us time scale, 1ms period
	timer1_callback = on_rtmi_int;
	timer_enable_interrupt(TIM1);
	core_enable_irq(TIM1_UP_IRQn);
	timer_start(TIM1);

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
					case 0xFD:	//Pin Data
						fw_write_pin_data((uint8_t)datagram_val);
						break;
					case 0xFC:	//Pin Set
						fw_pin_set((uint8_t)datagram_val);
						break;
					case 0xFB:	//Pin Clear
						fw_pin_clear((uint8_t)datagram_val);
						break;
					case 0xFA:	//RTMI Control
						fw_write_rtmi_control((uint16_t)datagram_val);
						break;
					case 0xF9:	//RTMI Num Samples
						rtmi_num_samples = datagram_val;
						break;
					case 0xF8:	//RTMI Threshold
						rtmi_threshold = datagram_val;
						break;
					case 0xF7:	//RTMI Channel 0
						rtmi_channels[0] = (uint8_t)datagram_val;
						break;
					case 0xF6:	//RTMI Channel 1
						rtmi_channels[1] = (uint8_t)datagram_val;
						break;
					case 0xF5:	//RTMI Channel 2
						rtmi_channels[2] = (uint8_t)datagram_val;
						break;
					case 0xF4:	//RTMI Channel 3
						rtmi_channels[3] = (uint8_t)datagram_val;
						break;
					case 0xF3:	//RTMI Channel 4
						rtmi_channels[4] = (uint8_t)datagram_val;
						break;
					case 0xF2:	//RTMI Channel 5
						rtmi_channels[5] = (uint8_t)datagram_val;
						break;
					case 0xF1:	//RTMI Channel 6
						rtmi_channels[6] = (uint8_t)datagram_val;
						break;
					case 0xF0:	//RTMI Channel 7
						rtmi_channels[7] = (uint8_t)datagram_val;
						break;
					case 0xEF:	//RTMI Period
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
					case 0xFF: //device_id
						datagram_val = 0x30333135;
						break;
					case 0xFE:	//unique_id
						datagram_val = *(volatile uint32_t*)0x1FFFF7E8;
						break;
					case 0xFD:	//Pin Data
						datagram_val = (uint32_t)fw_read_pin_data();
						break;
					case 0xFA:	//RTMI Control
						datagram_val = (uint32_t)rtmi_control;
						break;
					case 0xF9:	//RTMI Num Samples
						datagram_val = rtmi_num_samples;
						break;
					case 0xF8:	//RTMI Threshold
						datagram_val = rtmi_threshold;
						break;
					case 0xF7:	//RTMI Channel 0
						datagram_val = (uint32_t)rtmi_channels[0];
						break;
					case 0xF6:	//RTMI Channel 1
						datagram_val = (uint32_t)rtmi_channels[1];
						break;
					case 0xF5:	//RTMI Channel 2
						datagram_val = (uint32_t)rtmi_channels[2];
						break;
					case 0xF4:	//RTMI Channel 3
						datagram_val = (uint32_t)rtmi_channels[3];
						break;
					case 0xF3:	//RTMI Channel 4
						datagram_val = (uint32_t)rtmi_channels[4];
						break;
					case 0xF2:	//RTMI Channel 5
						datagram_val = (uint32_t)rtmi_channels[5];
						break;
					case 0xF1:	//RTMI Channel 6
						datagram_val = (uint32_t)rtmi_channels[6];
						break;
					case 0xF0:	//RTMI Channel 7
						datagram_val = (uint32_t)rtmi_channels[7];
						break;
					case 0xEF:	//RTMI Period
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
