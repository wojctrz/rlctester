#include "includes.h"

void SystemClockConfig(void)
{
 RCC_ClkInitTypeDef clkinitstruct = {0};
 RCC_OscInitTypeDef oscinitstruct = {0};

 /* Configure PLL ------------------------------------------------------*/
 /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
 /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
 /* Enable HSI and activate PLL with HSi_DIV2 as source */
 oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
 oscinitstruct.HSEState        = RCC_HSE_OFF;
 oscinitstruct.LSEState        = RCC_LSE_OFF;
 oscinitstruct.HSIState        = RCC_HSI_ON;
 oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
 oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
 oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
 oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
 oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
 if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
 {
   /* Initialization Error */
   while(1);
 }

 /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
    clocks dividers */
 clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
 clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
 clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
 if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
 {
   /* Initialization Error */
   while(1);
 }
}

void ConfigureTimer(TIM_HandleTypeDef *tim2)
{

	__HAL_RCC_TIM2_CLK_ENABLE();


	tim2->Instance = TIM2;
	tim2->Init.Period = 2000 - 1;
	tim2->Init.Prescaler = 64 - 1;
	tim2->Init.ClockDivision = 0;
	tim2->Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2->Init.RepetitionCounter = 0;
	tim2->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(tim2);

	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_TIM_Base_Start_IT(tim2);

}

void ConfigureSPI(SPI_HandleTypeDef *spi)
{
	__HAL_RCC_SPI1_CLK_ENABLE();

	/* SPI configuration */
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;		// SCK, MOSI
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_6;						// MISO
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_0;						// CS
	HAL_GPIO_Init(GPIOC, &gpio);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	spi->Instance = SPI1;
	spi->Init.Mode = SPI_MODE_MASTER;
	spi->Init.NSS = SPI_NSS_SOFT;
	spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;	// 1MHz
	spi->Init.Direction = SPI_DIRECTION_2LINES;
	spi->Init.CLKPhase = SPI_PHASE_1EDGE;
	spi->Init.CLKPolarity = SPI_POLARITY_LOW;
	spi->Init.DataSize = SPI_DATASIZE_8BIT;
	spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi->Init.TIMode = SPI_TIMODE_DISABLE;
	spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi->Init.CRCPolynomial = 7;
	HAL_SPI_Init(spi);

	__HAL_SPI_ENABLE(spi);
}

void ConfigurePWM(TIM_HandleTypeDef *tim4, uint8_t freq)
{

	__HAL_RCC_TIM4_CLK_ENABLE();
	uint32_t prescaler = 1000 / freq;
	/* timer configuration */
	tim4->Instance = TIM4;
	tim4->Init.Period = prescaler - 1;
	tim4->Init.Prescaler = 64000 - 1;
	tim4->Init.ClockDivision = 0;
	tim4->Init.CounterMode = TIM_COUNTERMODE_UP;
	tim4->Init.RepetitionCounter = 0;
	tim4->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_PWM_Init(tim4);

	/* PWM configuration */
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_6;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio);

	TIM_OC_InitTypeDef oc;
	oc.OCMode = TIM_OCMODE_PWM1;
	oc.Pulse = prescaler / 2;
	oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
	oc.OCFastMode = TIM_OCFAST_ENABLE;
	oc.OCIdleState = TIM_OCIDLESTATE_SET;
	oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(tim4, &oc, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(tim4, TIM_CHANNEL_1);

}

UART_HandleTypeDef ConfigureUART()
{
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_2;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &gpio);

	UART_HandleTypeDef uart;
	uart.Instance = USART2;
	uart.Init.BaudRate = 115200;
	uart.Init.WordLength = UART_WORDLENGTH_8B;
	uart.Init.Parity = UART_PARITY_NONE;
	uart.Init.StopBits = UART_STOPBITS_1;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OverSampling = UART_OVERSAMPLING_16;
	uart.Init.Mode = UART_MODE_TX_RX;
	HAL_UART_Init(&uart);

	return uart;
}

void ConfigureGPIO(void)
{

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();


	/* ext interrupt */
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpio.Pull = GPIO_PULLUP;
	gpio.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &gpio);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


	HAL_NVIC_EnableIRQ(TIM2_IRQn);

}


uint16_t BuildDacFrame(uint8_t value)
{	/* Build frame and send it to DAC via SPI interface.
 	 Frame format - refer to MCP4901 documentation */
	uint16_t data_shifted = 0xF00F | (uint16_t)(value << 4);
	// |       0111       || xxxx xxxx | | 0000			   |
	// |configuration bits|| data bits | | fill to 16 bits |
	uint16_t frame = 0x3FFF & data_shifted & 0xFFF0;
	return frame;
}


void SpiWrite(uint16_t data, SPI_HandleTypeDef *spi, GPIO_TypeDef *GPIO_line, uint16_t ss_pin)
{	/* convert data to format supported by HAL function and send it via SPI interface */
	uint8_t sendbuff[2] = { (uint8_t)(data >> 8), (uint8_t)data };
	HAL_GPIO_WritePin(GPIO_line, ss_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, sendbuff, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIO_line, ss_pin, GPIO_PIN_SET);
}

ADC_HandleTypeDef ConfigureADC(GPIO_TypeDef *GPIO_line, uint16_t adc_pin)
{
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_ANALOG;
	gpio.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIO_line, &gpio);


	RCC_PeriphCLKInitTypeDef adc_clk;
	adc_clk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	adc_clk.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	HAL_RCCEx_PeriphCLKConfig(&adc_clk);

	ADC_HandleTypeDef adc;
	adc.Instance = ADC1;
	adc.Init.ContinuousConvMode = ENABLE;
	adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	adc.Init.NbrOfConversion = 1;
	adc.Init.DiscontinuousConvMode = DISABLE;
	adc.Init.NbrOfDiscConversion = 1;
	HAL_ADC_Init(&adc);

	HAL_ADCEx_Calibration_Start(&adc);

	ADC_ChannelConfTypeDef adc_ch;
	adc_ch.Channel = ADC_CHANNEL_0;
	adc_ch.Rank = ADC_REGULAR_RANK_1;
	adc_ch.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	HAL_ADC_ConfigChannel(&adc, &adc_ch);

	HAL_ADC_Start(&adc);

	return adc;
}

void ConfigureMux(GPIO_TypeDef *GPIO_line, uint16_t pin1, uint16_t pin2, uint16_t pin3)
{
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = pin1|pin2|pin3;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_line, &gpio);

}

void SetMuxSine(GPIO_TypeDef *GPIO_line, uint16_t pin1, uint16_t pin2, uint16_t pin3)
{
	HAL_GPIO_WritePin(GPIO_line, pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_line, pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_line, pin3, GPIO_PIN_RESET);
}


void SetMuxSquare(GPIO_TypeDef *GPIO_line, uint16_t pin1, uint16_t pin2, uint16_t pin3)
{
	HAL_GPIO_WritePin(GPIO_line, pin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_line, pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_line, pin3, GPIO_PIN_RESET);
}
