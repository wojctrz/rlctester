#include "includes.h"

#define DAC_SS_LINE GPIOC
#define DAC_SS_PIN GPIO_PIN_0
#define MUX_IN_LINE GPIOC
#define MUX_IN1 GPIO_PIN_1
#define MUX_IN2 GPIO_PIN_2
#define MUX_IN3 GPIO_PIN_3
#define ADC_LINE GPIOA
#define ADC_IN_PIN GPIO_PIN_0

SPI_HandleTypeDef spi;
TIM_HandleTypeDef tim2;
TIM_HandleTypeDef tim4;
uint8_t freq = 10;
uint8_t should = 0;
uint32_t value;

ADC_HandleTypeDef adc;
UART_HandleTypeDef uart;

void send_char(char c)
{
	HAL_UART_Transmit(&uart, (uint8_t*)&c, 1, 1000);
}


void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint32_t miliseconds = HAL_GetTick();
	double currentsin = sin(2 * 3.14 * freq * miliseconds);

	/* convert sin to 0-255 */
	uint8_t dac_value = (uint8_t)((currentsin + 1) * 127);
	/* send value to DAC */
	uint16_t dac_frame = BuildDacFrame(dac_value);
	SpiWrite(dac_frame, &spi, DAC_SS_LINE, DAC_SS_PIN);

	value = HAL_ADC_GetValue(&adc);
	should = 1;
}

void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	TIM_OC_InitTypeDef oc;
	oc.OCMode = TIM_OCMODE_PWM1;
	oc.Pulse = 100;
	oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
	oc.OCFastMode = TIM_OCFAST_ENABLE;
	oc.OCIdleState = TIM_OCIDLESTATE_SET;
	oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&tim4, &oc, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&tim4, TIM_CHANNEL_1);
}

int main(void)
{
	//  current pinout:
	//	PA5 - SPI1_SCK
	//	PA7 - SPI1_MOSI
	//
	//	PA2 - USART2_TX
	//	PA3 - USART2_RX
	//
	//	PB6 - PWM (Timer 4 channel 1)

	SystemClockConfig();


	SystemCoreClock = 64000000; // 64Mhz
	HAL_Init();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	ConfigureTimer(&tim2);
	ConfigureSPI(&spi);
	ConfigurePWM(&tim4);
	//ConfigureGPIO();
	uart = ConfigureUART();
	adc = ConfigureADC(ADC_LINE, ADC_IN_PIN);
	ConfigureMux(MUX_IN_LINE, MUX_IN1, MUX_IN2, MUX_IN3);
	SetMuxSquare(MUX_IN_LINE, MUX_IN1, MUX_IN2, MUX_IN3);

	while (1)
	{

//		{
//			if (__HAL_UART_GET_FLAG(&uart, UART_FLAG_RXNE) == SET)
//			{
//				uint8_t value;
//				HAL_UART_Receive(&uart, &value, 1, 100);
//				freq = value;
//			}
//		}
		if(should)
		{
			should = 0;
			char buff[10];
			itoa(value, buff, 10);
			uint8_t i;
			for(i = 0; i < 4; i++)
			{
				send_char(buff[i]);
			}
			//send_char('\r');
			send_char('\n');
		}
		// printf("Adc = %ld (%.3fV)\r\n", value, value * 3.3f / 4096.0f);
	}
}

