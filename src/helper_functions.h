#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_
#include "stm32f1xx.h"

void SystemClockConfig(void);
void ConfigureTimer(TIM_HandleTypeDef *tim2);
void ConfigureSPI(SPI_HandleTypeDef *spi);
void ConfigurePWM(TIM_HandleTypeDef *tim4);
UART_HandleTypeDef ConfigureUART();
void ConfigureGPIO(void);
uint16_t BuildDacFrame(uint8_t value);
void SpiWrite(uint16_t data, SPI_HandleTypeDef *spi, GPIO_TypeDef *GPIO_line, uint16_t ss_pin);
ADC_HandleTypeDef ConfigureADC(GPIO_TypeDef *GPIO_line, uint16_t adc_pin);
void ConfigureMux(GPIO_TypeDef *GPIO_line, uint16_t pin1, uint16_t pin2, uint16_t pin3);
void SetMuxSine(GPIO_TypeDef *GPIO_line, uint16_t pin1, uint16_t pin2, uint16_t pin3);
void SetMuxSquare(GPIO_TypeDef *GPIO_line, uint16_t pin1, uint16_t pin2, uint16_t pin3);

#endif // HELPER_FUNCTIONS_H_
