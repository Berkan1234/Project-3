
#include "stm32f4xx.h"

void Configure_ADC(void);
void Configure_UART(void);
void Delay_ms(volatile uint32_t time_ms);
void UART_SendByte(uint8_t byte);

uint32_t Adc_value;

int main(void)
{
    Configure_ADC();
    Configure_UART();

    while (1)
    {
        ADC1->CR2 |= ADC_CR2_SWSTART;

        while (!(ADC1->SR & ADC_SR_EOC));

        Adc_value = ADC1->DR;

        UART_SendByte(Adc_value & 0xFF);
        UART_SendByte((Adc_value >> 8) & 0xFF);
        UART_SendByte((Adc_value >> 16) & 0xFF);
        UART_SendByte((Adc_value >> 24) & 0xFF);

        Delay_ms(200);
    }
}

void Configure_ADC(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= GPIO_MODER_MODER0;

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1;
}

void Configure_UART(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
    GPIOA->AFR[1] |= (7 << ((9 - 8) * 4)) | (7 << ((10 - 8) * 4));

    USART1->BRR = SystemCoreClock / 115200;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void Delay_ms(volatile uint32_t time_ms)
{
    volatile uint32_t delay = time_ms * (SystemCoreClock / 10000);
    while (delay--)
    {
    }
}

void UART_SendByte(uint8_t byte)
{
    while (!(USART1->SR & USART_SR_TXE))
    {
    }
    USART1->DR = byte;
}
