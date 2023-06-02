#include "stm32f4xx.h"

void Configure_GPIO(void);
void Configure_ADC(void);
void Delay_ms(volatile uint32_t time_ms);

uint32_t Adc_value;

int main(void)
{
    Configure_GPIO();
    Configure_ADC();

    while(1)
    {
        ADC1->CR2 |= ADC_CR2_SWSTART;

        while (!(ADC1->SR & ADC_SR_EOC));

        Adc_value = ADC1->DR;

        Delay_ms(200);
    }
}


void Configure_GPIO(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= GPIO_MODER_MODER0;
}
void Configure_ADC(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC1->CR1 = 0;
    ADC1->SQR3 = 0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1;
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;
}

void Delay_ms(volatile uint32_t time_ms)
{
    uint32_t delay = time_ms * (SystemCoreClock / 1000);
    while (delay--)
    {
    }
}
