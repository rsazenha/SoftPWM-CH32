/*
|| @author         Raul Azenha <rsazenha@gmail.com>
|| @contribution
|| @url            http://azenha.org/
||
|| @description
|| | A Software PWM Library for CH32V003 with Lib "ch32v003fun"
||
|| @name Software PWM Library
|| @type Library
|| @target CH32V003
||
|| @version 0.0.1
||
*/

#include <ch32v003fun.h>
#include "SoftPWM.h"

// Mantém o incremento da interrupção (8 bits)
uint8_t SoftPWM_Count;

// Armazena o valor (setpoint) do canal

uint8_t SoftPWM_Value[SoftPWMMaxChannels];
// Armazena o número do pino do canal

uint16_t SoftPWM_PINFast[SoftPWMMaxChannels];
// Armazena a porta do pino do canal
GPIO_TypeDef *SoftPWM_PINFastPort[SoftPWMMaxChannels];

// Armazena o número do pino do canal (ID único)
uint8_t SoftPWM_PIN[SoftPWMMaxChannels];

// Próximo canal livre para ser utilizado
uint8_t NextEmptyChannel = 0;

void Timer2_Init(uint16_t Value, uint16_t Prescaler)
{
    // Enable TIM2 (peripheral clock)
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2; // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    uint16_t tmpcr1 = TIM2->CTLR1;

    tmpcr1 &= (uint16_t)(~((uint16_t)(TIM_DIR | TIM_CMS)));
    tmpcr1 |= (uint32_t)TIM_CounterMode_Up;

    tmpcr1 &= (uint16_t)(~((uint16_t)TIM_CTLR1_CKD));
    tmpcr1 |= (uint32_t)TIM_CKD_DIV1;

    TIM2->CTLR1 = tmpcr1;
    TIM2->ATRLR = Value;
    TIM2->PSC = Prescaler;

    TIM2->SWEVGR = TIM_PSCReloadMode_Immediate;

    // initialize counter
    TIM2->DMAINTENR |= TIM_IT_Update; // TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_EnableIRQ(TIM2_IRQn);

    // Reset TIM2 to init all regs
    // RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    // RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    // Enable TIM2
    TIM2->CTLR1 |= TIM_CEN; // TIM_Cmd(TIM2, ENABLE);

    // Reset timer
    TIM2->CNT = 0;
}

void SoftPWMInitialize()
{
    SoftPWM_Count = 0;

    NextEmptyChannel = 0;

    for (int i = 0; i < SoftPWMMaxChannels; i++)
    {
        SoftPWM_Value[i] = 0;
        SoftPWM_PIN[i] = 0;
        SoftPWM_PINFast[i] = 0;
        // SoftPWM_PINFastPort[i] = 0;
    }

    // Timer2_Init(10 - 1, 48000 - 1); // 100hz
    Timer2_Init(1852 - 1, 1 - 1); // 100hz * 255 ? 1882
    // Timer2_Init(3744 - 1, 1 - 1); // 50hz * 255 ? 3762
};

void SoftPWMWrite(uint8_t Pin, uint8_t Value)
{
    for (int i = 0; i < SoftPWMMaxChannels; i++)
    {
        if (SoftPWM_PIN[i] == Pin)
            SoftPWM_Value[i] = Value;
    }
}

static uint8_t gpioForPin(uint8_t pin)
{
    if (pin < 2)
    {
        return 0;
    }
    else if (pin < 10)
    {
        return 2;
    }
    else
    {
        return 3;
    }
}

static GPIO_TypeDef *gpioRegister(uint8_t gpio)
{
    if (gpio == 0)
    {
        return GPIOA;
    }
    else if (gpio == 2)
    {
        return GPIOC;
    }
    else
    {
        return GPIOD;
    }
}

static uint8_t gpioPin(uint8_t gpio, uint8_t pin)
{
    if (gpio == 0)
    {
        return pin;
    }
    else if (gpio == 2)
    {
        return pin - 2;
    }
    else
    {
        return pin - 10;
    }
}

void SetPinOutputAndLow(uint8_t GPIO, GPIO_TypeDef *Port, uint8_t Pin)
{
    // Enable GPIO
    RCC->APB2PCENR |= (0x04 << GPIO);

    // Configure pin
    uint8_t pinConfig = GPIO_Speed_50MHz | GPIO_CNF_OUT_PP;

    Port->CFGLR &= ~(0xf << (Pin * 4));
    Port->CFGLR |= (pinConfig << (Pin * 4));

    // Set pin state LOW
    Port->BCR = ((uint32_t)1 << Pin);
}

void SoftPWMConfigurePIN(uint8_t Pin)
{
    // Só cria o SoftPWM se tem canal livre
    if (NextEmptyChannel < SoftPWMMaxChannels)
    {
        SoftPWM_PIN[NextEmptyChannel] = Pin;
        SoftPWM_Value[NextEmptyChannel] = 0;

        uint8_t GPIO = gpioForPin(Pin);

        SoftPWM_PINFast[NextEmptyChannel] = gpioPin(GPIO, Pin);
        SoftPWM_PINFastPort[NextEmptyChannel] = gpioRegister(GPIO);

        SetPinOutputAndLow(GPIO, SoftPWM_PINFastPort[NextEmptyChannel], SoftPWM_PINFast[NextEmptyChannel]);

        NextEmptyChannel++;
    }
}

volatile uint8_t Value;

// void TIM2_IRQHandler(void) __attribute__((interrupt));
// void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void TIM2_IRQHandler(void) __attribute__((interrupt));
extern "C" void TIM2_IRQHandler(void)
{
    // Reset the timer
    TIM2->INTFR = (uint16_t)~TIM_IT_Update;
    TIM2->CNT = 0;

    // digitalWrite(D0, HIGH);
    GPIOD->BSHR = ((uint32_t)1 << 0); // For DEBUG

    // ---

    SoftPWM_Count++;

    // 6% ~~ 26% 3,2us ~ 11,4us (39,4)
    if (SoftPWM_Count == 0)
    {
        for (uint32_t i = 0; i < NextEmptyChannel; i++)
            if (SoftPWM_Value[i] > 0)
                SoftPWM_PINFastPort[i]->BSHR = ((uint32_t)1 << SoftPWM_PINFast[i]);
            else
                SoftPWM_PINFastPort[i]->BSHR = ((uint32_t)1 << (SoftPWM_PINFast[i] + 16));
    }
    else
    {
        for (uint32_t i = 0; i < NextEmptyChannel; i++)
            if (SoftPWM_Value[i] < SoftPWM_Count)
                SoftPWM_PINFastPort[i]->BSHR = ((uint32_t)1 << (SoftPWM_PINFast[i] + 16));
    }

    // ---
    // digitalWrite(D0, LOW);
    GPIOD->BSHR = ((uint32_t)1 << 0 + 16); // For DEBUG
}
