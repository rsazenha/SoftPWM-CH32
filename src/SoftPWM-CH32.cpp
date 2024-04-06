#include <ch32v003fun.h>
#include "SoftPWM-CH32.h"

#ifdef FASTMODE

struct PinChain
{
    uint8_t PinIndex;
    PinChain *NextPin;
};

// List of pointers for PinChain (Position in chain is the pin value)
PinChain *PinList[256];

// Cache of PinChain to avoid use malloc
PinChain PinChainCache[SoftPWMMaxChannels];

#endif

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
#ifdef DEBUG
    SetPinOutputAndLow(DEBUG_GPIONUM, DEBUG_PORT, DEBUG_PIN);
#endif

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

#ifdef FASTMODE

    // Clean all list
    for (int i = 0; i < 256; i++)
        PinList[i] = 0;

    // Rebuild list
    for (uint32_t i = 0; i < NextEmptyChannel; i++)
    {
        if (SoftPWM_Value[i] > 0)
        {
            PinChainCache[i].PinIndex = i;
            PinChainCache[i].NextPin = 0;

            if (PinList[SoftPWM_Value[i]] == 0)
            {
                PinList[SoftPWM_Value[i]] = &PinChainCache[i];
            }
            else
            {
                // Search next empty PinPointer
                PinChain *NextPinPointer = PinList[SoftPWM_Value[i]];

                while (NextPinPointer->NextPin != 0)
                    NextPinPointer = NextPinPointer->NextPin;

                NextPinPointer->NextPin = &PinChainCache[i];
            }
        }
    }

#endif
}

uint8_t gpioForPin(uint8_t pin)
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

GPIO_TypeDef *gpioRegister(uint8_t gpio)
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

uint8_t gpioPin(uint8_t gpio, uint8_t pin)
{
    if (gpio == 0)
    {
        return pin + 1;
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

extern "C" void TIM2_IRQHandler(void) __attribute__((interrupt));
extern "C" void TIM2_IRQHandler(void)
{
    // Reset the timer
    TIM2->INTFR = (uint16_t)~TIM_IT_Update;
    TIM2->CNT = 0;

    SoftPWM_Count++;

#ifdef DEBUG
    DEBUG_PORT->BSHR = ((uint32_t)1 << DEBUG_PIN); // For DEBUG
#endif

    // ---
    //
    // Benchmark
    //
    // Total period between irq: 39.4us * 256 = 10086.4 (total)

#ifdef FASTMODE

    // FastMode method
    // 1 ch  = 179.8us (total) 1.8%
    // 4 ch  = 181.5us (total) 1.8%
    // 18 ch = 187.3us (total) 1.9%
    if (SoftPWM_Count == 0)
    {
        // Turn on all pins, except pins with value = 0
        for (uint32_t i = 0; i < NextEmptyChannel; i++)
            if (SoftPWM_Value[i] > 0)
                SoftPWM_PINFastPort[i]->BSHR = ((uint32_t)1 << SoftPWM_PINFast[i]);
            else
                // Turn OFF because his value is zero
                SoftPWM_PINFastPort[i]->BSHR = ((uint32_t)1 << (SoftPWM_PINFast[i] + 16));
    }
    else
    {
        PinChain *NextPinPointer = PinList[SoftPWM_Count - 1];

        while (NextPinPointer != 0)
        {
            PinChain CurrentPin = *NextPinPointer;

            // Turn OFF Pin by Index
            SoftPWM_PINFastPort[CurrentPin.PinIndex]->BSHR = ((uint32_t)1 << (SoftPWM_PINFast[CurrentPin.PinIndex] + 16));

            NextPinPointer = CurrentPin.NextPin;
        }
    }

#else

    // Traditional method
    // 1 ch  =  1.5us  3.8% of CPU use
    // 4 ch  =  3.2us  8.1% of CPU use
    // 18 ch = 11.4us 28.9% of CPU use
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

#endif

    // ---

#ifdef DEBUG
    DEBUG_PORT->BSHR = ((uint32_t)1 << (DEBUG_PIN + 16)); // For DEBUG
#endif
}
