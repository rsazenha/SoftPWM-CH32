#ifndef SOFTPWM_H
#define SOFTPWM_H

// DEBUG
#define DEBUG
#define DEBUG_PIN 0      // D0
#define DEBUG_PORT GPIOD // D0
#define DEBUG_GPIONUM 3  // D0

// FastMode
// Uncomment line below enable FastMode, that increases RAM usage significantly (about 1114 bytes!)
//
// 256 pointers of 4 bytes each = 1024 (256 bits of precision PWM)
// 18 cache of 1 byte + 4 bytes pointer = 90 (18 equals MaxChannels)
//

#define FASTMODE

// Define a quantidade máximas de canais
#define SoftPWMMaxChannels 18

// Inicializa o timer, com o valor de disparo e o prescaler
void Timer2_Init(uint16_t Value, uint16_t Prescaler);

// Inicializa o SoftPWM
void SoftPWMInitialize();

// Seta o valor para o PWM do pino especificado (0 - 255)
void SoftPWMWrite(uint8_t Pin, uint8_t Value);

uint8_t gpioForPin(uint8_t pin);

GPIO_TypeDef *gpioRegister(uint8_t gpio);

uint8_t gpioPin(uint8_t gpio, uint8_t pin);

void SetPinOutputAndLow(uint8_t GPIO, GPIO_TypeDef *Port, uint8_t Pin);

// Configura um pino para o SoftPWM
void SoftPWMConfigurePIN(uint8_t Pin);

// Gerencia a interrupção do Timer2
extern "C" void TIM2_IRQHandler(void) __attribute__((interrupt));
extern "C" void TIM2_IRQHandler(void);

#endif