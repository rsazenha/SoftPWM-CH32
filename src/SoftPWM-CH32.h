#ifndef SOFTPWM_H
#define SOFTPWM_H

// Define a quantidade máximas de canais
#define SoftPWMMaxChannels 18

// Inicializa o timer, com o valor de disparo e o prescaler
void Timer2_Init(uint16_t Value, uint16_t Prescaler);

// Inicializa o SoftPWM
void SoftPWMInitialize();

// Seta o valor para o PWM do pino especificado (0 - 255)
void SoftPWMWrite(uint8_t Pin, uint8_t Value);

// Configura um pino para o SoftPWM
void SoftPWMConfigurePIN(uint8_t Pin);

// Gerencia a interrupção do Timer2
extern "C" void TIM2_IRQHandler(void) __attribute__((interrupt));
extern "C" void TIM2_IRQHandler(void);

#endif