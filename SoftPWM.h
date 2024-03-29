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
#ifndef SOFTPWM_H
#define SOFTPWM_H

#include <cstdint>

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