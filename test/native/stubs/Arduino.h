#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>

uint32_t millis();
inline void noInterrupts() {}
inline void interrupts() {}
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}

struct TIM_TypeDef { uint32_t CCMR1 = 0; };
struct GPIO_TypeDef { uint32_t PUPDR = 0; };
inline TIM_TypeDef testTim2;
inline GPIO_TypeDef testGpioA;
#define TIM2 (&testTim2)
#define GPIOA (&testGpioA)
#define TIM_CCMR1_IC1F_Pos 4
#define TIM_CCMR1_IC2F_Pos 12
constexpr uint32_t PA0 = 0, PA1 = 1;
inline int PinMap_TIM = 0;
inline uint32_t digitalPinToPinName(uint32_t pin) { return pin; }
inline uintptr_t pinmap_peripheral(uint32_t, const int&) { return reinterpret_cast<uintptr_t>(TIM2); }
