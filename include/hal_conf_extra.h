/*
 * hal_conf_extra.h
 *
 * Extra HAL module enables for this project.
 *
 * The stm32duino core pulls this file in via:
 *   stm32f4xx_hal_conf.h  ->  __has_include("hal_conf_extra.h")
 * before it processes stm32yyxx_hal_conf.h (the file that decides which HAL
 * drivers actually get compiled). platformio.ini already passes -Iinclude and
 * -DHAL_CONF_EXTRA_HEADER="hal_conf_extra.h", so defining a module here is the
 * supported way to turn it on.
 */

#pragma once

/* DAC1 (PA4 = OUT1, PA5 = OUT2) — analog control output for the DPLL. */
#define HAL_DAC_MODULE_ENABLED

/* TIM modules are already enabled by default in this core; re-asserted here
 * only for documentation. The phase-capture timers will live on TIM. */
#define HAL_TIM_MODULE_ENABLED
