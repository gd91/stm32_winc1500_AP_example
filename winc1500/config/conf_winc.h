
/**
 *
 * \file
 *
 * \brief WINC1500 configuration.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "conf_winc.h"
/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/
/* Add extenal definition for spi handler variable, to communicate with winc1500 SPI */
extern SPI_HandleTypeDef hspiWifi;

#define CONF_WINC_PIN_RESET				    GPIO_PIN_7  /* Port C */
#define CONF_WINC_PIN_CHIP_ENABLE		    GPIO_PIN_15 /* Port A */
#define CONF_WINC_PIN_WAKE				    GPIO_PIN_13  /* Port B */
#define CONF_WINC_PIN_POWER_ENABLE          GPIO_PIN_10 /* Port A */
#define CONF_WINC_PIN_LEVEL_SHIFTER_ENABLE  GPIO_PIN_6  /* Port B */
#define CONF_WINC_PORT_LEVEL_SHIFTER_ENABLE GPIOB       /* Port B */


/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				    (1)

/** SPI pin and instance settings. */
    /* User can use this section to tailor SPIx instance used and associated
       resources */
    /* Definition for SPI_WINC1500 == SPI3, clock resources */
#   define SPI_WIFI                             SPI3
#   define SPI_WIFI_CLK_ENABLE()                __HAL_RCC_SPI3_CLK_ENABLE()
#   define SPI_WIFI_CS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()
#   define SPI_WIFI_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#   define SPI_WIFI_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#   define SPI_WIFI_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#   define SPI_WIFI_FORCE_RESET()               __HAL_RCC_SPI3_FORCE_RESET()
#   define SPI_WIFI_RELEASE_RESET()             __HAL_RCC_SPI3_RELEASE_RESET()

/* Definition for SPI3 Pins */
#   define SPI_WIFI_CS_PIN                      GPIO_PIN_4 
#   define SPI_WIFI_CS_GPIO_PORT                GPIOA
#   define SPI_WIFI_SCK_PIN                     GPIO_PIN_3
#   define SPI_WIFI_SCK_GPIO_PORT               GPIOB
#   define SPI_WIFI_MISO_PIN                    GPIO_PIN_4
#   define SPI_WIFI_MISO_GPIO_PORT              GPIOB
#   define SPI_WIFI_MOSI_PIN                    GPIO_PIN_5
#   define SPI_WIFI_MOSI_GPIO_PORT              GPIOB
#   define SPI3_WIFI_AF                         GPIO_AF6_SPI3

/** WiFi interrupt pin. */
/* Add WiFi Interrupt pin: ST interrupt pin definition */
#   define CONF_WINC_SPI_INT_PIN                GPIO_PIN_12      /* Port B */


#   define CONF_WINC_EXTI_IRQN                  EXTI15_10_IRQn

/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				        (12000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/

/* Debug variable, defined here instead of IAR project options */
#define CONF_WINC_DEBUG				            1
#ifdef CONF_WINC_DEBUG
#   define CONF_WINC_PRINTF                     printf //HAL_OutputMessage
#else
#   define CONF_WINC_PRINTF						printf
#endif

#endif /* CONF_WINC_H_INCLUDED */
