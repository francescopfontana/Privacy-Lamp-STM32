/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    templates/halconf.h
 * @brief   HAL configuration header.
 * @details HAL configuration file, this file allows to enable or disable the
 *          various device drivers from your application. You may also use
 *          this file in order to override the device drivers default settings.
 *
 * @addtogroup HAL_CONF
 * @{
 */

#ifndef HALCONF_H
#define HALCONF_H

#define _CHIBIOS_HAL_CONF_
#define _CHIBIOS_HAL_CONF_VER_8_4_

#include "mcuconf.h"

/**
 * @brief   Enables the PAL subsystem.
 */
#if !defined(HAL_USE_PAL) || defined(__DOXYGEN__)
#define HAL_USE_PAL                         TRUE
#endif

/**
 * @brief   Enables the ADC subsystem.
 */
#if !defined(HAL_USE_ADC) || defined(__DOXYGEN__)
#define HAL_USE_ADC                         FALSE
#endif

/**
 * @brief   Enables the CAN subsystem.
 */
#if !defined(HAL_USE_CAN) || defined(__DOXYGEN__)
#define HAL_USE_CAN                         FALSE
#endif

/**
 * @brief   Enables the cryptographic subsystem.
 */
#if !defined(HAL_USE_CRY) || defined(__DOXYGEN__)
#define HAL_USE_CRY                         FALSE
#endif

/**
 * @brief   Enables the DAC subsystem.
 */
#if !defined(HAL_USE_DAC) || defined(__DOXYGEN__)
#define HAL_USE_DAC                         FALSE
#endif

/**
 * @brief   Enables the EFlash subsystem.
 */
#if !defined(HAL_USE_EFL) || defined(__DOXYGEN__)
#define HAL_USE_EFL                         FALSE
#endif

/**
 * @brief   Enables the GPT subsystem.
 */
#if !defined(HAL_USE_GPT) || defined(__DOXYGEN__)
#define HAL_USE_GPT                         FALSE
#endif

/**
 * @brief   Enables the I2C subsystem.
 */
#if !defined(HAL_USE_I2C) || defined(__DOXYGEN__)
#define HAL_USE_I2C                         FALSE
#endif

/**
 * @brief   Enables the I2S subsystem.
 */
#if !defined(HAL_USE_I2S) || defined(__DOXYGEN__)
#define HAL_USE_I2S                         FALSE
#endif

/**
 * @brief   Enables the ICU subsystem.
 */
#if !defined(HAL_USE_ICU) || defined(__DOXYGEN__)
#define HAL_USE_ICU                         FALSE
#endif

/**
 * @brief   Enables the MAC subsystem.
 */
#if !defined(HAL_USE_MAC) || defined(__DOXYGEN__)
#define HAL_USE_MAC                         FALSE
#endif

/**
 * @brief   Enables the MMC_SPI subsystem.
 */
#if !defined(HAL_USE_MMC_SPI) || defined(__DOXYGEN__)
#define HAL_USE_MMC_SPI                     FALSE
#endif

/**
 * @brief   Enables the PWM subsystem.
 */
#if !defined(HAL_USE_PWM) || defined(__DOXYGEN__)
#define HAL_USE_PWM                         TRUE
#endif

/**
 * @brief   Enables the RTC subsystem.
 */
#if !defined(HAL_USE_RTC) || defined(__DOXYGEN__)
#define HAL_USE_RTC                         FALSE
#endif

/**
 * @brief   Enables the SDC subsystem.
 */
#if !defined(HAL_USE_SDC) || defined(__DOXYGEN__)
#define HAL_USE_SDC                         FALSE
#endif

/**
 * @brief   Enables the SERIAL subsystem.
 */
#if !defined(HAL_USE_SERIAL) || defined(__DOXYGEN__)
#define HAL_USE_SERIAL                      TRUE
#endif

/**
 * @brief   Enables the SERIAL over USB subsystem.
 */
#if !defined(HAL_USE_SERIAL_USB) || defined(__DOXYGEN__)
#define HAL_USE_SERIAL_USB                  FALSE
#endif

/**
 * @brief   Enables the SIO subsystem.
 */
#if !defined(HAL_USE_SIO) || defined(__DOXYGEN__)
#define HAL_USE_SIO                         FALSE
#endif

/**
 * @brief   Enables the SPI subsystem.
 */
#if !defined(HAL_USE_SPI) || defined(__DOXYGEN__)
#define HAL_USE_SPI                         FALSE
#endif

/**
 * @brief   Enables the TRNG subsystem.
 */
#if !defined(HAL_USE_TRNG) || defined(__DOXYGEN__)
#define HAL_USE_TRNG                        FALSE
#endif

/**
 * @brief   Enables the UART subsystem.
 */
#if !defined(HAL_USE_UART) || defined(__DOXYGEN__)
#define HAL_USE_UART                        FALSE
#endif

/**
 * @brief   Enables the USB subsystem.
 */
#if !defined(HAL_USE_USB) || defined(__DOXYGEN__)
#define HAL_USE_USB                         FALSE
#endif

/**
 * @brief   Enables the WDG subsystem.
 */
#if !defined(HAL_USE_WDG) || defined(__DOXYGEN__)
#define HAL_USE_WDG                         FALSE
#endif

/**
 * @brief   Enables the WSPI subsystem.
 */
#if !defined(HAL_USE_WSPI) || defined(__DOXYGEN__)
#define HAL_USE_WSPI                        FALSE
#endif

/*===========================================================================*/
/* PAL driver related settings.                                              */
/*===========================================================================*/
#if !defined(PAL_USE_CALLBACKS) || defined(__DOXYGEN__)
#define PAL_USE_CALLBACKS                   TRUE
#endif

#if !defined(PAL_USE_WAIT) || defined(__DOXYGEN__)
#define PAL_USE_WAIT                        TRUE
#endif

/*===========================================================================*/
/* ADC driver related settings.                                              */
/*===========================================================================*/
#if !defined(ADC_USE_WAIT) || defined(__DOXYGEN__)
#define ADC_USE_WAIT                        TRUE
#endif

#if !defined(ADC_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define ADC_USE_MUTUAL_EXCLUSION            TRUE
#endif

/*===========================================================================*/
/* CAN driver related settings.                                              */
/*===========================================================================*/
#if !defined(CAN_USE_SLEEP_MODE) || defined(__DOXYGEN__)
#define CAN_USE_SLEEP_MODE                  TRUE
#endif

#if !defined(CAN_ENFORCE_USE_CALLBACKS) || defined(__DOXYGEN__)
#define CAN_ENFORCE_USE_CALLBACKS           FALSE
#endif

/*===========================================================================*/
/* CRY driver related settings.                                              */
/*===========================================================================*/
#if !defined(HAL_CRY_USE_FALLBACK) || defined(__DOXYGEN__)
#define HAL_CRY_USE_FALLBACK                FALSE
#endif

#if !defined(HAL_CRY_ENFORCE_FALLBACK) || defined(__DOXYGEN__)
#define HAL_CRY_ENFORCE_FALLBACK            FALSE
#endif

/*===========================================================================*/
/* DAC driver related settings.                                              */
/*===========================================================================*/
#if !defined(DAC_USE_WAIT) || defined(__DOXYGEN__)
#define DAC_USE_WAIT                        TRUE
#endif

#if !defined(DAC_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define DAC_USE_MUTUAL_EXCLUSION            TRUE
#endif

/*===========================================================================*/
/* I2C driver related settings.                                              */
/*===========================================================================*/
#if !defined(I2C_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define I2C_USE_MUTUAL_EXCLUSION            TRUE
#endif

/*===========================================================================*/
/* MAC driver related settings.                                              */
/*===========================================================================*/
#if !defined(MAC_USE_ZERO_COPY) || defined(__DOXYGEN__)
#define MAC_USE_ZERO_COPY                   FALSE
#endif

#if !defined(MAC_USE_EVENTS) || defined(__DOXYGEN__)
#define MAC_USE_EVENTS                      TRUE
#endif

/*===========================================================================*/
/* MMC_SPI driver related settings.                                          */
/*===========================================================================*/
#if !defined(MMC_NICE_WAITING) || defined(__DOXYGEN__)
#define MMC_NICE_WAITING                    TRUE
#endif

/*===========================================================================*/
/* SDC driver related settings.                                              */
/*===========================================================================*/
#if !defined(SDC_INIT_RETRY) || defined(__DOXYGEN__)
#define SDC_INIT_RETRY                      100
#endif

#if !defined(SDC_MMC_SUPPORT) || defined(__DOXYGEN__)
#define SDC_MMC_SUPPORT                     FALSE
#endif

#if !defined(SDC_NICE_WAITING) || defined(__DOXYGEN__)
#define SDC_NICE_WAITING                    TRUE
#endif

#if !defined(SDC_INIT_OCR_V20) || defined(__DOXYGEN__)
#define SDC_INIT_OCR_V20                    0x50FF8000U
#endif

#if !defined(SDC_INIT_OCR) || defined(__DOXYGEN__)
#define SDC_INIT_OCR                        0x80100000U
#endif

/*===========================================================================*/
/* SERIAL driver related settings.                                           */
/*===========================================================================*/
#if !defined(SERIAL_DEFAULT_BITRATE) || defined(__DOXYGEN__)
#define SERIAL_DEFAULT_BITRATE              38400
#endif

#if !defined(SERIAL_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_BUFFERS_SIZE                 16
#endif

/*===========================================================================*/
/* SIO driver related settings.                                              */
/*===========================================================================*/
#if !defined(SIO_DEFAULT_BITRATE) || defined(__DOXYGEN__)
#define SIO_DEFAULT_BITRATE                 38400
#endif

#if !defined(SIO_USE_SYNCHRONIZATION) || defined(__DOXYGEN__)
#define SIO_USE_SYNCHRONIZATION             TRUE
#endif

/*===========================================================================*/
/* SERIAL_USB driver related setting.                                        */
/*===========================================================================*/
#if !defined(SERIAL_USB_BUFFERS_SIZE) || defined(__DOXYGEN__)
#define SERIAL_USB_BUFFERS_SIZE             256
#endif

#if !defined(SERIAL_USB_BUFFERS_NUMBER) || defined(__DOXYGEN__)
#define SERIAL_USB_BUFFERS_NUMBER           2
#endif

/*===========================================================================*/
/* SPI driver related settings.                                              */
/*===========================================================================*/
#if !defined(SPI_USE_WAIT) || defined(__DOXYGEN__)
#define SPI_USE_WAIT                        TRUE
#endif

#if !defined(SPI_USE_ASSERT_ON_ERROR) || defined(__DOXYGEN__)
#define SPI_USE_ASSERT_ON_ERROR             TRUE
#endif

#if !defined(SPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define SPI_USE_MUTUAL_EXCLUSION            TRUE
#endif

#if !defined(SPI_SELECT_MODE) || defined(__DOXYGEN__)
#define SPI_SELECT_MODE                     SPI_SELECT_MODE_PAD
#endif

/*===========================================================================*/
/* UART driver related settings.                                             */
/*===========================================================================*/
#if !defined(UART_USE_WAIT) || defined(__DOXYGEN__)
#define UART_USE_WAIT                       FALSE
#endif

#if !defined(UART_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define UART_USE_MUTUAL_EXCLUSION           FALSE
#endif

/*===========================================================================*/
/* USB driver related settings.                                              */
/*===========================================================================*/
#if !defined(USB_USE_WAIT) || defined(__DOXYGEN__)
#define USB_USE_WAIT                        FALSE
#endif

/*===========================================================================*/
/* WSPI driver related settings.                                             */
/*===========================================================================*/
#if !defined(WSPI_USE_WAIT) || defined(__DOXYGEN__)
#define WSPI_USE_WAIT                       TRUE
#endif

#if !defined(WSPI_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define WSPI_USE_MUTUAL_EXCLUSION           TRUE
#endif

#endif /* HALCONF_H */
