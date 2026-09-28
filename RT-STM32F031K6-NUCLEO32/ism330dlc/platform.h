/*
 * platform.h
 *
 *  Created on: Sep 2, 2026
 *      Author: gug
 */

#ifndef ISM330DLC_PLATFORM_H_
#define ISM330DLC_PLATFORM_H_

#include "ch.h"
#include "hal.h"

typedef struct {
  /*
   * @brief I2C driver associated to this LSM6DSO.
   */
  I2CDriver *i2cp;
  /*
   * @brief I2C configuration associated to this LSM6DSO accelerometer
   *        subsystem.
   */
  const I2CConfig *i2ccfg;

  uint8_t address;

} ISM330DLC_Platform;

/* WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
void platform_delay(uint32_t ms);
void platform_init(void);

#endif /* ISM330DLC_PLATFORM_H_ */
