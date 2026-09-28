/*
 * platform.c
 *
 *  Created on: Sep 2, 2026
 *      Author: gug
 */

#include "platform.h"
#include <string.h>

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  uint8_t buf[len + 1];
  buf[0] = reg;
  memcpy(&buf[1], bufp, len);
  ISM330DLC_Platform * ism330dlccfg = (ISM330DLC_Platform *) handle;
  i2cStart(ism330dlccfg->i2cp, ism330dlccfg->i2ccfg);

  return i2cMasterTransmitTimeout(ism330dlccfg->i2cp, ism330dlccfg->address, (const uint8_t *)&buf, len + 1, NULL, 0,
                                    TIME_INFINITE);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  ISM330DLC_Platform * ism330dlccfg = (ISM330DLC_Platform *) handle;

  i2cStart(ism330dlccfg->i2cp, ism330dlccfg->i2ccfg);

  return i2cMasterTransmitTimeout(ism330dlccfg->i2cp, ism330dlccfg->address, &reg, 1, bufp, len,
                                    TIME_INFINITE);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
  chThdSleepMilliseconds(ms);
}
