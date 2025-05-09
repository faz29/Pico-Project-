/* i2c.h
   Copyright (C) 2024 Timo Kokkonen <tjko@iki.fi>

   SPDX-License-Identifier: GPL-3.0-or-later

   This file is part of Pico-Sensor-Lib.

   Pico-Sensor-Lib is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Pico-Sensor-Lib is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Pico-Sensor-Lib. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef PICO_SENSOR_LIB_I2C_H
#define PICO_SENSOR_LIB_I2C_H 1

#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef I2C_DEBUG
#define I2C_DEBUG 0
#endif

#if I2C_DEBUG > 0
#define DEBUG_PRINT(fmt, ...)						\
	do {								\
		uint64_t ttt = to_us_since_boot(get_absolute_time());	\
		printf("[%6llu.%06llu] %s:%d: %s(): " fmt,		\
			(ttt / 1000000), (ttt % 1000000),		\
			__FILE__, __LINE__, __func__,			\
			##__VA_ARGS__);					\
	} while (0)
#else
#define DEBUG_PRINT(...)
#endif




typedef void* (i2c_init_func_t)(i2c_inst_t *i2c, uint8_t addr);
typedef int (i2c_start_measurement_func_t)(void *ctx);
typedef int (i2c_get_measurement_func_t)(void *ctx, float *temp, float *pressure, float *humidity);
typedef int (i2c_shutdown_func_t)(void *ctx);

typedef struct i2c_sensor_entry {
	const char* name;
	i2c_init_func_t *init;
	i2c_start_measurement_func_t *start_measurement;
	i2c_get_measurement_func_t *get_measurement;
	i2c_shutdown_func_t *shutdown;
	bool no_scan;
	uint8_t cycle_len;
} i2c_sensor_entry_t;


#define I2C_SENSOR_CONTEXT_MEMBERS \
	uint16_t sensor_type;	   \
	i2c_inst_t *i2c;	   \
	uint8_t addr;

typedef struct i2c_sensor_context {
	I2C_SENSOR_CONTEXT_MEMBERS
} i2c_sensor_context_t;



/* Helper functions for reading/writing sensor registers */

/* i2c.c */
int i2c_read_register_block(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len,
	uint32_t read_delay_us);
int i2c_read_register_u24(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint32_t *val);
int i2c_read_register_u16(i2c_inst_t *i2c, uint8_t i2c_addr, uint8_t reg, uint16_t *val);
int i2c_read_register_u8(i2c_inst_t *i2c, uint8_t i2c_addr, uint8_t reg, uint8_t *val);
int i2c_write_register_block(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len);
int i2c_write_register_u16(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint16_t val);
int i2c_write_register_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t val);

int i2c_read_raw(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf, size_t len, bool nostop);
int i2c_write_raw_u16(i2c_inst_t *i2c, uint8_t addr, uint16_t cmd, bool nostop);
int i2c_write_raw_u8(i2c_inst_t *i2c, uint8_t addr, uint8_t cmd, bool nostop);

int32_t twos_complement(uint32_t value, uint8_t bits);



#ifdef __cplusplus
}
#endif

#endif /* PICO_SENSOR_LIB_I2C_H */
