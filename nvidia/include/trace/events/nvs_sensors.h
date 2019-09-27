/*
 * include/trace/events/nvs_sensors.h
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef _LINUX_TRACE_EVENTS_NVS_SENSORS_H
#define _LINUX_TRACE_EVENTS_NVS_SENSORS_H

#include <trace/events/atrace.h>
#define TRACE_SENSOR_ID			(100)

/*
 * (keep sync with sensor HAL trace.h)
 * COOKIE defintion for sensor HAL ATRACE events
 *
 *  3322222222221111111111000 0 000000 - Bit
 *  1098765432109876543210987 6 543210   Position
 * +-------------------------+-+------+
 * | TIMESTAMP               |P| STYPE|
 * +-------------------------+-+------+
 *
 * Where:
 * TIMESTAMP = Sensor event timestamp
 * P         = Private sensor bit indicator (1=private, 0=not private)
 * STYPE     = Sensor TYPE
 */
#define COOKIE_TOTAL_BITS 32
#define COOKIE_SENSOR_TYPE_BITS 6
#define COOKIE_SENSOR_PRIVATE_BITS 1
#define COOKIE_TIMESTAMP_BITS (COOKIE_TOTAL_BITS - COOKIE_SENSOR_PRIVATE_BITS - COOKIE_SENSOR_TYPE_BITS)

#define COOKIE_SENSOR_TYPE_SHIFT 0
#define COOKIE_SENSOR_PRIVATE_SHIFT COOKIE_SENSOR_TYPE_BITS
#define COOKIE_TIMESTAMP_SHIFT (COOKIE_SENSOR_PRIVATE_BITS + COOKIE_SENSOR_TYPE_BITS)

#define COOKIE_SENSOR_TYPE_MASK (((1 << COOKIE_SENSOR_TYPE_BITS) - 1) << COOKIE_SENSOR_TYPE_SHIFT)
#define COOKIE_SENSOR_PRIVATE_MASK (((1 << COOKIE_SENSOR_PRIVATE_BITS) - 1) << COOKIE_SENSOR_PRIVATE_SHIFT)
#define COOKIE_TIMESTAMP_MASK (((1 << COOKIE_TIMESTAMP_BITS) - 1) << COOKIE_TIMESTAMP_SHIFT)

#define SENSOR_TYPE_PRIVATE_BIT_POS 16
#define TIMESTAMP_SCALE_SHIFT 16	// divide by 65536

#define COOKIE(sensor_type,timestamp) \
    (((sensor_type >> (COOKIE_TOTAL_BITS - SENSOR_TYPE_PRIVATE_BIT_POS - COOKIE_SENSOR_PRIVATE_SHIFT)) & COOKIE_SENSOR_PRIVATE_MASK) | \
    ((sensor_type << COOKIE_SENSOR_TYPE_SHIFT) & COOKIE_SENSOR_TYPE_MASK) | \
    (((timestamp >> TIMESTAMP_SCALE_SHIFT) << COOKIE_TIMESTAMP_SHIFT) & COOKIE_TIMESTAMP_MASK))

#endif /* _LINUX_TRACE_EVENTS_NVS_SENSORS_H */
