/*
 * Fernvale declarations.
 *
 * Copyright (C) 2013 Antony Pavlov <antonynpavlov@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef HW_ARM_FERNVALE_H
#define HW_ARM_FERNVALE_H

#include "cpu.h"

#define TYPE_FERNVALE "fernvale"

#define FERNVALE(obj) OBJECT_CHECK(FernvaleState, (obj), TYPE_FERNVALE)

typedef struct FernvaleState {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    ARMCPU cpu;

} FernvaleState;

#endif /* HW_ARM_FERNVALE_H */
