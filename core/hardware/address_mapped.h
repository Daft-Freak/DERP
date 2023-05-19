#pragma once

// pico-sdk hardware_base stub

#include <stdint.h>

#define _u(x) x##u

#define _REG_(x)

// not volatile or const
typedef uint32_t io_rw_32;
typedef uint32_t io_ro_32;
typedef uint32_t io_wo_32;
typedef uint16_t io_rw_16;
typedef uint16_t io_ro_16;
typedef uint16_t io_wo_16;
typedef uint8_t io_rw_8;
typedef uint8_t io_ro_8;
typedef uint8_t io_wo_8;