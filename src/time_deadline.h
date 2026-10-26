#pragma once
#include <stdint.h>
#include <stdbool.h>

// Deadlines must be no more than INT32_MAX milliseconds in the future.
// Zero is a valid deadline; callers track activation separately.
static inline bool time_deadline_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}
