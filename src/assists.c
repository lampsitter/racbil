#include "assists.h"
#include <math.h>

Abs abs_new(float desired_slip_ratio, float min_velocity)
{
    return (Abs) {
        .is_enabled = true, .desired_slip_ratio = desired_slip_ratio, .min_velocity = min_velocity
    };
}

float abs_pressure(const Abs* abs, float pressure, float velocity, float slip_ratio)
{
    if (abs->is_enabled && slip_ratio < abs->desired_slip_ratio
        && fabsf(velocity) >= abs->min_velocity) {
        return 0.0;
    } else {
        return pressure;
    }
}
