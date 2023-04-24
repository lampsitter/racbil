#ifndef RA_ASSISTS_H
#define RA_ASSISTS_H
#include <stdbool.h>

/**
 * ABS (anti-lock braking system)
 * If enabled, ABS will kick in when the wheels are below `desired_slip_ratio`,
 * and will be deactivated when slip ratio is above. ABS will also not be
 * applied when below minimum velocity.
 * */
typedef struct {
    bool is_enabled;
    float desired_slip_ratio;
    float min_velocity;
} Abs;

Abs abs_new(float desired_slip_ratio, float min_velocity);
float abs_pressure(const Abs* abs, float pressure, float velocity, float slip_ratio);

#endif /* RA_ASSISTS_H */
