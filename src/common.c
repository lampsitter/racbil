#include "common.h"
#include <math.h>

Vector2f vector2f_default(void)
{
    return (Vector2f) { .x = 0.0f, .y = 0.0f };
}

float vector2f_length(Vector2f v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}

Vector2f vector2f_rotate(Vector2f v, float angle)
{
    float a_sin = sin(angle);
    float a_cos = cos(angle);
    float x = v.x * a_cos - v.y * a_sin;
    float y = v.x * a_sin + v.y * a_cos;

    return (Vector2f) { .x = x, .y = y };
}

Vector3f vector3f_default(void)
{
    return (Vector3f) { .x = 0.0f, .y = 0.0f, .z = 0.0f };
}

float vector3f_length(Vector3f v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float integrate(float torque, float inv_inertia, float dt)
{
    return torque * inv_inertia * dt;
}

float rad_to_deg(float radians)
{
    return radians * (180.0 / M_PI);
}

float deg_to_rad(float degrees)
{
    return (degrees * M_PI) / 180.0;
}

float signum(float v)
{
    return (copysignf(1.0, v));
}
