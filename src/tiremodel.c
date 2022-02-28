#include "tiremodel.h"
#include <math.h>

static inline float thread_velocity(float angular_velocity, float effective_radius)
{
    return angular_velocity * effective_radius;
}


float slip_angle(Vector2f velocity, float angle)
{
    float slip = atan(velocity.y / velocity.x) - angle;

     if (isnan(slip) || isinf(slip)) {
            return 0.0f;
        } else {
            return slip;
        }
}

float slip_ratio(Vector2f velocity, float angular_velocity, float effective_radius)
{
    float thread_vel = thread_velocity(angular_velocity, effective_radius);
    float slip_velocity = velocity.x - thread_vel;

    float slip = slip_velocity / (fmaxf(fabsf(velocity.x), fabsf(thread_vel)));

    if (isnan(slip) || isinf(slip)) {
        return 0.0f;
    } else {
        return slip;
    }
}

static float pacejka(float b, float c, float d, float e, float vh, float vv, float slip)
{
    float b1 = b * (slip + vh);
    return d * sin(c * atan(b1 - e * (b1 - atan(b1)))) + vv;
}

Vector2f tiremodel_force(const TireModel* m, float normal_force, float slip_ratio, float slip_angle, float friction_coefficent)
{
    float dx_inf = m->dx * normal_force * friction_coefficent;
    float dy_inf = m->dy * normal_force * friction_coefficent;

    float fx, fy;
    if (fabsf(slip_ratio) < EPSILON && fabsf(slip_angle) < EPSILON) {
        // Pure slip scenario
        fx = pacejka(m->bx, m->cx, dx_inf, m->ex, m->vhx, m->vvx, slip_ratio);
        fy = pacejka(m->by, m->cy, dy_inf, m->ey, m->vhy, m->vvy, slip_angle);
    } else {
        // Combined slip
        float norm_slip_x = slip_ratio / m->peak_slip_x;
        float norm_slip_y = slip_angle / m->peak_slip_y;

        float norm_slip = sqrtf(norm_slip_x * norm_slip_x + norm_slip_y * norm_slip_y);
        float fx0 = pacejka(m->bx, m->cx, dx_inf, m->ex, m->vhx, m->vvx, norm_slip);
        float fy0 = pacejka(m->by, m->cy, dy_inf, m->ey, m->vhy, m->vvy, norm_slip);
        fx = (norm_slip_x / norm_slip) * fx0;
        fy = (norm_slip_x / norm_slip) * fy0;
    }

    return (Vector2f) { .x = fx, .y = fy };
}
