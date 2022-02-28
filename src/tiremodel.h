#ifndef RA_TIREMODEL_H
#define RA_TIREMODEL_H
#include "common.h"

// TODO: Add alignment moment
typedef struct
{
    // *x = longitudinal, *y = lateral *mz = alignment moment

    // Stiffness factor
    float bx, by;
    // Shape
    float cx, cy;
    // Peak
    float dx, dy;
    // Curvature
    float ex, ey;
    // Vertical offset
    float vvx, vvy;
    // Horisontal offset
    float vhx, vhy;

    float peak_slip_x, peak_slip_y;
} TireModel;

float slip_angle(Vector2f velocity, float angle);
float slip_ratio(Vector2f velocity, float angular_velocity, float effective_radius);

Vector2f tiremodel_force(const TireModel* m, float normal_force, float slip_ratio, float slip_angle, float friction_coefficent);

#endif /* RA_TIREMODEL_H */
