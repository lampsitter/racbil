#ifndef RA_WHEEL_H
#define RA_WHEEL_H
#include "common.h"
#include "tiremodel.h"

typedef struct
{
    Vector2f hub_velocity;
    Vector2f position;

    float angle;
    AngularVelocity angular_velocity;
    float inv_inertia;
    float unloaded_radius;
    float effective_radius;
    float reaction_torque;
    float toe;
} Wheel;

Wheel* wheel_new(float inv_inertia, float radius, float toe, Vector2f position);
void wheel_free(Wheel* wheel);

Vector2f wheel_slip(Wheel* wheel);
void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inv_inertia, float torque, float dt);
Vector2f wheel_force(Wheel* wheel, TireModel* model, float normal_force, float friction_coefficent);

#endif /* RA_WHEEL_H */
