#ifndef RA_WHEEL_H
#define RA_WHEEL_H
#include "common.h"
#include "tiremodel.h"

typedef struct {
    Vector2f hub_velocity;
    Vector2f position;
    float min_speed;

    float angle;
    AngularVelocity angular_velocity;
    float inertia;
    float effective_radius;
    float reaction_torque;
} Wheel;

Wheel* wheel_new(float inertia, float radius, Vector2f position, float min_speed);
raTaggedComponent* ra_tag_wheel(Wheel* w);

Vector2f wheel_slip(const Wheel* wheel);
void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inertia, float torque, float dt);

/**
 * wheel_update must be called before this function
 */
Vector2f wheel_force(Wheel* wheel, TireModel* model, float normal_force, float friction_coefficent);

#endif /* RA_WHEEL_H */
