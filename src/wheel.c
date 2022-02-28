#include "wheel.h"
#include <stdlib.h>

Wheel* wheel_new(float inv_inertia, float radius, Vector2f position)
{
    Wheel* wheel = malloc(sizeof *wheel);
    wheel->inv_inertia = inv_inertia;
    wheel->effective_radius = wheel->unloaded_radius = radius;
    wheel->position = position;
    wheel->hub_velocity = vector2f_default();
    wheel->angle = 0.0f;
    wheel->reaction_torque = 0.0f;
    return wheel;
}

void wheel_free(Wheel* wheel)
{
    free(wheel);
}

static Vector2f translate_velocity(Vector2f velocity_cog, float yaw_angular_velocity_cog,
    Vector2f position)
{
    float x = velocity_cog.x - yaw_angular_velocity_cog * position.y;
    float y = velocity_cog.y + yaw_angular_velocity_cog * position.x;

    return (Vector2f) { .x = x, .y = y };
}

void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inv_inertia, float torque, float dt)
{
    wheel->hub_velocity = translate_velocity(velocity_cog, yaw_angular_velocity_cog, wheel->position);

    float total_torque = torque + wheel->reaction_torque;
    wheel->angular_velocity += integrate(total_torque, external_inv_inertia + wheel->inv_inertia, dt);
}

static inline float wheel_reaction_torque(Wheel* wheel, Vector2f force)
{
    return -(force.x * wheel->effective_radius);
}

// wheel_update must be called before this function
Vector2f wheel_force(Wheel* wheel, TireModel* model, float normal_force, float friction_coefficent)
{
    float slip_x = slip_ratio(wheel->hub_velocity, wheel->angular_velocity, wheel->effective_radius);
    float slip_y = slip_angle(wheel->hub_velocity, wheel->angle);

    Vector2f force = tiremodel_force(model, normal_force, slip_x, slip_y, friction_coefficent);
    wheel->reaction_torque = wheel_reaction_torque(wheel, force);
    return force;
}
