#include "wheel.h"

Wheel wheel_new(float inv_inertia, float radius, float toe, Vector2f position)
{
    return (Wheel) {
        .inv_inertia = inv_inertia,
        .effective_radius = radius,
        .unloaded_radius = radius,
        .position = position,
        .hub_velocity = vector2f_default(),
        .angle = toe,
        .toe = toe,
        .reaction_torque = 0.0f,
    };
}

void wheel_change_angle(Wheel* wheel, float angle) { wheel->angle = wheel->toe + angle; }

static Vector2f translate_velocity(
    Vector2f velocity_cog, float yaw_angular_velocity_cog, Vector2f position)
{
    float x = velocity_cog.x - yaw_angular_velocity_cog * position.y;
    float y = velocity_cog.y + yaw_angular_velocity_cog * position.x;

    return (Vector2f) { .x = x, .y = y };
}

void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inv_inertia, float torque, float dt)
{
    wheel->hub_velocity
        = translate_velocity(velocity_cog, yaw_angular_velocity_cog, wheel->position);

    float total_torque = torque + wheel->reaction_torque;
    wheel->angular_velocity
        += integrate(total_torque, external_inv_inertia + wheel->inv_inertia, dt);
    if (wheel->angular_velocity < 0.0) {
        // TODO: Allow reverse driving
        wheel->angular_velocity = 0.0f;
    }
}

static float wheel_reaction_torque(const Wheel* wheel, Vector2f force)
{
    return -force.x * wheel->effective_radius;
}

Vector2f wheel_slip(const Wheel* wheel)
{
    float slip_x
        = slip_ratio(wheel->hub_velocity, wheel->angular_velocity, wheel->effective_radius);
    float slip_y = slip_angle(wheel->hub_velocity, wheel->angle);
    return (Vector2f) { .x = slip_x, .y = slip_y };
}

// wheel_update must be called before this function
Vector2f wheel_force(Wheel* wheel, TireModel* model, float normal_force, float friction_coefficent)
{

    Vector2f slip = wheel_slip(wheel);
    Vector2f force = tiremodel_force(model, normal_force, slip.x, slip.y, friction_coefficent);
    wheel->reaction_torque = wheel_reaction_torque(wheel, force);
    return force;
}
