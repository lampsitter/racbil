#include "wheel.h"
#include "common.h"
#include <stdlib.h>

Wheel* wheel_new(float inertia, float radius, Vector2f position, float min_speed)
{
    Wheel* w = malloc(sizeof *w);
    w->inertia = inertia;
    w->effective_radius = radius;
    w->position = position;
    w->min_speed = min_speed;
    w->angular_velocity = min_speed / radius;
    w->hub_velocity = (Vector2f) { .x = min_speed, .y = 0.0 };
    w->angle = 0.0;
    w->reaction_torque = 0.0;
    return w;
}

static Vector2f translate_velocity(
    Vector2f velocity_cog, float yaw_angular_velocity_cog, Vector2f position)
{
    float x = velocity_cog.x - yaw_angular_velocity_cog * position.y;
    float y = velocity_cog.y + yaw_angular_velocity_cog * position.x;

    return (Vector2f) { .x = x, .y = y };
}

static void set_hub_speed(Wheel* wheel, Vector2f new_velocity)
{
    if (signum(new_velocity.x) != signum(wheel->hub_velocity.x)) {
        // This prevents driving in reverse, so the hub velocity must be flipped
        // manually when the car is set into reverse
        wheel->hub_velocity.x = signum(wheel->hub_velocity.x) * wheel->min_speed;
    } else if (fabs(new_velocity.x) < wheel->min_speed) {
        wheel->hub_velocity.x = signum(wheel->hub_velocity.x) * wheel->min_speed;
    } else {
        wheel->hub_velocity.x = new_velocity.x;
    }

    wheel->hub_velocity.y = new_velocity.y;
}

static void set_angular_velocity(Wheel* wheel, float new_velocity, Vector2f velocity_cog)
{
    // Only apply artificial rotation when the vehicle is standing still
    if (velocity_cog.x < EPSILON) {
        float new_thread_vel = new_velocity * wheel->effective_radius;
        float hub_vel_dir = signum(wheel->hub_velocity.x);
        if (fabs(new_thread_vel) < wheel->min_speed || signum(new_thread_vel) != hub_vel_dir) {
            wheel->angular_velocity = hub_vel_dir * wheel->min_speed / wheel->effective_radius;
        } else {
            wheel->angular_velocity = new_velocity;
        }
    } else if (signum(velocity_cog.x) != signum(new_velocity)) {
        // lock the wheel
        wheel->angular_velocity = 0.0;
    } else {
        wheel->angular_velocity = new_velocity;
    }
}

void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inertia, float torque, float dt)
{
    Vector2f hub_velocity
        = translate_velocity(velocity_cog, yaw_angular_velocity_cog, wheel->position);
    set_hub_speed(wheel, hub_velocity);

    float total_torque = torque + wheel->reaction_torque;
    float dv = total_torque / (external_inertia + wheel->inertia);
    float new_velocity = wheel->angular_velocity + integrate(dv, dt);

    set_angular_velocity(wheel, new_velocity, velocity_cog);
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

Vector2f wheel_force(Wheel* wheel, TireModel* model, float normal_force, float friction_coefficent)
{
    Vector2f slip = wheel_slip(wheel);
    Vector2f force = tiremodel_force(model, normal_force, slip.x, slip.y, friction_coefficent);
    wheel->reaction_torque = wheel_reaction_torque(wheel, force);
    return force;
}
