#include "wheel.h"
#include "common.h"

Wheel wheel_new(float inv_inertia, float radius, Vector2f position, float min_speed)
{
    return (Wheel) {
        .inv_inertia = inv_inertia,
        .effective_radius = radius,
        .position = position,
        .angular_velocity = min_speed / radius,
        .hub_velocity = (Vector2f) { .x = min_speed, .y = 0.0 },
        .angle = 0.0,
        .reaction_torque = 0.0f,
    };
}

void wheel_change_angle(Wheel* wheel, float angle) { wheel->angle = angle; }

static Vector2f translate_velocity(
    Vector2f velocity_cog, float yaw_angular_velocity_cog, Vector2f position)
{
    float x = velocity_cog.x - yaw_angular_velocity_cog * position.y;
    float y = velocity_cog.y + yaw_angular_velocity_cog * position.x;

    return (Vector2f) { .x = x, .y = y };
}

static void set_hub_speed(Wheel* wheel, Vector2f new_velocity, float min_speed)
{
    if (signum(new_velocity.x) != signum(wheel->hub_velocity.x)) {
        // This prevents driving in reverse, so the hub velocity must be flipped
        // manually when the car is set into reverse
        wheel->hub_velocity.x = signum(wheel->hub_velocity.x) * min_speed;
    } else if (fabs(new_velocity.x) < min_speed) {
        wheel->hub_velocity.x = signum(wheel->hub_velocity.x) * min_speed;
    } else {
        wheel->hub_velocity.x = new_velocity.x;
    }

    wheel->hub_velocity.y = new_velocity.y;
}

static void set_angular_velocity(
    Wheel* wheel, float new_velocity, Vector2f velocity_cog, float min_speed)
{
    // Only apply artificial rotation when the vehicle is standing still
    if (velocity_cog.x < EPSILON) {
        wheel->angular_velocity = new_velocity;
        if (fabs(wheel->angular_velocity * wheel->effective_radius) < min_speed) {
            wheel->angular_velocity
                = signum(wheel->angular_velocity) * min_speed / wheel->effective_radius;
        }
    } else if (signum(velocity_cog.x) != signum(new_velocity)) {
        // lock the wheel
        wheel->angular_velocity = 0.0;
    } else {
        wheel->angular_velocity = new_velocity;
    }
}

void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inv_inertia, float torque, float dt, float min_speed)
{
    Vector2f hub_velocity
        = translate_velocity(velocity_cog, yaw_angular_velocity_cog, wheel->position);
    set_hub_speed(wheel, hub_velocity, min_speed);

    float total_torque = torque + wheel->reaction_torque;
    float dv = total_torque * (external_inv_inertia + wheel->inv_inertia);
    float new_velocity = wheel->angular_velocity + integrate(dv, dt);

    set_angular_velocity(wheel, new_velocity, velocity_cog, min_speed);
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
