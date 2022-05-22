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
    w->input_torque = 0.0;
    w->reaction_torque = 0.0;
    w->external_torque = 0.0;
    return w;
}

void wheel_change_direction(Wheel* w, WheelDirection d)
{
    float min_angular = w->min_speed / w->effective_radius;
    if (d == WheelDirectionForward) {
        w->angular_velocity = min_angular;
        w->hub_velocity.x = w->min_speed;
    } else if (d == WheelDirectionReverse) {
        w->angular_velocity = -min_angular;
        w->hub_velocity.x = -w->min_speed;
    }
}

void wheel_try_change_direction(Wheel* w, WheelDirection d)
{
    if (fabsf(fabsf(w->hub_velocity.x) - w->min_speed) < EPSILON
        && fabsf(w->angular_velocity) <= w->min_speed / w->effective_radius) {
        wheel_change_direction(w, d);
    }
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
    if (signum(new_velocity.x) != signum(wheel->hub_velocity.x)
        || fabsf(new_velocity.x) < wheel->min_speed) {
        // This prevents driving in reverse, so the hub velocity must be flipped
        // manually when the car is set into reverse
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
        if (fabsf(new_thread_vel) < wheel->min_speed || signum(new_thread_vel) != hub_vel_dir) {
            wheel->angular_velocity = hub_vel_dir * wheel->min_speed / wheel->effective_radius;
        } else {
            wheel->angular_velocity = new_velocity;
        }
    } else if (signum(wheel->hub_velocity.x) != signum(new_velocity)) {
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

    wheel->input_torque = torque;

    float total_torque = torque + wheel->reaction_torque + wheel->external_torque;
    float dv = total_torque / (external_inertia + wheel->inertia);
    float new_velocity = wheel->angular_velocity + integrate(dv, dt);

    set_angular_velocity(wheel, new_velocity, velocity_cog);
}

static float wheel_reaction_torque(const Wheel* wheel, Vector2f force)
{
    return -force.x * wheel->effective_radius;
}

static float wheel_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d)
{
    SUPPRESS_UNUSED(prev);
    float inertia = ((Wheel*)t->ty)->inertia;
    if (d == raInertiaDirectionNext) {
        return inertia;
    } else {
        return ra_tagged_inertia(t->prev, t, raInertiaDirectionPrev) + inertia;
    }
}

static float wheel_ang_vel(raTaggedComponent* t) { return ((Wheel*)t->ty)->angular_velocity; }

static void wheel_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    float inertia = ra_tagged_inertia(t->prev, t, raInertiaDirectionPrev);

    wheel_update((Wheel*)t->ty, v.velocity_cog, v.yaw_velocity_cog, inertia, torque, dt);
}

static float wheel_update_angular_velocity(raTaggedComponent* t)
{
    // Wheel velocity has already been updated in wheel_send_torque.
    return wheel_ang_vel(t);
}

static float wheel_ext_torque(raTaggedComponent* t)
{
    Wheel* w = (Wheel*)t->ty;
    return w->reaction_torque + w->external_torque;
}

raTaggedComponent* ra_tag_wheel(Wheel* w)
{
    return ra_tagged_new(w, wheel_inertia, wheel_ang_vel, wheel_send_torque, NULL,
        wheel_update_angular_velocity, wheel_ext_torque, free);
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
