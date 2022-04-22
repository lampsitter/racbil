#include "powertrain.h"
#include "common.h"
#include <stdbool.h>
#include <stdlib.h>

RevLimiterHard rev_limiter_hard_new(
    float activation_angular_velocity, float deactivation_angular_velocity)
{
    return (RevLimiterHard) {
        .activation_angular_velocity = activation_angular_velocity,
        .deactivation_angular_velocity = deactivation_angular_velocity,
        .is_active = false,
    };
}

float rev_limiter_hard(RevLimiterHard* r, Engine* e, float throttle_pos)
{
    if (r->is_active && e->angular_velocity < r->deactivation_angular_velocity) {
        r->is_active = false;
    } else if (!r->is_active && e->angular_velocity >= r->activation_angular_velocity) {
        r->is_active = true;
    }

    if (r->is_active) {
        return 0.0;
    } else {
        return throttle_pos;
    }
}

Engine engine_new(float inertia, Table torque_map)
{
    return (Engine) {
        .torque_map = torque_map,
        .angular_velocity = 0.0f,
        .inertia = inertia,
    };
}

void engine_free(Engine* engine) { table_free(&engine->torque_map); }

float engine_torque(Engine* engine, float throttle_pos)
{
    return table_lookup(&engine->torque_map, throttle_pos, engine->angular_velocity);
}

void engine_set_angular_velocity(Engine* engine, AngularVelocity velocity)
{
    engine->angular_velocity = fmaxf(velocity, 0.0);
}

void differential_torque(
    Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque)
{
    float torque = input_torque * diff->ratio;
    // Simple open diff for now
    *output_left_torque = torque * 0.5;
    *output_right_torque = torque * 0.5;
}

float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity)
{
    return (left_angular_velocity + right_angular_velocity) * 0.5 * diff->ratio;
}

Gearbox gearbox_new(VecFloat ratios, VecFloat inertias)
{
    return (Gearbox) {
        .ratios = ratios,
        .inertias = inertias,
        .curr_gear = 0,
        .input_angular_velocity = 0.0,
    };
}

void gearbox_free(Gearbox* gb)
{
    vec_free(&gb->ratios);
    vec_free(&gb->inertias);
}

static float gearbox_ratio(const Gearbox* gb)
{
    int curr_gear = gb->curr_gear;
    if (curr_gear < 0) {
        return gb->ratios.elements[0];
    } else if (curr_gear == 0) {
        return 0;
    } else {
        return gb->ratios.elements[curr_gear];
    }
}

float gearbox_inertia(const Gearbox* gb)
{
    int curr_gear = gb->curr_gear;
    if (curr_gear < 0) {
        return gb->inertias.elements[0];
    } else if (curr_gear == 0) {
        return 0;
    } else {
        return gb->inertias.elements[curr_gear];
    }
}

float gearbox_angular_velocity_in(Gearbox* gb, float angular_velocity_out)
{
    if (gb->curr_gear != 0) {
        gb->input_angular_velocity = gearbox_ratio(gb) * angular_velocity_out;
    }
    return gb->input_angular_velocity;
}

float gearbox_torque_out(const Gearbox* gb, float torque_in)
{
    return gearbox_ratio(gb) * torque_in;
}

// Calculate max normal force from desired torque caracheristics of the clutch.
Clutch clutch_with_torque(
    float* max_normal_force, float max_static_torque, float max_kinetic_torque)
{
    *max_normal_force = max_static_torque;
    float static_coefficient = 1.0f;
    float kinetic_coefficient = max_kinetic_torque / *max_normal_force;

    return (Clutch) { .static_coefficient = static_coefficient,
        .kinetic_coefficient = kinetic_coefficient,
        .velocity_threshold = 1.0,
        .torque_sensitivity = 2.0,
        .is_locked = false };
}

static inline float tanh_friction(float torque, AngularVelocity vel_diff, float transition)
{
    return torque * tanh(2.0 * (vel_diff / transition));
}

void clutch_torque_out(Clutch* clutch, float torque_in, float normal_force,
    AngularVelocity left_vel, AngularVelocity right_vel, float* torque_left, float* torque_right)
{
    float vel_diff = left_vel - right_vel;
    // The whole formula is technically (2/3) * effective_radius * normal_force *
    // static_coefficient. For simplisity (2/3) * effective_radius is dropped, reducing the amount
    // of variables that need to be configured. It can be compensated for by including it in the
    // input normal force if necessary.
    float static_torque = clutch->static_coefficient * normal_force;
    if (fabsf(vel_diff) < clutch->velocity_threshold && fabsf(torque_in) <= static_torque) {
        // Locked
        *torque_left = torque_in;
        *torque_right = torque_in;
        clutch->is_locked = true;
    } else {
        // Slipping
        float kinetic_torque = clutch->kinetic_coefficient * normal_force;
        float out_torque = tanh_friction(kinetic_torque, vel_diff, clutch->torque_sensitivity);
        *torque_left = torque_in - out_torque;
        *torque_right = out_torque;
        clutch->is_locked = false;
    }
}
