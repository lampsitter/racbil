#include "powertrain.h"
#include "common.h"
#include <stdbool.h>
#include <stdlib.h>

Engine engine_new(float inertia, Table torque_map, float max_map_rpm, float max_map_torque)
{
    return (Engine) {
        .torque_map = torque_map,
        .angular_velocity = 0.0f,
        .inertia = inertia,
        .max_angular_velocity = angular_vel_rpm_to_rads(max_map_rpm),
        .max_torque = max_map_torque,
    };
}

void engine_free(Engine* engine) { table_free(&engine->torque_map); }

float engine_torque(Engine* engine, float throttle_pos)
{
    float normalized_torque = table_lookup(
        &engine->torque_map, throttle_pos, engine->angular_velocity / engine->max_angular_velocity);
    return normalized_torque * engine->max_torque;
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

Gearbox gearbox_new(VecFloat ratios, VecFloat inertias, float reverse_ratio, float reverse_inertia)
{
    return (Gearbox) {
        .ratios = ratios,
        .inertias = inertias,
        .reverse_ratio = reverse_ratio,
        .reverse_inertia = reverse_inertia,
        .curr_gear = 0,
        .input_angular_velocity = 0.0,
    };
}

void gearbox_free(Gearbox* gb)
{
    vec_free(&gb->ratios);
    vec_free(&gb->inertias);
}

float gearbox_ratio(const Gearbox* trans)
{
    int curr_gear = trans->curr_gear;
    if (curr_gear < 0) {
        return trans->reverse_ratio;
    } else if (curr_gear == 0) {
        return 0;
    } else {
        return trans->ratios.elements[curr_gear - 1];
    }
}

float gearbox_inertia(const Gearbox* trans)
{
    int curr_gear = trans->curr_gear;
    if (curr_gear < 0) {
        return trans->reverse_inertia;
    } else if (curr_gear == 0) {
        return 0;
    } else {
        return trans->inertias.elements[curr_gear - 1];
    }
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

    float threshold = 0.1;
    float torque_sensitivity = 0.1;
    if (fabsf(vel_diff) < threshold && fabsf(torque_in) <= static_torque) {
        // Locked
        *torque_left = torque_in;
        *torque_right = torque_in;
        clutch->is_locked = true;
    } else {
        // Slipping
        float kinetic_torque = clutch->kinetic_coefficient * normal_force;
        float out_torque = tanh_friction(kinetic_torque, vel_diff, torque_sensitivity);
        *torque_left = torque_in - out_torque;
        *torque_right = out_torque;
        clutch->is_locked = false;
    }
}
