#include "powertrain.h"
#include "common.h"
#include <stdlib.h>

Engine engine_new(float inv_inertia, Table torque_map, float max_map_rpm, float max_map_torque)
{
    return (Engine) {
        .torque_map = torque_map,
        .angular_velocity = 0.0f,
        .inv_inertia = inv_inertia,
        .max_rpm = max_map_rpm,
        .max_torque = max_map_torque,
    };
}

void engine_free(Engine* engine) { table_free(&engine->torque_map); }

float engine_torque(Engine* engine, float throttle_pos)
{
    float rpm = angular_vel_rads_to_rpm(engine->angular_velocity);
    float normalized_torque
        = table_lookup(&engine->torque_map, throttle_pos, rpm / engine->max_rpm);
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
    return (Gearbox) { .ratios = ratios,
        .inertias = inertias,
        .reverse_ratio = reverse_ratio,
        .reverse_inertia = reverse_inertia,
        .curr_gear = 0 };
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
