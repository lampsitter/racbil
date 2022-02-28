#include "powertrain.h"
#include <stdlib.h>

Engine* engine_new(float inv_inertia)
{
    Engine* engine = malloc(sizeof *engine);
    engine->angular_velocity = 0.0f;
    engine->inv_inertia = inv_inertia;
    engine->torque = 50.0f;
    return engine;
}

float engine_torque(Engine* engine, float throttle_pos) { return engine->torque * throttle_pos; }

void engine_free(Engine* engine) { free(engine); }

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
