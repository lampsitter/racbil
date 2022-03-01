#ifndef POWERTRAIN_H
#define POWERTRAIN_H
#include "common.h"

typedef struct {
    AngularVelocity angular_velocity;
    float inv_inertia;
    // TODO: table
    float torque;
} Engine;

Engine engine_new(float inv_inertia);
float engine_torque(Engine* engine, float throttle_pos);

typedef struct {
    float ratio;
    float inv_inertia;
} Differential;

void differential_torque(
    Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque);
float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity);

typedef struct {
    int num_gears;
    int curr_gear;
    float reverse_ratio;
    float reverse_inertia;
    float* ratios;
    float* inertias;
} Gearbox;

Gearbox gearbox_new(
    float num_gears, float* ratios, float* inertias, float reverse_ratio, float reverse_inertia);
float gearbox_ratio(const Gearbox* trans);
float gearbox_inertia(const Gearbox* trans);
#endif /* POWERTRAIN_H */
