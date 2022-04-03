#ifndef POWERTRAIN_H
#define POWERTRAIN_H
#include "common.h"
#include <stdbool.h>

typedef struct {
    /**
     * `x` is the throttle position from 0.0 to 1.0
     * `y` is the rpm from 0.0 to 1.0
     * `z` is torque from 0.0 to 1.0
     * */
    Table torque_map;
    AngularVelocity angular_velocity;
    float inertia;
} Engine;

Engine engine_new(float inertia, Table torque_map);
void engine_free(Engine* engine);
float engine_torque(Engine* engine, float throttle_pos);
void engine_set_angular_velocity(Engine* engine, AngularVelocity velocity);

typedef struct {
    float ratio;
    float inertia;
} Differential;

void differential_torque(
    Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque);
float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity);

typedef struct {
    int curr_gear;
    float reverse_ratio;
    float reverse_inertia;
    float input_angular_velocity;
    VecFloat ratios;
    VecFloat inertias;
} Gearbox;

Gearbox gearbox_new(VecFloat ratios, VecFloat inertias, float reverse_ratio, float reverse_inertia);
void gearbox_free(Gearbox* gb);
float gearbox_inertia(const Gearbox* gb);
float gearbox_torque_out(const Gearbox* gb, float torque_in);
float gearbox_angular_velocity_in(Gearbox* gb, float angular_velocity_out);

typedef struct {
    float kinetic_coefficient;
    float static_coefficient;
    bool is_locked;
} Clutch;

Clutch clutch_with_torque(
    float* max_normal_force, float max_static_torque, float max_kinetic_torque);

void clutch_torque_out(Clutch* clutch, float torque_in, float normal_force,
    AngularVelocity left_vel, AngularVelocity right_vel, float* torque_left, float* torque_right);

#endif /* POWERTRAIN_H */
