#ifndef POWERTRAIN_H
#define POWERTRAIN_H
#include "common.h"

typedef struct {
    /**
     * `x` is the throttle position from 0.0 to 1.0
     * `y` is the rpm from 0.0 to 1.0
     * `z` is torque from 0.0 to 1.0
     * */
    Table torque_map;
    AngularVelocity angular_velocity;
    float inv_inertia;
    float max_rpm;
    float max_torque;
} Engine;

Engine engine_new(float inv_inertia, Table torque_map, float max_map_rpm, float max_map_torque);
void engine_free(Engine* engine);
float engine_torque(Engine* engine, float throttle_pos);
void engine_set_angular_velocity(Engine* engine, AngularVelocity velocity);

typedef struct {
    float ratio;
    float inv_inertia;
} Differential;

void differential_torque(
    Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque);
float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity);

typedef struct {
    int curr_gear;
    float reverse_ratio;
    float reverse_inertia;
    VecFloat ratios;
    VecFloat inertias;
} Gearbox;

Gearbox gearbox_new(VecFloat ratios, VecFloat inertias, float reverse_ratio, float reverse_inertia);
void gearbox_free(Gearbox* gb);
float gearbox_ratio(const Gearbox* trans);
float gearbox_inertia(const Gearbox* trans);
#endif /* POWERTRAIN_H */
