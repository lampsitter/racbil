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

Engine* engine_new(float inertia, Table torque_map);
raTaggedComponent* ra_tag_engine(Engine* engine);
void engine_free(Engine* engine);
float engine_torque(Engine* engine, float throttle_pos);
void engine_set_angular_velocity(Engine* engine, AngularVelocity velocity);

/** Disables all engine throttle input when the engine velocity goes above
 * `activation_angular_velocity`. The limiter will only be disabled once the engine
 * goes below `deactivation_angular_velocity`*/
typedef struct {
    float activation_angular_velocity;
    float deactivation_angular_velocity;
    bool is_active;
} RevLimiterHard;

RevLimiterHard rev_limiter_hard_new(
    float activation_angular_velocity, float deactivation_angular_velocity);
float rev_limiter_hard(RevLimiterHard* r, Engine* e, float throttle_pos);

typedef struct {
    float ratio;
    float inertia;
} Differential;

Differential* differential_new(float ratio, float inertia);
raTaggedComponent* ra_tag_differential(Differential* diff);
void differential_torque(
    Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque);
float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity);

typedef struct {
    int curr_gear;
    float input_angular_velocity;
    VecFloat ratios;
    VecFloat inertias;
} Gearbox;

/**Reverse ratio/inertia is the first element in the lists*/
Gearbox* gearbox_new(VecFloat ratios, VecFloat inertias);
raTaggedComponent* ra_tag_gearbox(Gearbox* gb);
void gearbox_free(Gearbox* gb);
float gearbox_inertia(const Gearbox* gb);
float gearbox_torque_out(const Gearbox* gb, float torque_in);
float gearbox_angular_velocity_in(Gearbox* gb, float angular_velocity_out);

typedef struct {
    float velocity_threshold;
    float torque_sensitivity;
    float kinetic_coefficient;
    float static_coefficient;
    bool is_locked;
} Clutch;

Clutch* clutch_with_torque(
    float* max_normal_force, float max_static_torque, float max_kinetic_torque);

raTaggedComponent* ra_tag_clutch(Clutch* c);
void clutch_torque_out(Clutch* clutch, float torque_in, float normal_force,
    AngularVelocity left_vel, AngularVelocity right_vel, float* torque_left, float* torque_right);

#endif /* POWERTRAIN_H */
