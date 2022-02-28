#ifndef POWERTRAIN_H
#define POWERTRAIN_H

typedef float AngularVelocity;

typedef struct
{
    AngularVelocity angular_velocity;
    float inv_inertia;
    // TODO: table
    float torque;
} Engine;

Engine* engine_new(float inv_inertia);
float engine_torque(Engine* engine, float throttle_pos);
void engine_free(Engine* engine);

typedef struct
{
    float ratio;
    float inv_inertia;
} Differential;

void differential_torque(Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque);
float differential_velocity(Differential* diff, float left_angular_velocity, float right_angular_velocity);
#endif /* POWERTRAIN_H */
