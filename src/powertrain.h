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

#endif /* POWERTRAIN_H */
