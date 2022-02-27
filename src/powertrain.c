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

float engine_torque(Engine* engine, float throttle_pos)
{
    return engine->torque * throttle_pos;
}

void engine_free(Engine* engine)
{
    free(engine);
}

