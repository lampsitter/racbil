#include <stdio.h>
#include <math.h>
#include <stdlib.h>

typedef struct
{
    float x, y;
} Vector2f;


Vector2f vector2f_default(void)
{
    return (Vector2f) { .x = 0.0f, .y = 0.0f };
}

typedef struct
{
    float x, y, z;
} Vector3f;

Vector3f vector3f_default(void)
{
    return (Vector3f) { .x = 0.0f, .y = 0.0f, .z = 0.0f };
}

float vector3f_length(Vector3f v)
{
    return (float)sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float integrate(float torque, float inv_inertia, float dt)
{
    return torque * inv_inertia * dt;
}

typedef float AngularVelocity;

typedef struct
{
    AngularVelocity angular_velocity;
    float inv_inertia;
    // TODO: table
    float torque;
} Engine;

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

typedef struct
{
    AngularVelocity angular_velocity;
    float inv_inertia;
    float unloaded_radius;
    float effective_radius;
} Wheel;

Wheel* wheel_new(float inv_inertia, float radius)
{
    Wheel* wheel = malloc(sizeof *wheel);
    wheel->inv_inertia = inv_inertia;
    wheel->effective_radius = wheel->unloaded_radius = radius;
    return wheel;
}


typedef Vector2f Force;

Force wheel_update(Wheel* wheel, float external_inv_inertia, float torque, float dt)
{
    wheel->angular_velocity += integrate(torque, external_inv_inertia, dt);
    return (Vector2f) { .x = torque * wheel->effective_radius, .y = 0.0f };
}


void wheel_free(Wheel* wheel)
{
    free(wheel);
}

int main(int argc, char **argv)
{
    float throttle_pos = 1.0;
    float dt = 1.0 / 50.0;
    float vehicle_mass = 1580.0;
    // Uses iso8855 coordinates
    Vector3f velocity = vector3f_default();
    Engine* engine = engine_new(1.0 / 0.5);
    Wheel* wheel = wheel_new(1.0 / 0.6, 0.344);

    while (1) {
        float torque = engine_torque(engine, throttle_pos);
        Force force = wheel_update(wheel, engine->inv_inertia, torque, dt);
        velocity.x += integrate(force.x, 1.0 / vehicle_mass, dt);
        engine->angular_velocity = wheel->angular_velocity;

        printf("Km/h = %f\r", vector3f_length(velocity) / 3.6);
    }

    engine_free(engine);
    wheel_free(wheel);
    return 0;
}
