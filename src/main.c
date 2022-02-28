#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "powertrain.h"
#include "common.h"
#include "tiremodel.h"
#include "wheel.h"

typedef struct
{
    float c_drag;
    float frontal_area;
    float half_cd_a;
} Body;

Body body_new(float c_drag, float frontal_area)
{
    return (Body) { .c_drag = c_drag, .frontal_area = frontal_area, .half_cd_a = 0.5 * c_drag * frontal_area };
}

float body_air_resistance(const Body* body, float air_density, float longitudinal_velocity)
{
    float long_sq = longitudinal_velocity * longitudinal_velocity;
    return -(air_density * body->half_cd_a * long_sq) * signum(longitudinal_velocity);
}

int main(void)
{
    float throttle_pos = 1.0;
    float dt = 1.0 / 50.0;
    float mass = 1580.0f;
    float inv_vehicle_mass = 1.0 / mass;
    float gravity = 9.806f;
    float air_density = 1.2041f;

    // Uses iso8855 coordinates
    Vector2f velocity = vector2f_default();
    float yaw = 0.0;

    Body body = body_new(0.36, 1.9);
    Engine* engine = engine_new(1.0 / 0.5);
    Differential diff = (Differential) { .ratio = 2.4, .inv_inertia = 1.0 / 0.18 };

    TireModel model = (TireModel) {
        .bx = 1.9,
        .cx = 1.65,
        .dx = 1.1,
        .ex = -1.0,
        .vvx = 0.0,
        .vhx = 0.0,

        .by = 9.0,
        .cy = 1.36,
        .dy = 1.0,
        .ey = 0.96,
        .vvy = 0.0,
        .vhy = 0.0,

        .peak_slip_x = 0.4,
        .peak_slip_y = deg_to_rad(20.0f),
    };

    Wheel* wfl = wheel_new(1.0 / 0.6, 0.344, vector2f_default());
    Wheel* wfr = wheel_new(1.0 / 0.6, 0.344, vector2f_default());

    Wheel* wrl = wheel_new(1.0 / 0.6, 0.344, vector2f_default());
    Wheel* wrr = wheel_new(1.0 / 0.6, 0.344, vector2f_default());

    while (1) {
        float torque = engine_torque(engine, throttle_pos);
        float inv_inertia = engine->inv_inertia + diff.inv_inertia;

        float left_torque;
        float right_torque;
        differential_torque(&diff, torque, &left_torque, &right_torque);

        wheel_update(wfl, velocity, yaw, 0.0, 0.0f, dt);
        wheel_update(wfr, velocity, yaw, 0.0, 0.0f, dt);

        wheel_update(wrl, velocity, yaw, inv_inertia, left_torque, dt);
        wheel_update(wrr, velocity, yaw, inv_inertia, right_torque, dt);

        float fz = mass * gravity * 0.25;
        // FIXME: Rotate forces to car coordinates

        Vector2f wfl_f = wheel_force(wfl, &model, fz, 1.0);
        Vector2f wfr_f = wheel_force(wfr, &model, fz, 1.0);
        Vector2f wrl_f = wheel_force(wrl, &model, fz, 1.0);
        Vector2f wrr_f = wheel_force(wrr, &model, fz, 1.0);

        float resitance_force_x = body_air_resistance(&body, air_density, velocity.x);
        Vector2f force = (Vector2f) {
            .x = wfl_f.x + wfr_f.x + wrl_f.x + wrr_f.x + resitance_force_x,
            .y = wfl_f.y + wfr_f.y + wrl_f.y + wrr_f.y
        };

        printf("Force: %f/%f\n", force.x, force.y);

        velocity.x += integrate(force.x, inv_vehicle_mass, dt);
        velocity.y += integrate(force.y, inv_vehicle_mass, dt);
        // To prevent inital flip flop. TODO: remove
        if (velocity.x < 0.0) {
            velocity.x = 0.0;
        }

        engine->angular_velocity =
            differential_velocity(&diff, wrl->angular_velocity, wrr->angular_velocity);

        Vector2f slip_front = wheel_slip(wfl);
        Vector2f slip_rear = wheel_slip(wrl);
        printf("Slip F x/y = %.2f/%.2f | Slip R = %.2f/%.2f | ", slip_front.x, slip_front.y,
                slip_rear.x, slip_rear.y);
        printf("m/s = %f/%f\n", velocity.x, velocity.y);

        // Just to get a feel for the simulation
        sleep(dt);
    }

    engine_free(engine);
    wheel_free(wfl);
    wheel_free(wfr);
    wheel_free(wrl);
    wheel_free(wrr);
    return 0;
}
