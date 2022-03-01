#include "body.h"
#include "common.h"
#include "powertrain.h"
#include "tiremodel.h"
#include "wheel.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(void)
{
    float throttle_pos = 1.0;
    float steering_angle = 0.0;

    float dt = 1.0 / 50.0;
    float mass = 1580.0f;
    float inv_vehicle_mass = 1.0 / mass;
    float gravity = 9.806f;
    float air_density = 1.2041f;

    // Uses iso8855 coordinates
    Vector2f velocity = vector2f_default();
    float yaw = 0.0;

    Body body = body_new(0.36, 1.9, 3.6f, 1.47f, 1.475f);
    Engine engine = engine_new(1.0 / 0.5);

    int num_gears = 6;
    float* ratios = malloc(sizeof *ratios * num_gears);
    ratios[0] = 3.2;
    ratios[1] = 2.31;
    ratios[2] = 1.82;
    ratios[3] = 1.52;
    ratios[4] = 1.3;
    ratios[5] = 1.0;
    float* inertias = malloc(sizeof *inertias * num_gears);
    inertias[0] = 0.2;
    inertias[1] = 0.18;
    inertias[2] = 0.16;
    inertias[3] = 0.15;
    inertias[4] = 0.14;
    inertias[5] = 0.1;

    Gearbox gb = gearbox_new(num_gears, ratios, inertias, -1.6, 0.95);
    gb.curr_gear = 1;

    Differential diff = (Differential) { .ratio = 2.4, .inv_inertia = 1.0 / 0.18 };

    Cog cog = cog_from_distribution(0.55, 0.4, body.wheelbase);

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

    Vector2f fl_pos = (Vector2f) { .x = cog_distance_to_front(cog),
        .y = cog_distance_to_left(cog, body.front_track_width) };
    Vector2f fr_pos = (Vector2f) { .x = cog_distance_to_front(cog),
        .y = cog_distance_to_right(cog, body.front_track_width) };
    Vector2f rl_pos = (Vector2f) { .x = cog_distance_to_rear(cog, body.wheelbase),
        .y = cog_distance_to_left(cog, body.rear_track_width) };
    Vector2f rr_pos = (Vector2f) { .x = cog_distance_to_rear(cog, body.wheelbase),
        .y = cog_distance_to_right(cog, body.rear_track_width) };

    assert(fl_pos.x > 0.0 && fl_pos.y > 0.0);
    assert(fr_pos.x > 0.0 && fr_pos.y < 0.0);
    assert(rl_pos.x < 0.0 && rl_pos.y > 0.0);
    assert(rr_pos.x < 0.0 && rr_pos.y < 0.0);

    Wheel wfl = wheel_new(1.0 / 0.6, 0.344, -0.2, fl_pos);
    Wheel wfr = wheel_new(1.0 / 0.6, 0.344, 0.2, fr_pos);

    Wheel wrl = wheel_new(1.0 / 0.6, 0.344, -0.5, rl_pos);
    Wheel wrr = wheel_new(1.0 / 0.6, 0.344, 0.5, rr_pos);

    while (1) {
        wheel_change_angle(&wfl, steering_angle);
        wheel_change_angle(&wfr, steering_angle);

        float t_ratio = gearbox_ratio(&gb);
        float eng_torque = engine_torque(&engine, throttle_pos);
        float trans_torque = eng_torque * t_ratio;
        float inv_inertia = engine.inv_inertia + diff.inv_inertia + gearbox_inertia(&gb);

        float left_torque;
        float right_torque;
        differential_torque(&diff, trans_torque, &left_torque, &right_torque);

        wheel_update(&wfl, velocity, yaw, 0.0, 0.0f, dt);
        wheel_update(&wfr, velocity, yaw, 0.0, 0.0f, dt);

        wheel_update(&wrl, velocity, yaw, inv_inertia, left_torque, dt);
        wheel_update(&wrr, velocity, yaw, inv_inertia, right_torque, dt);

        float fz = mass * gravity * 0.25;

        Vector2f wfl_f = vector2f_rotate(wheel_force(&wfl, &model, fz, 1.0), -wfl.angle);
        Vector2f wfr_f = vector2f_rotate(wheel_force(&wfr, &model, fz, 1.0), -wfr.angle);
        Vector2f wrl_f = vector2f_rotate(wheel_force(&wrl, &model, fz, 1.0), -wrl.angle);
        Vector2f wrr_f = vector2f_rotate(wheel_force(&wrr, &model, fz, 1.0), -wrr.angle);

        float resitance_force_x = body_air_resistance(&body, air_density, velocity.x);
        Vector2f force
            = (Vector2f) { .x = wfl_f.x + wfr_f.x + wrl_f.x + wrr_f.x + resitance_force_x,
                  .y = wfl_f.y + wfr_f.y + wrl_f.y + wrr_f.y };

        printf("Force: %f/%f\n", force.x, force.y);

        velocity.x += integrate(force.x, inv_vehicle_mass, dt);
        velocity.y += integrate(force.y, inv_vehicle_mass, dt);
        // To prevent inital flip flop. TODO: remove
        if (velocity.x < 0.0) {
            velocity.x = 0.0;
        }

        engine.angular_velocity
            = differential_velocity(&diff, wrl.angular_velocity, wrr.angular_velocity) * t_ratio;

        Vector2f slip_front = wheel_slip(&wfl);
        Vector2f slip_rear = wheel_slip(&wrl);
        printf("Slip F x/y = %.2f/%.2f | Slip R = %.2f/%.2f | ", slip_front.x, slip_front.y,
            slip_rear.x, slip_rear.y);
        printf("m/s = %f/%f\n", velocity.x, velocity.y);

        // Just to get a feel for the simulation
        sleep(dt);
    }

    free(ratios);
    free(inertias);

    return 0;
}
