#include "../common.h"
#include "../powertrain.h"
#include "../wheel.h"
#include <assert.h>
#include <stdlib.h>

#define ASSERT_EQ(a, b) assert(fabs(a - b) < EPSILON)
#define ASSERT_NEQ(a, b) assert(fabs(a - b) > EPSILON)

int main(void)
{
    Differential* diff = differential_new(2.4, 0.18, DiffTypeOpen);

    float min_speed = 0.01;
    Wheel* wl = wheel_new(0.5, 0.344, (Vector2f) { .x = 1.0, .y = 0.0 }, min_speed);
    Wheel* wr = wheel_new(0.5, 0.344, (Vector2f) { .x = -1.0, .y = 0.0 }, min_speed);
    ASSERT_EQ(wl->angular_velocity, wr->angular_velocity);

    float yaw_vel = 3.0;
    wl->reaction_torque = -1000.0;
    wr->reaction_torque = -500.0;

    wl->external_torque = -500.0;
    wr->external_torque = -100.0;

    // Open
    {
        float t_left, t_right;
        differential_torque(
            diff, 500.0, wl->reaction_torque, wr->reaction_torque, &t_left, &t_right);
        wheel_update(wl, (Vector2f) { .x = 10.0, .y = 0.0 }, yaw_vel, 0.0, t_left, 1.0 / 100.0);
        wheel_update(wr, (Vector2f) { .x = 10.0, .y = 0.0 }, yaw_vel, 0.0, t_right, 1.0 / 100.0);
    }

    ASSERT_NEQ(wl->angular_velocity, wr->angular_velocity);

    // Locked
    wl->angular_velocity = 0.0;
    wr->angular_velocity = 0.0;

    diff->ty = DiffTypeLocked;

    {
        float t_left, t_right;
        differential_torque(
            diff, 500.0, wl->reaction_torque, wr->reaction_torque, &t_left, &t_right);
        wheel_update(wl, (Vector2f) { .x = 10.0, .y = 0.0 }, yaw_vel, 0.0, t_left, 1.0 / 100.0);
        wheel_update(wr, (Vector2f) { .x = 10.0, .y = 0.0 }, yaw_vel, 0.0, t_right, 1.0 / 100.0);
    }

    ASSERT_EQ(wl->angular_velocity, wr->angular_velocity);

    free(wl);
    free(wr);
    free(diff);

    return 0;
}
