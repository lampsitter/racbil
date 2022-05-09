#include "../brake.h"
#include "../common.h"
#include "../wheel.h"
#include <assert.h>
#include <stdlib.h>

int main(void)
{
    float min_speed = 0.01;
    Wheel* ww = wheel_new(0.5, 0.344, (Vector2f) { .x = 0.0, .y = 0.0 }, min_speed);
    Wheel w = *ww;
    assert(w.angular_velocity == min_speed / w.effective_radius);
    assert(w.hub_velocity.x == min_speed);

    MasterCylinder mc = master_cylinder_new(17000e3);
    BrakeDisc disc = brake_disc_new(0.3, 0.24);
    Caliper caliper = caliper_new(cylinder_from_diameter(0.05), 0.18, 2);

    float bt = brake_torque(&disc, &caliper, mc.max_pressure, w.angular_velocity);

    wheel_update(&w, (Vector2f) { .x = 0.0, .y = 0.0 }, 0.0, 0.0, bt, 1.0 / 400.0);
    assert(w.angular_velocity == min_speed / w.effective_radius);
    assert(w.hub_velocity.x == min_speed);

    wheel_update(&w, (Vector2f) { .x = 2.0, .y = 0.0 }, 0.0, 0.0, bt, 1.0 / 400.0);
    assert(fabs(w.angular_velocity) < EPSILON);
    assert(w.hub_velocity.x == 2.0);

    free(ww);
    return 0;
}
