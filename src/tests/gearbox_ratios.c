#include "../common.h"
#include "../powertrain.h"
#include <assert.h>
#include <math.h>

#define ASSERT_EQF(a, b) assert(fabs(a - b) < EPSILON)
int main(void)
{
    int num_gears = 4;
    VecFloat ratios = vec_with_capacity(num_gears);
    vec_push_float(&ratios, -1.6);
    vec_push_float(&ratios, 3.2);
    vec_push_float(&ratios, 2.31);
    vec_push_float(&ratios, 1.82);

    VecFloat inertias = vec_with_capacity(num_gears);
    vec_push_float(&inertias, 0.3);
    vec_push_float(&inertias, 0.2);
    vec_push_float(&inertias, 0.18);
    vec_push_float(&inertias, 0.16);

    Gearbox* gb = gearbox_new(ratios, inertias);
    assert(gb->curr_gear == 0);
    ASSERT_EQF(gearbox_torque_out(gb, 1.0), 0.0);
    ASSERT_EQF(gearbox_inertia(gb), 0.0);

    gb->curr_gear = 1;
    ASSERT_EQF(gearbox_torque_out(gb, 1.0), 3.2);
    ASSERT_EQF(gearbox_inertia(gb), 0.2);

    gb->curr_gear = 3;
    ASSERT_EQF(gearbox_torque_out(gb, 1.0), 1.82);
    ASSERT_EQF(gearbox_inertia(gb), 0.16);

    gb->curr_gear = -1;
    ASSERT_EQF(gearbox_torque_out(gb, 1.0), -1.6);
    ASSERT_EQF(gearbox_inertia(gb), 0.3);

    gearbox_free(gb);
}
