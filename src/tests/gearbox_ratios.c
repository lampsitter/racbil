#include "../common.h"
#include "../powertrain.h"
#include "test.h"
#include <assert.h>
#include <math.h>

#define ASSERT_EQF(a, b) assert(fabs(a - b) < EPSILON)
int main(void)
{
    Gearbox* gb = test_gearbox();
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
    ASSERT_EQF(gearbox_torque_out(gb, 1.0), -3.6);
    ASSERT_EQF(gearbox_inertia(gb), 0.3);

    gearbox_free(gb);
}
