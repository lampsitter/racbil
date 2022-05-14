#include "../common.h"
#include "../powertrain.h"
#include <assert.h>

int main(void)
{
    VecFloat ratios = vec_with_capacity(7);
    vec_push_float(&ratios, -1.6);
    vec_push_float(&ratios, 3.2);
    vec_push_float(&ratios, 2.31);
    vec_push_float(&ratios, 1.82);
    vec_push_float(&ratios, 1.52);
    vec_push_float(&ratios, 1.3);
    vec_push_float(&ratios, 1.0);

    VecFloat inertias = vec_with_capacity(7);
    vec_push_float(&inertias, 0.3);
    vec_push_float(&inertias, 0.2);
    vec_push_float(&inertias, 0.18);
    vec_push_float(&inertias, 0.16);
    vec_push_float(&inertias, 0.15);
    vec_push_float(&inertias, 0.14);
    vec_push_float(&inertias, 0.1);

    Gearbox* gb = gearbox_new(ratios, inertias);

    assert(gb->curr_gear == 0);
    gearbox_downshift(gb);
    assert(gb->curr_gear == -1);
    gearbox_downshift(gb);
    assert(gb->curr_gear == -1);

    gb->curr_gear = 4;
    gearbox_upshift(gb);
    assert(gb->curr_gear == 5);
    gearbox_upshift(gb);
    assert(gb->curr_gear == 6);
    gearbox_upshift(gb);
    assert(gb->curr_gear == 6);

    gearbox_free(gb);
}
