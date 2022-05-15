#include "../common.h"
#include "../powertrain.h"
#include "test.h"
#include <assert.h>

int main(void)
{
    Gearbox* gb = test_gearbox();

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
