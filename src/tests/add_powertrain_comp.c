#include "../common.h"
#include "../powertrain.h"
#include "../powertrainabs.h"
#include "../wheel.h"
#include <assert.h>
#include <stdlib.h>

int main(void)
{
    Table torque_map = table_with_capacity(2, 2);
    Engine* engine = engine_new(0.5, torque_map);
    Differential* diff = differential_new(2.4, 0.18, DiffTypeLocked);
    Wheel* w = wheel_new(0.6, 0.344, (Vector2f) { .x = 0.0, .y = 0.0 }, 0.01);

    raTaggedComponent* cengine = ra_tag_engine(engine);
    raTaggedComponent* cdiff = ra_tag_differential(diff);
    raTaggedComponent* cwheel = ra_tag_wheel(w);

    assert(ra_tagged_add_next(cengine, cdiff) == 0);
    assert(ra_tagged_add_next_left(cdiff, cwheel) == 0);
    cdiff->tty.split.next_left = NULL;
    assert(ra_tagged_add_next_right(cdiff, cwheel) == 0);

    assert(ra_tagged_add_next(cdiff, cwheel) == RaErrorTaggedInvalid);
    assert(ra_tagged_add_next_left(cengine, cwheel) == RaErrorTaggedInvalid);
    assert(ra_tagged_add_next_right(cengine, cwheel) == RaErrorTaggedInvalid);

    assert(ra_tagged_add_next_left(cdiff, cwheel) == RaErrorTaggedSame);
    cdiff->tty.split.next_right = NULL;
    assert(ra_tagged_add_next_left(cdiff, cwheel) == 0);
    assert(ra_tagged_add_next_right(cdiff, cwheel) == RaErrorTaggedSame);

    ra_tagged_component_free(cengine);
}
