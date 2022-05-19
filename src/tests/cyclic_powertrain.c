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

    raTaggedComponent* cengine = ra_tag_engine(engine);
    raTaggedComponent* cdiff = ra_tag_differential(diff);

    assert(ra_tagged_add_next(cengine, cengine) == RaErrorTaggedCyclic);
    assert(cengine->prev == NULL);
    assert(cengine->tty.normal.next == NULL);
    assert(ra_tagged_add_next(cengine, cdiff) == 0);
    assert(ra_tagged_add_next_left(cdiff, cengine) == RaErrorTaggedCyclic);
    assert(cdiff->prev == cengine);
    assert(cdiff->tty.split.next_left == NULL);
    assert(ra_tagged_add_next_right(cdiff, cengine) == RaErrorTaggedCyclic);
    assert(cdiff->prev == cengine);
    assert(cdiff->tty.split.next_right == NULL);

    ra_tagged_component_free(cengine);
}
