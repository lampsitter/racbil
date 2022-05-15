#include "../powertrain.h"
#include "test.h"
#include <assert.h>

int main(void)
{
    Engine* engine = test_engine();
    RevLimiterHard limiter = rev_limiter_hard_new(rpm_to_rads(1000.0), rpm_to_rads(500.0));

    engine->angular_velocity = rpm_to_rads(600.0);
    assert(1.0 == rev_limiter_hard(&limiter, engine, 1.0));

    engine->angular_velocity = rpm_to_rads(1200.0);
    assert(0.0 == rev_limiter_hard(&limiter, engine, 1.0));

    engine->angular_velocity = rpm_to_rads(600.0);
    assert(0.0 == rev_limiter_hard(&limiter, engine, 1.0));

    engine->angular_velocity = rpm_to_rads(400.0);
    assert(1.0 == rev_limiter_hard(&limiter, engine, 1.0));

    engine->angular_velocity = rpm_to_rads(600.0);
    assert(1.0 == rev_limiter_hard(&limiter, engine, 1.0));

    engine_free(engine);
}
