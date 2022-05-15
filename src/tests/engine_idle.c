#include "../powertrain.h"
#include "test.h"
#include <assert.h>
#include <stdbool.h>

int main(void)
{
    Engine* engine = test_engine();
    engine->angular_velocity = rpm_to_rads(800.0);

    float max_torque = engine_torque(engine, 1.0);
    float min_torque = engine_torque(engine, 0.0);

    float idle_velocity = rpm_to_rads(900.0);
    float dt = 1.0 / 60.0;
    assert(idle_engine_torque(idle_velocity, engine, max_torque, false, dt) == max_torque);
    assert(idle_engine_torque(idle_velocity, engine, max_torque, true, dt) == max_torque);
    assert(idle_engine_torque(idle_velocity, engine, min_torque, false, dt) == min_torque);
    assert(idle_engine_torque(idle_velocity, engine, min_torque, true, dt) > min_torque);

    engine_free(engine);
}
