#include "../powertrain.h"
#include <assert.h>

int main(void)
{
    Table torque_map = table_with_capacity(2, 2);
    torque_map.x[0] = 0.0;
    torque_map.x[1] = 1.0;
    torque_map.y[0] = 0.0;
    torque_map.y[1] = 1.0;
    torque_map.z[0][0] = 0.0;
    torque_map.z[0][1] = 0.0;
    torque_map.z[1][0] = 0.0;
    torque_map.z[1][1] = 0.0;

    Engine engine = engine_new(0.5, torque_map);
    RevLimiterHard limiter
        = rev_limiter_hard_new(angular_vel_rpm_to_rads(1000.0), angular_vel_rpm_to_rads(500.0));

    engine.angular_velocity = angular_vel_rpm_to_rads(600.0);
    assert(1.0 == rev_limiter_hard(&limiter, &engine, 1.0));

    engine.angular_velocity = angular_vel_rpm_to_rads(1200.0);
    assert(0.0 == rev_limiter_hard(&limiter, &engine, 1.0));

    engine.angular_velocity = angular_vel_rpm_to_rads(600.0);
    assert(0.0 == rev_limiter_hard(&limiter, &engine, 1.0));

    engine.angular_velocity = angular_vel_rpm_to_rads(400.0);
    assert(1.0 == rev_limiter_hard(&limiter, &engine, 1.0));

    engine.angular_velocity = angular_vel_rpm_to_rads(600.0);
    assert(1.0 == rev_limiter_hard(&limiter, &engine, 1.0));
}
