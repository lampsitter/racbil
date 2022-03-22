#include "brake.h"
#include "common.h"
#include <assert.h>
#include <math.h>

Cylinder cylinder_from_diameter(float diameter)
{
    return (Cylinder) {
        .area = M_PI * 0.25 * diameter * diameter,
    };
}

float cylinder_force_to_pressure(const Cylinder* c, float force) { return force / c->area; }

float cylinder_pressure_to_force(const Cylinder* c, float pressure) { return pressure * c->area; }

MasterCylinder master_cylinder_new(float max_pressure, float ratio)
{
    assert(ratio >= 0.0 && ratio <= 1.0);
    return (MasterCylinder) {
        .max_pressure = max_pressure,
        .ratio = ratio,
    };
}

void master_cylinder_pressure(
    const MasterCylinder* cyl, float pressure, float* front_pressure, float* rear_pressure)
{
    *front_pressure = cyl->ratio * pressure;
    *rear_pressure = pressure - *front_pressure;
}

BrakeDisc brake_disc_new(float static_friction_coeff, float kinetic_friction_coeff)
{
    return (BrakeDisc) {
        .static_friction_coeff = static_friction_coeff,
        .kinetic_friction_coeff = kinetic_friction_coeff,
    };
}

Caliper caliper_new(Cylinder cylinder, float effective_radius, u_int8_t num_pads)
{
    assert(num_pads % 2 == 0);
    return (Caliper) {
        .num_pads = (float)num_pads,
        .effective_radius = effective_radius,
        .cylinder = cylinder,
    };
}

float brake_torque(const BrakeDisc* b, const Caliper* c, float pressure, float angular_velocity)
{
    float normal_force = cylinder_pressure_to_force(&c->cylinder, pressure);

    // Brakes are stateful due to kinetic torque <-> static torque, but since
    // we always have a minimum wheel hub velocity, we do not have to care
    // about all the issues that arrise when the disc's angular velocity
    // approaches 0. The wheel is able to handle all of that for us.

    float friction_coeff;
    if (fabsf(angular_velocity) < EPSILON) {
        friction_coeff = b->static_friction_coeff;
    } else {
        friction_coeff = b->kinetic_friction_coeff;
    }

    return normal_force * friction_coeff * c->effective_radius * c->num_pads
        * -signum(angular_velocity);
}
