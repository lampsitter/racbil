#ifndef RA_BRAKE_H
#define RA_BRAKE_H
#include <sys/types.h>

typedef struct {
    float area;
} Cylinder;

Cylinder cylinder_from_diameter(float diameter);
float cylinder_force_to_pressure(const Cylinder* c, float force);
float cylinder_pressure_to_force(const Cylinder* c, float pressure);

typedef struct {
    float max_pressure;
} MasterCylinder;

MasterCylinder master_cylinder_new(float max_pressure);

typedef struct {
    // TODO: vary with temperature (calculated at macroscopic level?)
    float kinetic_friction_coeff;
    float static_friction_coeff;
} BrakeDisc;

BrakeDisc brake_disc_new(float static_friction_coeff, float kinetic_friction_coeff);

typedef struct {
    Cylinder cylinder;
    /* The pads effective_radius relative to the center of the brake disc */
    float effective_radius;
    float num_pads;
} Caliper;

/** num_pads must always be a pair number, since there is a pad on each side of the disc*/
Caliper caliper_new(Cylinder cylinder, float effective_radius, u_int8_t num_pads);

float brake_torque(const BrakeDisc* b, const Caliper* c, float pressure, float angular_velocity,
    float hub_velocity_x);

#endif
