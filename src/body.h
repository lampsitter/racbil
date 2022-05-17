#ifndef RA_BODY_H
#define RA_BODY_H
#include "common.h"
#include "wheel.h"

typedef Vector3f Cog;

/** This assumes that the cog is in between each axle */
Cog cog_from_distribution(float ratio_front, float height, float wheelbase);
float cog_distance_to_front(Cog cog);
float cog_distance_to_rear(Cog cog, float wheelbase);
float cog_distance_to_left(Cog cog, float track_width);
float cog_distance_to_right(Cog cog, float track_width);

typedef struct {
    float c_drag;
    float frontal_area;
    float wheelbase;
    float front_track_width, rear_track_width;
    float half_cd_a;
    float half_clf_a;
    float half_clr_a;
} Body;

Body body_new(float c_drag, float c_lift_front, float c_lift_rear, float frontal_area,
    float wheelbase, float front_track_width, float rear_track_width);

Vector2f body_air_resistance(const Body* body, float air_density, float longitudinal_velocity);
float body_lift_front(const Body* body, float air_density, float longitudinal_velocity);
float body_lift_rear(const Body* body, float air_density, float longitudinal_velocity);
float yaw_torque(Wheel** wheels, Vector2f* forces, int num_elements);
void set_ackerman_angle(float angle, float wheelbase, Wheel* wl, Wheel* wr);

#endif /* RA_BODY_H */
