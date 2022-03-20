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
    float i_zz;
} Body;

Body body_new(float i_zz, float c_drag, float frontal_area, float wheelbase,
    float front_track_width, float rear_track_width);

float body_air_resistance(const Body* body, float air_density, float longitudinal_velocity);
float yaw_torque(Wheel* fl, Wheel* fr, Wheel* rl, Wheel* rr, Vector2f ffl, Vector2f ffr,
    Vector2f frl, Vector2f frr);
#endif /* RA_BODY_H */
