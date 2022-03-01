#ifndef RA_BODY_H
#define RA_BODY_H
#include "common.h"

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
} Body;

Body body_new(float c_drag, float frontal_area, float wheelbase, float front_track_width,
    float rear_track_width);

float body_air_resistance(const Body* body, float air_density, float longitudinal_velocity);
#endif /* RA_BODY_H */
