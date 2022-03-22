#include "body.h"
#include <assert.h>

Cog cog_from_distribution(float ratio_front, float height, float wheelbase)
{
    assert(ratio_front >= 0.0 && ratio_front <= 1.0);
    // FIXME: It is not clear that y is in relation to the middle of the car.
    return (Cog) { .x = ratio_front * wheelbase, .y = 0.0, .z = height };
}

float cog_distance_to_front(Cog cog) { return cog.x; }

float cog_distance_to_rear(Cog cog, float wheelbase) { return cog.x - wheelbase; }

float cog_distance_to_left(Cog cog, float track_width) { return track_width * 0.5 - cog.y; }

float cog_distance_to_right(Cog cog, float track_width) { return cog.y - track_width * 0.5; }

Body body_new(float i_zz, float c_drag, float frontal_area, float wheelbase,
    float front_track_width, float rear_track_width)
{
    return (Body) { .i_zz = i_zz,
        .c_drag = c_drag,
        .frontal_area = frontal_area,
        .half_cd_a = 0.5 * c_drag * frontal_area,
        .wheelbase = wheelbase,
        .front_track_width = front_track_width,
        .rear_track_width = rear_track_width };
}

float body_air_resistance(const Body* body, float air_density, float longitudinal_velocity)
{
    float long_sq = longitudinal_velocity * longitudinal_velocity;
    return -(air_density * body->half_cd_a * long_sq) * signum(longitudinal_velocity);
}

float yaw_torque(Wheel* fl, Wheel* fr, Wheel* rl, Wheel* rr, Vector2f ffl, Vector2f ffr,
    Vector2f frl, Vector2f frr)
{
    float y = ffl.y * fl->position.x + ffr.y * fr->position.x + frl.y * rl->position.x
        + frr.y * rr->position.x;

    float x = ffl.x * fl->position.y + ffr.x * fr->position.y + frl.x * rl->position.y
        + frr.x * rr->position.y;

    return y + x;
}

void set_ackerman_angle(float angle, float wheelbase, Wheel* wl, Wheel* wr)
{
    float a_tan = tanf(angle);

    wl->angle = atanf((wheelbase * a_tan) / (wheelbase + a_tan * wl->position.y));
    wr->angle = atanf((wheelbase * a_tan) / (wheelbase + a_tan * wr->position.y));
}
