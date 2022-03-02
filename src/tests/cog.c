#include "../body.h"
#include "../common.h"
#include <assert.h>

int main(void)
{
    Body body = body_new(0.36, 1.9, 3.6f, 1.47f, 1.475f);
    Cog cog = cog_from_distribution(0.55, 0.4, body.wheelbase);

    Vector2f fl_pos = (Vector2f) { .x = cog_distance_to_front(cog),
        .y = cog_distance_to_left(cog, body.front_track_width) };

    Vector2f fr_pos = (Vector2f) { .x = cog_distance_to_front(cog),
        .y = cog_distance_to_right(cog, body.front_track_width) };

    Vector2f rl_pos = (Vector2f) { .x = cog_distance_to_rear(cog, body.wheelbase),
        .y = cog_distance_to_left(cog, body.rear_track_width) };

    Vector2f rr_pos = (Vector2f) { .x = cog_distance_to_rear(cog, body.wheelbase),
        .y = cog_distance_to_right(cog, body.rear_track_width) };

    assert(fl_pos.x > 0.0 && fl_pos.y > 0.0);
    assert(fr_pos.x > 0.0 && fr_pos.y < 0.0);
    assert(rl_pos.x < 0.0 && rl_pos.y > 0.0);
    assert(rr_pos.x < 0.0 && rr_pos.y < 0.0);
}
