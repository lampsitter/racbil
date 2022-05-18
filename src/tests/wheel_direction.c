#include "../wheel.h"
#include <assert.h>
#include <stdlib.h>

int main(void)
{
    Wheel* w = wheel_new(0.6, 0.344, (Vector2f) { .x = 0.0, .y = 0.0 }, 0.01);

    wheel_change_direction(w, WheelDirectionReverse);
    assert(w->hub_velocity.x == -w->min_speed);
    assert(w->angular_velocity == -w->min_speed / w->effective_radius);
    wheel_change_direction(w, WheelDirectionForward);
    assert(w->hub_velocity.x == w->min_speed);
    assert(w->angular_velocity == w->min_speed / w->effective_radius);

    wheel_try_change_direction(w, WheelDirectionReverse);
    assert(w->hub_velocity.x == -w->min_speed);
    assert(w->angular_velocity == -w->min_speed / w->effective_radius);

    w->hub_velocity.x = -3.5;
    wheel_try_change_direction(w, WheelDirectionForward);
    assert(w->hub_velocity.x == -3.5);
    assert(w->angular_velocity == -w->min_speed / w->effective_radius);

    w->hub_velocity.x = -w->min_speed;
    w->angular_velocity = -100.0;
    wheel_try_change_direction(w, WheelDirectionForward);
    assert(w->hub_velocity.x == -w->min_speed);
    assert(w->angular_velocity == -100.0);

    free(w);
}
