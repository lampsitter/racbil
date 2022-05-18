#ifndef RA_WHEEL_H
#define RA_WHEEL_H
#include "common.h"
#include "powertrainabs.h"
#include "tiremodel.h"

typedef enum { WheelDirectionForward, WheelDirectionReverse } WheelDirection;

typedef struct {
    Vector2f hub_velocity;
    Vector2f position;
    float min_speed;

    float angle;
    AngularVelocity angular_velocity;
    float inertia;
    float effective_radius;
    /**Only used for telemtry*/
    float input_torque;
    float reaction_torque;
    /**Torque that is not applied by the powertrain, such as brake torque*/
    float external_torque;
} Wheel;

Wheel* wheel_new(float inertia, float radius, Vector2f position, float min_speed);
raTaggedComponent* ra_tag_wheel(Wheel* w);
/** Changes the rotation direction of the wheel. It changes direction regardless whether
 * the current hub velocity is larger than min_speed. Upon direction change the hub_velocity
 * is always set to min_speed*/
void wheel_change_direction(Wheel* w, WheelDirection d);
/** Same as `wheel_change_direction` but only changes direction when hub_velocity == min_speed*/
void wheel_try_change_direction(Wheel* w, WheelDirection d);

Vector2f wheel_slip(const Wheel* wheel);
void wheel_update(Wheel* wheel, Vector2f velocity_cog, float yaw_angular_velocity_cog,
    float external_inertia, float torque, float dt);

/**
 * wheel_update must be called before this function
 */
Vector2f wheel_force(Wheel* wheel, TireModel* model, float normal_force, float friction_coefficent);

#endif /* RA_WHEEL_H */
