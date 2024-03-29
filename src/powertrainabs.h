#ifndef RA_POWERTRAIN_ABS_H
#define RA_POWERTRAIN_ABS_H
#include "common.h"
#include <sys/types.h>

typedef enum {
    /**Tried to use incorrect add function for linking components*/
    RaErrorTaggedInvalid = -1,
    /**Added component more than once.*/
    RaErrorTaggedSame = -2,
    /**Component reaches itself. This will cause infinite recursion and double free.*/
    RaErrorTaggedCyclic = -3,
} RaErrorTaggedComponent;

typedef struct {
    Vector2f velocity_cog;
    float yaw_velocity_cog;
} raVelocities;

typedef struct raTaggedComponent raTaggedComponent;

typedef struct {
    raTaggedComponent* next;
} raComponent;

typedef struct {
    raTaggedComponent* next_left;
    raTaggedComponent* next_right;
} raSplitComponent;

enum raTy { raTyNormal, raTySplit };

union raComponentTypes {
    raSplitComponent split;
    raComponent normal;
};

typedef enum { raInertiaDirectionPrev, raInertiaDirectionNext } raInertiaDirection;

struct raTaggedComponent {
    void* ty;
    raTaggedComponent* prev;
    void (*free_fn)(void* ptr);
    float (*inertia_fn)(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d);
    float (*angular_velocity_fn)(raTaggedComponent* t);
    void (*send_torque_fn)(raTaggedComponent* t, raVelocities v, float torque, float dt);
    void (*receive_torque)(raTaggedComponent* t, float torque, float dt);
    float (*update_angular_velocity)(raTaggedComponent* t);
    float (*external_torque)(raTaggedComponent* t);
    enum raTy comp_ty;
    union raComponentTypes tty;
};

/** Create a component */
raTaggedComponent* ra_tagged_new(void* ty,
    float (*inertia_fn)(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d),
    float (*angular_velocity)(raTaggedComponent* next),
    void (*send_torque_fn)(raTaggedComponent* t, raVelocities v, float torque, float dt),
    void (*receive_torque)(raTaggedComponent* t, float torque, float dt),
    float (*update_angular_velocity)(raTaggedComponent* t),
    float (*external_torque)(raTaggedComponent* t), void (*free_fn)(void* ptr));

/** Create a component with two outputs */
raTaggedComponent* ra_tagged_split_new(void* ty,
    float (*inertia_fn)(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d),
    float (*angular_velocity)(raTaggedComponent* t),
    void (*send_torque_fn)(raTaggedComponent* t, raVelocities v, float torque, float dt),
    void (*receive_torque)(raTaggedComponent* t, float torque, float dt),
    float (*update_angular_velocity)(raTaggedComponent* t),
    float (*external_torque)(raTaggedComponent* t), void (*free_fn)(void* ptr));

void ra_tagged_component_free(raTaggedComponent* c);

/** Get the inner type. e.g: Clutch, Engine. It must be manually casted to it's real type */
void* ra_tagged_component_inner(raTaggedComponent* c);

/* Returns -1 if the component cant be added. For example if the component has two next
 * components*/
RaErrorTaggedComponent ra_tagged_add_next(raTaggedComponent* t, raTaggedComponent* next);

/* Returns -1 if the component cant be added */
RaErrorTaggedComponent ra_tagged_add_next_left(raTaggedComponent* t, raTaggedComponent* next);
/* Returns -1 if the component cant be added */
RaErrorTaggedComponent ra_tagged_add_next_right(raTaggedComponent* t, raTaggedComponent* next);

// Updates all subsystems, this is needed for updating both driven and undriven wheels, which are
// independent systems
typedef struct {
    size_t num_subsystems;
    raTaggedComponent** subsystems;
} raPowertrainSystem;

raPowertrainSystem ra_powertrain_system_from_varags(size_t num_args, ...);
raPowertrainSystem ra_powertrain_system_new(size_t max_subsystems);
void ra_powertrain_system_free(raPowertrainSystem o);

#define RA_POWERTRAIN_SYSTEM(...)                                                                  \
    ra_powertrain_system_from_varags(                                                              \
        sizeof((raTaggedComponent*[]) { __VA_ARGS__ }) / sizeof(raTaggedComponent*), __VA_ARGS__)

float ra_tagged_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d);
float ra_tagged_angular_velocity(raTaggedComponent* t);
void ra_tagged_send_torque(raTaggedComponent* t, float torque, raVelocities v, float dt);
void ra_tagged_receive_torque(raTaggedComponent* t, float torque, float dt);
float ra_tagged_update_angular_velocity(raTaggedComponent* t);
float ra_tagged_external_torque(raTaggedComponent* t);

#endif /* RA_POWERTRAIN_ABS_H */
