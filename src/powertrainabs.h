#ifndef RA_POWERTRAIN_ABS_H
#define RA_POWERTRAIN_ABS_H
#include "common.h"
#include <sys/types.h>

typedef struct {
    Vector2f velocity_cog;
    float yaw_velocity_cog;
} raVelocities;

typedef struct raTaggedComponent raTaggedComponent;

typedef struct {
    raTaggedComponent* next;
    void (*send_torque_fn)(void* ty, raVelocities v, float torque, float dt,
        raTaggedComponent* prev, raTaggedComponent* next);
} raComponent;

typedef struct {
    raTaggedComponent* next_left;
    raTaggedComponent* next_right;
    void (*send_torque_fn)(void* ty, raVelocities v, float torque, float dt,
        raTaggedComponent* prev, raTaggedComponent* next_left, raTaggedComponent* next_right);
} raSplitComponent;

enum raTy { raTyNormal, raTySplit };

union raComponentTypes {
    raSplitComponent split;
    raComponent normal;
};

struct raTaggedComponent {
    void* ty;
    raTaggedComponent* prev;
    void (*free_fn)(void* ptr);
    float (*inertia_fn)(void* ty);
    float (*angular_velocity_fn)(raTaggedComponent* t);
    enum raTy comp_ty;
    union raComponentTypes tty;
};

/** Create a component */
raTaggedComponent* ra_tagged_new(void* ty, float (*inertia)(void* ty),
    float (*angular_velocity)(raTaggedComponent* next),
    void (*send_torque_fn)(void* ty, raVelocities v, float torque, float dt,
        raTaggedComponent* prev, raTaggedComponent* next),
    void (*free_fn)(void* ptr));

/** Create a component with two outputs */
raTaggedComponent* ra_tagged_split_new(void* ty, float (*inertia)(void* ty),
    float (*angular_velocity)(raTaggedComponent* t),
    // TODO: Simplify with TaggedComp * t
    void (*send_torque_fn)(void* ty, raVelocities v, float torque, float dt,
        raTaggedComponent* prev, raTaggedComponent* next_left, raTaggedComponent* next_right),
    void (*free_fn)(void* ptr));

void ra_tagged_component_free(raTaggedComponent* c);

/** Get the inner type. e.g: Clutch, Engine. It must be manually casted to it's real type */
void* ra_tagged_component_inner(raTaggedComponent* c);

/* Returns -1 if the component cant be added. For example if the component has two next
 * components*/
int ra_tagged_add_next(raTaggedComponent* t, raTaggedComponent* next);

/* Returns -1 if the componenet cant be added */
int ra_tagged_add_next_left(raTaggedComponent* t, raTaggedComponent* next);
/* Returns -1 if the componenet cant be added */
int ra_tagged_add_next_right(raTaggedComponent* t, raTaggedComponent* next);

// Updates all subsystems, this is needed for updating both driven and undriven wheels, which are
// independent systems
typedef struct {
    size_t num_subsystems;
    raTaggedComponent** subsystems;
} raOverviewSystem;

raOverviewSystem ra_overwiew_system_new(size_t max_subsystems);
void ra_overwiew_system_free(raOverviewSystem o);

void ra_tagged_send_torque(raTaggedComponent* t, float torque, raVelocities v, float dt);

#endif /* RA_POWERTRAIN_ABS_H */