#ifndef RA_POWERTRAIN_ABS_H
#define RA_POWERTRAIN_ABS_H
#include <sys/types.h>

typedef struct raTaggedComponent raTaggedComponent;

/** Create a component */
raTaggedComponent* ra_tagged_new(void* ty, float (*inertia)(void* ty), void (*free_fn)(void* ptr));

/** Create a component with two outputs */
raTaggedComponent* ra_tagged_split_new(
    void* ty, float (*inertia)(void* ty), void (*free_fn)(void* ptr));

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

#endif /* RA_POWERTRAIN_ABS_H */
