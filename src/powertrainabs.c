#include "powertrainabs.h"
#include <assert.h>
#include <stdlib.h>

void* ra_tagged_component_inner(raTaggedComponent* c) { return c->ty; }

raTaggedComponent* ra_tagged_new(void* ty, float (*inertia)(void* ty),
    float (*angular_velocity)(raTaggedComponent* t),
    void (*send_torque_fn)(void* ty, raVelocities v, float torque, float dt,
        raTaggedComponent* prev, raTaggedComponent* next),
    void (*free_fn)(void* ptr))
{
    assert(ty != NULL);
    assert(free_fn != NULL);

    raTaggedComponent* t = malloc(sizeof *t);
    if (t == NULL) {
        abort();
    }
    t->comp_ty = raTyNormal;
    t->ty = ty;
    t->free_fn = free_fn;
    t->inertia_fn = inertia;
    t->angular_velocity_fn = angular_velocity, t->prev = NULL;

    t->tty.normal = (raComponent) {
        .next = NULL,
        .send_torque_fn = send_torque_fn,
    };

    return t;
}

int ra_tagged_add_next(raTaggedComponent* t, raTaggedComponent* next)
{
    if (t->comp_ty != raTyNormal) {
        return -1;
    } else {
        t->tty.normal.next = next;
        next->prev = t;
    }

    return 0;
}

raTaggedComponent* ra_tagged_split_new(void* ty, float (*inertia)(void* ty),
    float (*angular_velocity)(raTaggedComponent* t),
    void (*send_torque_fn)(void* ty, raVelocities v, float torque, float dt,
        raTaggedComponent* prev, raTaggedComponent* next_left, raTaggedComponent* next_right),
    void (*free_fn)(void* ptr))
{
    raTaggedComponent* t = malloc(sizeof *t);
    if (t == NULL) {
        abort();
    }
    t->comp_ty = raTySplit;
    t->ty = ty;
    t->free_fn = free_fn;
    t->inertia_fn = inertia;
    t->angular_velocity_fn = angular_velocity, t->prev = NULL;

    t->tty.split = (raSplitComponent) {
        .next_left = NULL,
        .next_right = NULL,
        .send_torque_fn = send_torque_fn,
    };

    return t;
}

int ra_tagged_add_next_left(raTaggedComponent* t, raTaggedComponent* next)
{
    if (t->comp_ty != raTySplit) {
        return -1;
    } else {
        t->tty.split.next_left = next;
        next->prev = t;
    }

    return 0;
}

int ra_tagged_add_next_right(raTaggedComponent* t, raTaggedComponent* next)
{
    if (t->comp_ty != raTySplit) {
        return -1;
    } else {
        t->tty.split.next_right = next;
        next->prev = t;
    }

    return 0;
}

static void ra_free_split_component(raSplitComponent* c);
static void ra_free_component(raComponent* c);

void ra_tagged_component_free(raTaggedComponent* c)
{
    if (c->comp_ty == raTyNormal) {
        ra_free_component(&c->tty.normal);
    } else if (c->comp_ty == raTySplit) {
        ra_free_split_component(&c->tty.split);
    } else {
        // This should be unreachable
        abort();
    }

    c->free_fn(c->ty);

    free(c);
}

static void ra_free_component(raComponent* c)
{
    if (c->next != NULL) {
        ra_tagged_component_free(c->next);
    }
}

static void ra_free_split_component(raSplitComponent* c)
{
    if (c->next_left != NULL) {
        ra_tagged_component_free(c->next_left);
    }

    if (c->next_right != NULL) {
        ra_tagged_component_free(c->next_right);
    }
}

// TODO: Growable list
raOverviewSystem ra_overwiew_system_new(size_t max_subsystems)
{
    raTaggedComponent** subsystems = malloc(max_subsystems * sizeof *subsystems);
    if (subsystems == NULL) {
        abort();
    }

    return (raOverviewSystem) {
        .num_subsystems = max_subsystems,
        .subsystems = subsystems,
    };
}

void ra_overwiew_system_free(raOverviewSystem o)
{
    for (size_t i = 0; i < o.num_subsystems; i++) {
        ra_tagged_component_free(o.subsystems[i]);
    }
    free(o.subsystems);

    o.num_subsystems = 0;
    o.subsystems = NULL;
}

void ra_tagged_send_torque(raTaggedComponent* t, float torque, raVelocities v, float dt)
{
    if (t->comp_ty == raTyNormal) {
        raComponent ty = t->tty.normal;
        ty.send_torque_fn(t->ty, v, torque, dt, t->prev, ty.next);
    } else if (t->comp_ty == raTySplit) {
        raSplitComponent ty = t->tty.split;
        ty.send_torque_fn(t->ty, v, torque, dt, t->prev, ty.next_left, ty.next_right);
    }
}
