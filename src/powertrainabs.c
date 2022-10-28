#include "powertrainabs.h"
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>

void* ra_tagged_component_inner(raTaggedComponent* c) { return c->ty; }

raTaggedComponent* ra_tagged_new(void* ty,
    float (*inertia_fn)(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d),
    float (*angular_velocity)(raTaggedComponent* t),
    void (*send_torque_fn)(raTaggedComponent* t, raVelocities v, float torque, float dt),
    void (*receive_torque)(raTaggedComponent* t, float torque, float dt),
    float (*update_angular_velocity)(raTaggedComponent* t),
    float (*external_torque)(raTaggedComponent* t), void (*free_fn)(void* ptr))
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
    t->inertia_fn = inertia_fn;
    t->angular_velocity_fn = angular_velocity;
    t->send_torque_fn = send_torque_fn, t->prev = NULL;
    t->receive_torque = receive_torque;
    t->update_angular_velocity = update_angular_velocity;
    t->external_torque = external_torque;

    t->tty.normal = (raComponent) {
        .next = NULL,
    };

    return t;
}

static bool has_cycle(raTaggedComponent* c, raTaggedComponent* prev)
{
    if (prev == NULL) {
        return false;
    } else if (c == prev) {
        return true;
    } else {
        return has_cycle(c, prev->prev);
    }
}

RaErrorTaggedComponent ra_tagged_add_next(raTaggedComponent* t, raTaggedComponent* next)
{

    if (t->comp_ty != raTyNormal) {
        return RaErrorTaggedInvalid;
    } else {
        raTaggedComponent* curr_next = t->tty.normal.next;
        raTaggedComponent* curr_prev = t->prev;
        t->tty.normal.next = next;
        next->prev = t;

        // By disallowing cycles we prevent double free and infinite recursion
        if (has_cycle(t, t->prev)) {
            // restore to before the component was added
            t->tty.normal.next = curr_next;
            next->prev = curr_prev;
            return RaErrorTaggedCyclic;
        }
    }

    return 0;
}

raTaggedComponent* ra_tagged_split_new(void* ty,
    float (*inertia_fn)(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d),
    float (*angular_velocity)(raTaggedComponent* t),
    void (*send_torque_fn)(raTaggedComponent* t, raVelocities v, float torque, float dt),
    void (*receive_torque)(raTaggedComponent* t, float torque, float dt),
    float (*update_angular_velocity)(raTaggedComponent* t),
    float (*external_torque)(raTaggedComponent* t), void (*free_fn)(void* ptr))
{
    raTaggedComponent* t = malloc(sizeof *t);
    if (t == NULL) {
        abort();
    }
    t->comp_ty = raTySplit;
    t->ty = ty;
    t->free_fn = free_fn;
    t->inertia_fn = inertia_fn;
    t->angular_velocity_fn = angular_velocity;
    t->send_torque_fn = send_torque_fn, t->prev = NULL;
    t->receive_torque = receive_torque;
    t->update_angular_velocity = update_angular_velocity;
    t->external_torque = external_torque;

    t->tty.split = (raSplitComponent) {
        .next_left = NULL,
        .next_right = NULL,
    };

    return t;
}

RaErrorTaggedComponent ra_tagged_add_next_left(raTaggedComponent* t, raTaggedComponent* next)
{
    if (t->comp_ty != raTySplit) {
        return RaErrorTaggedInvalid;
    } else {
        if (t->tty.split.next_right == next) {
            return RaErrorTaggedSame;
        } else {
            raTaggedComponent* curr_next = t->tty.split.next_left;
            raTaggedComponent* curr_prev = t->prev;
            t->tty.split.next_left = next;
            next->prev = t;
            if (has_cycle(t, t->prev)) {
                t->tty.split.next_left = curr_next;
                next->prev = curr_prev;
                return RaErrorTaggedCyclic;
            }
        }
    }

    return 0;
}

RaErrorTaggedComponent ra_tagged_add_next_right(raTaggedComponent* t, raTaggedComponent* next)
{
    if (t->comp_ty != raTySplit) {
        return RaErrorTaggedInvalid;
    } else {
        if (t->tty.split.next_left == next) {
            return RaErrorTaggedSame;
        } else {
            raTaggedComponent* curr_next = t->tty.split.next_right;
            raTaggedComponent* curr_prev = t->prev;
            t->tty.split.next_right = next;
            next->prev = t;

            if (has_cycle(t, t->prev)) {
                t->tty.split.next_right = curr_next;
                t->prev = curr_prev;
                return RaErrorTaggedCyclic;
            }
        }
    }

    return 0;
}

static void ra_free_split_component(raSplitComponent* c);
static void ra_free_component(raComponent* c);

void ra_tagged_component_free(raTaggedComponent* c)
{
    if (c != NULL) {
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
raPowertrainSystem ra_powertrain_system_new(size_t max_subsystems)
{
    raTaggedComponent** subsystems = malloc(max_subsystems * sizeof *subsystems);
    if (subsystems == NULL) {
        abort();
    }

    return (raPowertrainSystem) {
        .num_subsystems = max_subsystems,
        .subsystems = subsystems,
    };
}

raPowertrainSystem ra_powertrain_system_from_varags(size_t num_args, ...)
{
    va_list ap;
    va_start(ap, num_args);

    raPowertrainSystem s = ra_powertrain_system_new(num_args);

    for (size_t i = 0; i < num_args; ++i) {
        s.subsystems[i] = va_arg(ap, raTaggedComponent*);
    }

    va_end(ap);

    return s;
}

void ra_powertrain_system_free(raPowertrainSystem o)
{
    for (size_t i = 0; i < o.num_subsystems; i++) {
        ra_tagged_component_free(o.subsystems[i]);
    }
    free(o.subsystems);

    o.num_subsystems = 0;
    o.subsystems = NULL;
}

float ra_tagged_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d)
{
    // TODO: Check for null on all relevant prev and next invocations
    if (t == NULL) {
        return 0.0f;
    } else {
        return t->inertia_fn(t, prev, d);
    }
}

float ra_tagged_angular_velocity(raTaggedComponent* t) { return t->angular_velocity_fn(t); }
void ra_tagged_send_torque(raTaggedComponent* t, float torque, raVelocities v, float dt)
{
    t->send_torque_fn(t, v, torque, dt);
}

void ra_tagged_receive_torque(raTaggedComponent* t, float torque, float dt)
{
    t->receive_torque(t, torque, dt);
}

float ra_tagged_update_angular_velocity(raTaggedComponent* t)
{
    return t->update_angular_velocity(t);
}

float ra_tagged_external_torque(raTaggedComponent* t)
{
    if (t == NULL) {
        return 0.0;
    } else {
        return t->external_torque(t);
    }
}
