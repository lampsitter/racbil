#include "powertrain.h"
#include "common.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static float normal_ext_torque(raTaggedComponent* t)
{
    return ra_tagged_external_torque(t->tty.normal.next);
}

RevLimiterHard rev_limiter_hard_new(
    float activation_angular_velocity, float deactivation_angular_velocity)
{
    return (RevLimiterHard) {
        .activation_angular_velocity = activation_angular_velocity,
        .deactivation_angular_velocity = deactivation_angular_velocity,
        .is_active = false,
    };
}

float rev_limiter_hard(RevLimiterHard* r, Engine* e, float throttle_pos)
{
    if (r->is_active && e->angular_velocity < r->deactivation_angular_velocity) {
        r->is_active = false;
    } else if (!r->is_active && e->angular_velocity >= r->activation_angular_velocity) {
        r->is_active = true;
    }

    if (r->is_active) {
        return 0.0;
    } else {
        return throttle_pos;
    }
}

Engine* engine_new(float inertia, Table torque_map)
{
    Engine* engine = malloc(sizeof *engine);
    if (engine == NULL) {
        exit(EXIT_FAILURE);
    }

    engine->torque_map = torque_map;
    engine->angular_velocity = 0.0f;
    engine->inertia = inertia;
    return engine;
}

void engine_free(Engine* engine)
{
    table_free(&engine->torque_map);
    free(engine);
}

float engine_torque(Engine* engine, float throttle_pos)
{
    return table_lookup(&engine->torque_map, throttle_pos, engine->angular_velocity);
}

float engine_demanded_torque(
    Engine* engine, float desired_velocity, float external_inertia, float dt)
{
    float vel_diff = desired_velocity - engine->angular_velocity;
    float torque = (vel_diff / dt) * (engine->inertia + external_inertia);

    float min_torque = engine_torque(engine, 0.0);
    float max_torque = engine_torque(engine, 1.0);

    return fmaxf(min_torque, fminf(torque, max_torque));
}

void engine_set_angular_velocity(Engine* engine, AngularVelocity velocity)
{
    engine->angular_velocity = fmaxf(velocity, 0.0);
}

static float engine_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d)
{
    SUPPRESS_UNUSED(prev);
    float inertia = ((Engine*)t->ty)->inertia;
    if (d == raInertiaDirectionNext) {
        return inertia + ra_tagged_inertia(t->tty.normal.next, t, d);
    } else {
        return inertia;
    }
}

static float engine_angular_vel(raTaggedComponent* t) { return ((Engine*)t->ty)->angular_velocity; }

static void engine_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    // Ignore engine torque for now since it is responsible for producing torque
    ra_tagged_send_torque(t->tty.normal.next, torque, v, dt);
}

static void engine_receive_torque(raTaggedComponent* t, float torque, float dt)
{
    float a = torque / ra_tagged_inertia(t, t, raInertiaDirectionPrev);
    Engine* e = (Engine*)t->ty;
    engine_set_angular_velocity(e, e->angular_velocity + integrate(a, dt));
}

static float engine_update_angular_velocity(raTaggedComponent* t)
{
    raTaggedComponent* c = t->tty.normal.next;
    Engine* e = (Engine*)t->ty;
    engine_set_angular_velocity(e, ra_tagged_update_angular_velocity(c));
    return e->angular_velocity;
}

raTaggedComponent* ra_tag_engine(Engine* engine)
{
    return ra_tagged_new(engine, engine_inertia, engine_angular_vel, engine_send_torque,
        engine_receive_torque, engine_update_angular_velocity, normal_ext_torque,
        (void (*)(void*))engine_free);
}

float idle_engine_torque(
    float idle_velocity, Engine* engine, float engine_torque, bool is_disconnected, float dt)
{
    if (engine->angular_velocity < idle_velocity && is_disconnected) {
        float idle_torque = engine_demanded_torque(engine, idle_velocity, 0.0, dt);
        // Only override user throttle if it does not provide enough torque
        if (engine_torque < idle_torque) {
            return idle_torque;
        } else {
            return engine_torque;
        }
    } else {
        return engine_torque;
    }
}

Differential* differential_new(float ratio, float inertia, DiffType ty)
{
    Differential* diff = malloc(sizeof *diff);
    if (diff == NULL) {
        exit(EXIT_FAILURE);
    }
    diff->ratio = ratio;
    diff->inertia = inertia;
    diff->ty = ty;
    return diff;
}

static void differential_spool(float torque, float reaction_torque_left,
    float reaction_torque_right, float* output_left_torque, float* output_right_torque)
{
    float bias = (reaction_torque_right - reaction_torque_left) * 0.5;
    *output_left_torque = torque * 0.5 + bias;
    *output_right_torque = torque * 0.5 - bias;
}

static void differential_open(float torque, float* output_left_torque, float* output_right_torque)
{
    *output_left_torque = torque * 0.5;
    *output_right_torque = torque * 0.5;
}

void differential_torque(Differential* diff, float input_torque, float reaction_torque_left,
    float reaction_torque_right, float* output_left_torque, float* output_right_torque)
{
    float torque = input_torque * diff->ratio;
    if (diff->ty == DiffTypeOpen) {
        differential_open(torque, output_left_torque, output_right_torque);
    } else if (diff->ty == DiffTypeLocked) {
        differential_spool(torque, reaction_torque_left, reaction_torque_right, output_left_torque,
            output_right_torque);
    } else {
        fprintf(stderr, "Unkown diff type");
        exit(EXIT_FAILURE);
    }
}

float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity)
{
    return (left_angular_velocity + right_angular_velocity) * 0.5 * diff->ratio;
}

static float diff_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d)
{
    Differential* diff = (Differential*)t->ty;
    raTaggedComponent* next_left = t->tty.split.next_left;
    raTaggedComponent* next_right = t->tty.split.next_right;

    float inertia = diff->inertia;

    if (d == raInertiaDirectionNext) {
        return inertia + ra_tagged_inertia(next_left, t, d) + ra_tagged_inertia(next_right, t, d);
    } else {
        float inertia_prev = ra_tagged_inertia(t->prev, t, d) + inertia;
        if (diff->ty == DiffTypeLocked) {
            if (prev == next_left) {
                return inertia_prev + ra_tagged_inertia(next_right, t, raInertiaDirectionNext);
            } else if (prev == next_right) {
                return inertia_prev + ra_tagged_inertia(next_left, t, raInertiaDirectionNext);
            } else {
                fprintf(stderr, "Incorrect previous inertia component");
                exit(EXIT_FAILURE);
            }
        } else {
            return inertia_prev;
        }
    }
}

static float diff_angular_vel(raTaggedComponent* t)
{
    raSplitComponent s = t->tty.split;
    return differential_velocity(((Differential*)t->ty), ra_tagged_angular_velocity(s.next_left),
        ra_tagged_angular_velocity(s.next_right));
}

static void diff_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    raSplitComponent s = t->tty.split;

    float react_left = ra_tagged_external_torque(s.next_left);
    float react_right = ra_tagged_external_torque(s.next_right);

    float torque_left, torque_right;
    differential_torque(
        (Differential*)t->ty, torque, react_left, react_right, &torque_left, &torque_right);

    ra_tagged_send_torque(s.next_left, torque_left, v, dt);
    ra_tagged_send_torque(s.next_right, torque_right, v, dt);
}

static float diff_update_angular_velocity(raTaggedComponent* t)
{
    raSplitComponent s = t->tty.split;
    return differential_velocity(((Differential*)t->ty),
        ra_tagged_update_angular_velocity(s.next_left),
        ra_tagged_update_angular_velocity(s.next_right));
}

static float diff_ext_torque(raTaggedComponent* t)
{
    raSplitComponent s = t->tty.split;
    return ra_tagged_external_torque(s.next_left) + ra_tagged_external_torque(s.next_right);
}

raTaggedComponent* ra_tag_differential(Differential* diff)
{
    return ra_tagged_split_new(diff, diff_inertia, diff_angular_vel, diff_send_torque, NULL,
        diff_update_angular_velocity, diff_ext_torque, free);
}

Gearbox* gearbox_new(VecFloat ratios, VecFloat inertias)
{
    Gearbox* gb = malloc(sizeof *gb);
    if (gb == NULL) {
        exit(EXIT_FAILURE);
    }

    gb->ratios = ratios;
    gb->inertias = inertias;
    gb->curr_gear = 0;
    gb->input_angular_velocity = 0.0;
    return gb;
}

void gearbox_free(Gearbox* gb)
{
    vec_free(&gb->ratios);
    vec_free(&gb->inertias);
    free(gb);
}

static float gearbox_current_gear(const Gearbox* gb, const VecFloat* vec)
{
    int curr_gear = gb->curr_gear;
    if (curr_gear < 0) {
        return vec->elements[0];
    } else if (curr_gear == 0) {
        return 0;
    } else {
        return vec->elements[curr_gear];
    }
}

static float gearbox_ratio(const Gearbox* gb) { return gearbox_current_gear(gb, &gb->ratios); }

float gearbox_inertia(const Gearbox* gb) { return gearbox_current_gear(gb, &gb->inertias); }

float gearbox_angular_velocity_in(Gearbox* gb, float angular_velocity_out)
{
    if (gb->curr_gear != 0) {
        gb->input_angular_velocity = gearbox_ratio(gb) * angular_velocity_out;
    }
    return gb->input_angular_velocity;
}

float gearbox_torque_out(const Gearbox* gb, float torque_in)
{
    return gearbox_ratio(gb) * torque_in;
}

static void gearbox_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    float gb_torque = gearbox_torque_out((Gearbox*)t->ty, torque);
    ra_tagged_send_torque(t->tty.normal.next, gb_torque, v, dt);
}

static float gb_update_angular_velocity(raTaggedComponent* t)
{
    raTaggedComponent* next = t->tty.normal.next;
    return gearbox_angular_velocity_in((Gearbox*)t->ty, ra_tagged_update_angular_velocity(next));
}

static float gb_angular_vel(raTaggedComponent* t)
{
    return gearbox_angular_velocity_in(
        (Gearbox*)t->ty, ra_tagged_angular_velocity(t->tty.normal.next));
}

static float gb_tagged_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d)
{
    SUPPRESS_UNUSED(prev);
    float inertia = gearbox_inertia((Gearbox*)t->ty);
    if (d == raInertiaDirectionNext) {
        raTaggedComponent* next = t->tty.normal.next;
        return inertia + ra_tagged_inertia(next, t, d);
    } else {
        return inertia + ra_tagged_inertia(t->prev, t, d);
    }
}

raTaggedComponent* ra_tag_gearbox(Gearbox* gb)
{
    return ra_tagged_new(gb, gb_tagged_inertia, gb_angular_vel, gearbox_send_torque, NULL,
        gb_update_angular_velocity, normal_ext_torque, (void (*)(void*))gearbox_free);
}

// Calculate max normal force from desired torque caracheristics of the clutch.
Clutch* clutch_with_torque(
    float* max_normal_force, float max_static_torque, float max_kinetic_torque)
{
    *max_normal_force = max_static_torque;
    float static_coefficient = 1.0f;
    float kinetic_coefficient = max_kinetic_torque / *max_normal_force;

    Clutch* c = malloc(sizeof *c);
    if (c == NULL) {
        exit(EXIT_FAILURE);
    }

    c->static_coefficient = static_coefficient;
    c->kinetic_coefficient = kinetic_coefficient;
    c->velocity_threshold = 1.0;
    c->torque_sensitivity = 2.0;
    c->is_locked = false;
    return c;
}

static ClutchTagged* clutch_tagged_new(Clutch* c)
{
    ClutchTagged* ct = malloc(sizeof *ct);
    if (ct == NULL) {
        exit(EXIT_FAILURE);
    }
    ct->c = c;
    ct->curr_normal_force = 0.0f;
    return ct;
}

static void clutch_tagged_free(void* ty)
{
    ClutchTagged* t = ((ClutchTagged*)ty);
    free(t->c);
    free(t);
    ty = NULL;
}

static inline float tanh_friction(float torque, AngularVelocity vel_diff, float transition)
{
    return torque * tanh(2.0 * (vel_diff / transition));
}

void clutch_torque_out(Clutch* clutch, float torque_in, float normal_force,
    AngularVelocity left_vel, AngularVelocity right_vel, float* torque_left, float* torque_right)
{
    float vel_diff = left_vel - right_vel;
    // The whole formula is technically (2/3) * effective_radius * normal_force *
    // static_coefficient. For simplisity (2/3) * effective_radius is dropped, reducing the amount
    // of variables that need to be configured. It can be compensated for by including it in the
    // input normal force if necessary.
    float static_torque = clutch->static_coefficient * normal_force;
    if (fabsf(vel_diff) < clutch->velocity_threshold && fabsf(torque_in) <= static_torque) {
        // Locked
        *torque_left = torque_in;
        *torque_right = torque_in;
        clutch->is_locked = true;
    } else {
        // Slipping
        float kinetic_torque = clutch->kinetic_coefficient * normal_force;
        float out_torque = tanh_friction(kinetic_torque, vel_diff, clutch->torque_sensitivity);
        *torque_left = torque_in - out_torque;
        *torque_right = out_torque;
        clutch->is_locked = false;
    }
}

static void clutch_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    ClutchTagged* ct = (ClutchTagged*)t->ty;
    float torque_left, torque_right;

    float left_vel = ra_tagged_angular_velocity(t->prev);
    float right_vel = ra_tagged_angular_velocity(t->tty.normal.next);
    clutch_torque_out(
        ct->c, torque, ct->curr_normal_force, left_vel, right_vel, &torque_left, &torque_right);

    ra_tagged_send_torque(t->tty.normal.next, torque_right, v, dt);
    if (!ct->c->is_locked) {
        ra_tagged_receive_torque(t->prev, torque_left, dt);
    }
}

static float clutch_inertia(raTaggedComponent* t, raTaggedComponent* prev, raInertiaDirection d)
{
    SUPPRESS_UNUSED(prev);
    ClutchTagged* ct = (ClutchTagged*)t->ty;
    if (!ct->c->is_locked) {
        // Each shaft rotates independently of each other
        return 0.0;
    } else if (d == raInertiaDirectionNext) {
        return ra_tagged_inertia(t->tty.normal.next, t, d);
    } else {
        return ra_tagged_inertia(t->prev, t, d);
    }
}

static float clutch_update_angular_velocity(raTaggedComponent* t)
{
    if (!((ClutchTagged*)t->ty)->c->is_locked) {
        // By returning the current angular velocity the prev's body velocity will
        // stay the same, which is important as we have already updated it in clutch_send_torque.
        return ra_tagged_angular_velocity(t->prev);
    } else {
        // There is nothing wrong with sending torque up instead, but by doing it this way we can
        // force the two rotating shafts to have the same velocity. Meaning that is_locked is now a
        // correct assumption.
        return ra_tagged_update_angular_velocity(t->tty.normal.next);
    }
}

static float clutch_ext_torque(raTaggedComponent* t)
{
    ClutchTagged* ct = (ClutchTagged*)t->ty;
    if (!ct->c->is_locked) {
        // TODO:
        exit(EXIT_FAILURE);
    } else {
        return ra_tagged_external_torque(t->tty.normal.next);
    }
}

static float clutch_angular_velocity(raTaggedComponent* t)
{
    // FIXME: What about when the clutch is disconnected?
    return ra_tagged_angular_velocity(t->tty.normal.next);
}

raTaggedComponent* ra_tag_clutch(Clutch* c)
{
    return ra_tagged_new(clutch_tagged_new(c), clutch_inertia, clutch_angular_velocity,
        clutch_send_torque, NULL, clutch_update_angular_velocity, clutch_ext_torque,
        clutch_tagged_free);
}
