#include "powertrain.h"
#include "common.h"
#include <stdbool.h>
#include <stdlib.h>

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

static float engine_inertia(void* ty) { return ((Engine*)ty)->inertia; }

static float engine_angular_vel(raTaggedComponent* t) { return ((Engine*)t->ty)->angular_velocity; }

static void engine_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    // Ignore engine torque for now since it is responsible for producing torque
    ra_tagged_send_torque(t->tty.normal.next, torque, v, dt);
}

raTaggedComponent* ra_tag_engine(Engine* engine)
{
    return ra_tagged_new(engine, engine_inertia, engine_angular_vel, engine_send_torque,
        (void (*)(void*))engine_free);
}

float engine_torque(Engine* engine, float throttle_pos)
{
    return table_lookup(&engine->torque_map, throttle_pos, engine->angular_velocity);
}

void engine_set_angular_velocity(Engine* engine, AngularVelocity velocity)
{
    engine->angular_velocity = fmaxf(velocity, 0.0);
}

Differential* differential_new(float ratio, float inertia)
{
    Differential* diff = malloc(sizeof *diff);
    diff->ratio = ratio;
    diff->inertia = inertia;
    return diff;
}

void differential_torque(
    Differential* diff, float input_torque, float* output_left_torque, float* output_right_torque)
{
    float torque = input_torque * diff->ratio;
    // Simple open diff for now
    *output_left_torque = torque * 0.5;
    *output_right_torque = torque * 0.5;
}

float differential_velocity(
    Differential* diff, float left_angular_velocity, float right_angular_velocity)
{
    return (left_angular_velocity + right_angular_velocity) * 0.5 * diff->ratio;
}

static float diff_inertia(void* ty) { return ((Differential*)ty)->inertia; }

static float diff_angular_vel(raTaggedComponent* t)
{
    raSplitComponent s = t->tty.split;
    return differential_velocity(((Differential*)t->ty),
        s.next_left->angular_velocity_fn(s.next_left),
        s.next_right->angular_velocity_fn(s.next_right));
}

static void diff_send_torque(raTaggedComponent* t, raVelocities v, float torque, float dt)
{
    float torque_left, torque_right;
    differential_torque((Differential*)t->ty, torque, &torque_left, &torque_right);

    raSplitComponent s = t->tty.split;
    ra_tagged_send_torque(s.next_left, torque_left, v, dt);
    ra_tagged_send_torque(s.next_right, torque_right, v, dt);
}

raTaggedComponent* ra_tag_differential(Differential* diff)
{
    return ra_tagged_split_new(diff, diff_inertia, diff_angular_vel, diff_send_torque, free);
}

Gearbox* gearbox_new(VecFloat ratios, VecFloat inertias)
{
    Gearbox* gb = malloc(sizeof *gb);
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

static float gb_angular_vel(raTaggedComponent* t)
{
    return ((Gearbox*)t->ty)->input_angular_velocity;
}

raTaggedComponent* ra_tag_gearbox(Gearbox* gb)
{
    return ra_tagged_new(gb, (float (*)(void*))gearbox_inertia, gb_angular_vel, gearbox_send_torque,
        (void (*)(void*))gearbox_free);
}

// Calculate max normal force from desired torque caracheristics of the clutch.
Clutch* clutch_with_torque(
    float* max_normal_force, float max_static_torque, float max_kinetic_torque)
{
    *max_normal_force = max_static_torque;
    float static_coefficient = 1.0f;
    float kinetic_coefficient = max_kinetic_torque / *max_normal_force;

    Clutch* c = malloc(sizeof *c);
    c->static_coefficient = static_coefficient;
    c->kinetic_coefficient = kinetic_coefficient;
    c->velocity_threshold = 1.0;
    c->torque_sensitivity = 2.0;
    c->is_locked = false;
    return c;
}

static float clutch_inertia(void* ty)
{
    SUPPRESS_UNUSED(ty);
    // for now the clutch inertia is 0
    return 0;
}

/**Needed for configuring the normal force without a specific clutch implementation*/
typedef struct {
    Clutch* c;
    float curr_normal_force;
} ClutchTagged;

static ClutchTagged* clutch_tagged_new(Clutch* c)
{
    ClutchTagged* ct = malloc(sizeof *ct);
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

    float left_vel = t->prev->angular_velocity_fn(t->prev);
    float right_vel = t->tty.normal.next->angular_velocity_fn(t->tty.normal.next);
    clutch_torque_out(
        ct->c, torque, ct->curr_normal_force, left_vel, right_vel, &torque_left, &torque_right);

    ra_tagged_send_torque(t->tty.normal.next, torque_right, v, dt);
    if (ct->c->is_locked) {

    } else {
        // TODO: If clutch is not locked send torque_left back up the chain as velocity.
    }
}

static float clutch_vel(raTaggedComponent* t)
{
    SUPPRESS_UNUSED(t);
    return 0.0;
}

raTaggedComponent* ra_tag_clutch(Clutch* c)
{
    return ra_tagged_new(
        clutch_tagged_new(c), clutch_inertia, clutch_vel, clutch_send_torque, clutch_tagged_free);
}
