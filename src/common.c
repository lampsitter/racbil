#include "common.h"
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

inline float rads_to_rpm(AngularVelocity rads) { return rads * 60.0 / (2.0 * M_PI); }
inline float rpm_to_rads(AngularVelocity rpm) { return 2.0 * M_PI * rpm / 60.0; }
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

struct raTaggedComponent {
    /* float (*angular_velocity)(void); */
    /* void (*send_torque)(float torque, raTaggedComponent* next); */
    /* void (*receive_angular_velocity)(raTaggedComponent* next); */
    void* ty;
    raTaggedComponent* prev;
    void (*free_fn)(void* ptr);
    enum raTy comp_ty;
    union raComponentTypes tty;
};

void* ra_tagged_component_inner(raTaggedComponent* c) { return c->ty; }

raTaggedComponent* ra_tagged_new(void* ty, void (*free_fn)(void* ptr))
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
    t->prev = NULL;

    t->tty.normal = (raComponent) {
        .next = NULL,
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

raTaggedComponent* ra_tagged_split_new(void* ty, void (*free_fn)(void* ptr))
{
    raTaggedComponent* t = malloc(sizeof *t);
    if (t == NULL) {
        abort();
    }
    t->comp_ty = raTySplit;
    t->ty = ty;
    t->free_fn = free_fn;
    t->prev = NULL;

    t->tty.split = (raSplitComponent) {
        .next_left = NULL,
        .next_right = NULL,
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

Vector2f vector2f_default(void) { return (Vector2f) { .x = 0.0f, .y = 0.0f }; }

float vector2f_length(Vector2f v) { return sqrtf(v.x * v.x + v.y * v.y); }

Vector2f vector2f_plus_vec(size_t num_args, ...)
{
    va_list ap;
    va_start(ap, num_args);

    Vector2f a = { .x = 0.0, .y = 0.0 };

    for (size_t i = 0; i < num_args; i++) {
        Vector2f v = va_arg(ap, Vector2f);
        a.x += v.x;
        a.y += v.y;
    }

    va_end(ap);

    return a;
}

Vector2f vector2f_rotate(Vector2f v, float angle)
{
    float a_sin = sinf(angle);
    float a_cos = cosf(angle);
    float x = v.x * a_cos - v.y * a_sin;
    float y = v.x * a_sin + v.y * a_cos;

    return (Vector2f) { .x = x, .y = y };
}

Vector3f vector3f_default(void) { return (Vector3f) { .x = 0.0f, .y = 0.0f, .z = 0.0f }; }

float vector3f_length(Vector3f v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

float integrate(float dv, float dt) { return dv * dt; }

float rad_to_deg(float radians) { return radians * (180.0 / M_PI); }

float deg_to_rad(float degrees) { return (degrees * M_PI) / 180.0; }

float signum(float v) { return (copysignf(1.0, v)); }

VecFloat vec_with_capacity(int capacity)
{
    float* elements = malloc(sizeof *elements * capacity);
    if (elements == NULL) {
        abort();
    }

    return (VecFloat) { .elements = elements, .capacity = capacity, .len = 0 };
}

void vec_push_float(VecFloat* v, float element)
{
    if (v->capacity == v->len) {
        v->capacity *= 2;
        v->elements = realloc(v->elements, sizeof *v->elements * v->capacity);
    }

    v->elements[v->len] = element;
    v->len += 1;
}

void vec_free(VecFloat* v)
{
    free(v->elements);
    v->elements = NULL;
    v->capacity = 0;
    v->len = 0;
}

Table table_with_capacity(size_t x_elements, size_t y_elements)
{
    assert(x_elements >= 2 && y_elements >= 2);
    float* x = malloc(x_elements * sizeof *x);
    if (x == NULL) {
        exit(EXIT_FAILURE);
    }

    float* y = malloc(y_elements * sizeof *y);
    if (y == NULL) {
        exit(EXIT_FAILURE);
    }

    float** z = malloc(x_elements * sizeof *z);
    if (z == NULL) {
        exit(EXIT_FAILURE);
    }

    for (size_t i = 0; i < x_elements; i++) {
        z[i] = malloc(y_elements * sizeof **z);
        if (z[i] == NULL) {
            exit(EXIT_FAILURE);
        }
    }

    return (Table) { .x = x, .y = y, .z = z, .x_capacity = x_elements, .y_capacity = y_elements };
}

void table_free(Table* table)
{
    free(table->x);
    free(table->y);

    for (size_t i = 0; i < table->x_capacity; i++) {
        free(table->z[i]);
    }

    free(table->z);

    table->x = NULL;
    table->y = NULL;
    table->z = NULL;
    table->x_capacity = 0;
    table->y_capacity = 0;
}

static inline float linear_interpolation(float x1, float x2, float x3, float y1, float y3)
{
    return (x2 - x1) * (y3 - y1) / (x3 - x1) + y1;
}

/**
 * find two values on each side of a value
 */
static void boundary_values(
    float value, float* elements, size_t num_elements, size_t* index_below, size_t* index_above)
{
    bool found = false;
    for (size_t i = 0; i < num_elements; ++i) {
        if (elements[i] >= value) {
            if (i == 0) {
                *index_below = 0;
                *index_above = 1;
            } else {
                *index_below = i - 1;
                *index_above = i;
            }

            found = true;
            break;
        }
    }

    if (!found) {
        *index_below = num_elements - 2;
        *index_above = num_elements - 1;
    }
}

float table_lookup(Table* table, float x, float y)
{
    size_t below_xi, above_xi;
    boundary_values(x, table->x, table->x_capacity, &below_xi, &above_xi);

    size_t below_yi, above_yi;
    boundary_values(y, table->y, table->y_capacity, &below_yi, &above_yi);

    float below_x = table->x[below_xi];
    float below_y = table->y[below_yi];
    float above_x = table->x[above_xi];
    float above_y = table->y[above_yi];

    float lower_z = linear_interpolation(
        below_y, y, above_y, table->z[below_xi][below_yi], table->z[below_xi][above_yi]);

    float upper_z = linear_interpolation(
        below_y, y, above_y, table->z[above_xi][below_yi], table->z[above_xi][above_yi]);

    return linear_interpolation(below_x, x, above_x, lower_z, upper_z);
}
