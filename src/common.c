#include "common.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

Vector2f vector2f_default(void) { return (Vector2f) { .x = 0.0f, .y = 0.0f }; }

float vector2f_length(Vector2f v) { return sqrtf(v.x * v.x + v.y * v.y); }

Vector2f vector2f_rotate(Vector2f v, float angle)
{
    float a_sin = sin(angle);
    float a_cos = cos(angle);
    float x = v.x * a_cos - v.y * a_sin;
    float y = v.x * a_sin + v.y * a_cos;

    return (Vector2f) { .x = x, .y = y };
}

Vector3f vector3f_default(void) { return (Vector3f) { .x = 0.0f, .y = 0.0f, .z = 0.0f }; }

float vector3f_length(Vector3f v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

float integrate(float torque, float inv_inertia, float dt) { return torque * inv_inertia * dt; }

float rad_to_deg(float radians) { return radians * (180.0 / M_PI); }

float deg_to_rad(float degrees) { return (degrees * M_PI) / 180.0; }

float signum(float v) { return (copysignf(1.0, v)); }

VecFloat vec_with_capacity(int capacity)
{
    float* elements = malloc(sizeof *elements * capacity);

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
