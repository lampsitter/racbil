#ifndef RA_COMMON_H
#define RA_COMMON_H
#include <math.h>
#include <sys/types.h>

#define EPSILON 1.19209290e-07 // From rust stdlib
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define SUPPRESS_UNUSED(x) (void)(x)

typedef float AngularVelocity;

float rads_to_rpm(AngularVelocity rads);
float rpm_to_rads(AngularVelocity rpm);

typedef struct {
    float x, y;
} Vector2f;

Vector2f vector2f_default(void);
float vector2f_length(Vector2f v);
Vector2f vector2f_plus_vec(size_t num_args, ...);
Vector2f vector2f_rotate(Vector2f v, float angle);

#define VECTOR2F_PLUS(...)                                                                         \
    vector2f_plus_vec(sizeof((Vector2f[]) { __VA_ARGS__ }) / sizeof(Vector2f), __VA_ARGS__)

typedef struct {
    float x, y, z;
} Vector3f;

Vector3f vector3f_default(void);
float vector3f_length(Vector3f v);

float integrate(float dv, float dt);

float rad_to_deg(float radians);
float deg_to_rad(float degrees);
float signum(float v);

typedef struct {
    float* elements;
    size_t capacity;
    size_t len;
} VecFloat;

VecFloat vec_with_capacity(int capacity);
void vec_push_float(VecFloat* v, float element);
void vec_free(VecFloat* v);

/** A table with sorted x and y from lowest to highest */
typedef struct {
    size_t x_capacity, y_capacity;
    float* x;
    float* y;
    float** z;
} Table;

Table table_with_capacity(size_t x_elements, size_t y_elements);
void table_free(Table* table);
float table_lookup(const Table* table, float x, float y);

#endif /* RA_COMMON_H */
