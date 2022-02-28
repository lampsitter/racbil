#ifndef RA_COMMON_H
#define RA_COMMON_H
#include <math.h>


#define EPSILON 1.19209290e-07 // From rust stdlib
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

typedef float AngularVelocity;

typedef struct
{
    float x, y;
} Vector2f;

Vector2f vector2f_default(void);
float vector2f_length(Vector2f v);

typedef struct
{
    float x, y, z;
} Vector3f;

Vector3f vector3f_default(void);
float vector3f_length(Vector3f v);

float integrate(float torque, float inv_inertia, float dt);

float rad_to_deg(float radians);
float deg_to_rad(float degrees);

#endif /* RA_COMMON_H */
