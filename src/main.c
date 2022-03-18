#include "body.h"
#include "common.h"
#include "powertrain.h"
#include "tiremodel.h"
#include "wheel.h"
#include <cjson/cJSON.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static inline cJSON* json_create_arr(void)
{
    cJSON* arr = cJSON_CreateArray();
    if (arr == NULL)
        exit(EXIT_FAILURE);
    return arr;
}

typedef struct {
    cJSON* obj;
    cJSON* position_x;
    cJSON* position_y;

    cJSON* velocity_x;
    cJSON* velocity_y;

    cJSON* yaw_velocity;
} JsonVehicle;

static JsonVehicle json_vehicle_new(cJSON* obj)
{
    cJSON* pos_x = json_create_arr();
    cJSON* pos_y = json_create_arr();
    cJSON* vel_x = json_create_arr();
    cJSON* vel_y = json_create_arr();
    cJSON* yaw_velocity = json_create_arr();

    cJSON_AddItemToObject(obj, "position_x", pos_x);
    cJSON_AddItemToObject(obj, "position_y", pos_y);

    cJSON_AddItemToObject(obj, "velocity_x", vel_x);
    cJSON_AddItemToObject(obj, "velocity_y", vel_y);

    cJSON_AddItemToObject(obj, "yaw_velocity", yaw_velocity);

    return (JsonVehicle) {
        .position_x = pos_x,
        .position_y = pos_y,
        .velocity_x = vel_x,
        .velocity_y = vel_y,
        .yaw_velocity = yaw_velocity,
    };
}

static void add_json_vehicle(
    JsonVehicle* v, Vector2f velocity, Vector2f position, float yaw_velocity)
{
    cJSON_AddItemToArray(v->position_x, cJSON_CreateNumber(position.x));
    cJSON_AddItemToArray(v->position_y, cJSON_CreateNumber(position.y));

    cJSON_AddItemToArray(v->velocity_x, cJSON_CreateNumber(velocity.x));
    cJSON_AddItemToArray(v->velocity_y, cJSON_CreateNumber(velocity.y));
    cJSON_AddItemToArray(v->yaw_velocity, cJSON_CreateNumber(yaw_velocity));
}

typedef struct {
    cJSON* obj;
    cJSON* hub_velocity_x;
    cJSON* hub_velocity_y;
    cJSON* angle;
    cJSON* angular_velocity;
    cJSON* reaction_torque;
    cJSON* slip_ratio;
    cJSON* slip_angle;
} JsonWheel;

static JsonWheel json_wheel_new(void)
{
    cJSON* hub_velocity_x = json_create_arr();
    cJSON* hub_velocity_y = json_create_arr();
    cJSON* angle = json_create_arr();
    cJSON* angular_velocity = json_create_arr();
    cJSON* reaction_torque = json_create_arr();
    cJSON* slip_ratio = json_create_arr();
    cJSON* slip_angle = json_create_arr();

    cJSON* obj = cJSON_CreateObject();

    cJSON_AddItemToObject(obj, "hub_velocity_x", hub_velocity_x);
    cJSON_AddItemToObject(obj, "hub_velocity_y", hub_velocity_y);
    cJSON_AddItemToObject(obj, "angle", angle);
    cJSON_AddItemToObject(obj, "angular_velocity", angular_velocity);
    cJSON_AddItemToObject(obj, "reaction_torque", reaction_torque);

    cJSON_AddItemToObject(obj, "slip_ratio", slip_ratio);
    cJSON_AddItemToObject(obj, "slip_angle", slip_angle);

    return (JsonWheel) {
        .obj = obj,
        .hub_velocity_x = hub_velocity_x,
        .hub_velocity_y = hub_velocity_y,
        .angle = angle,
        .angular_velocity = angular_velocity,
        .reaction_torque = reaction_torque,
        .slip_ratio = slip_ratio,
        .slip_angle = slip_angle,
    };
}

static void add_json_wheel(JsonWheel* j, const Wheel* w)
{
    cJSON_AddItemToArray(j->hub_velocity_x, cJSON_CreateNumber(w->hub_velocity.x));
    cJSON_AddItemToArray(j->hub_velocity_y, cJSON_CreateNumber(w->hub_velocity.y));
    cJSON_AddItemToArray(j->angle, cJSON_CreateNumber(w->angle));
    cJSON_AddItemToArray(j->angular_velocity, cJSON_CreateNumber(w->angular_velocity));
    cJSON_AddItemToArray(j->reaction_torque, cJSON_CreateNumber(w->reaction_torque));

    Vector2f slip = wheel_slip(w);
    cJSON_AddItemToArray(j->slip_ratio, cJSON_CreateNumber(slip.x));
    cJSON_AddItemToArray(j->slip_angle, cJSON_CreateNumber(slip.y));
}

typedef struct {
    cJSON* obj;
    cJSON* angular_velocity;
    cJSON* torque;
} JsonRotating;

static JsonRotating json_rotating_new(void)
{
    cJSON* angular_velocity = json_create_arr();
    cJSON* torque = json_create_arr();

    cJSON* obj = cJSON_CreateObject();

    cJSON_AddItemToObject(obj, "torque", torque);
    cJSON_AddItemToObject(obj, "angular_velocity", angular_velocity);

    return (JsonRotating) {
        .obj = obj,
        .angular_velocity = angular_velocity,
        .torque = torque,
    };
}

static void add_json_rotating(JsonRotating* e, float angular_velocity, float torque)
{
    cJSON_AddItemToArray(e->angular_velocity, cJSON_CreateNumber(angular_velocity));
    cJSON_AddItemToArray(e->torque, cJSON_CreateNumber(torque));
}

int main(void)
{
    float throttle_pos = 1.0;
    float brake_pos = 0.0;
    float clutch_pos = 0.0;
    float steering_angle = 0.0;

    float elapsed_time = 0.0;

    float dt = 1.0 / 50.0;
    float mass = 1580.0f;
    float inv_vehicle_mass = 1.0 / mass;
    float gravity = 9.806f;
    float air_density = 1.2041f;

    // Uses iso8855 coordinates
    Vector2f velocity = vector2f_default();
    Vector2f position = vector2f_default();
    float yaw_velocity = 0.0;

    Body body = body_new(0.36, 1.9, 3.6f, 1.47f, 1.475f);

    Table torque_map = table_with_capacity(2, 2);
    torque_map.x[0] = 0.0;
    torque_map.x[1] = 1.0;

    torque_map.y[0] = 0.0;
    torque_map.y[1] = 1.0;

    torque_map.z[0][0] = -0.2;
    torque_map.z[0][1] = -0.2;
    /* torque_map.z[0][0] = 0.0; */
    /* torque_map.z[0][1] = 0.0; */
    torque_map.z[1][0] = 1.0;
    torque_map.z[1][1] = 1.0;

    Engine engine = engine_new(1.0 / 0.5, torque_map, 5000.0, 80.0);

    int num_gears = 6;
    VecFloat ratios = vec_with_capacity(num_gears);
    vec_push_float(&ratios, 3.2);
    vec_push_float(&ratios, 2.31);
    vec_push_float(&ratios, 1.82);
    vec_push_float(&ratios, 1.52);
    vec_push_float(&ratios, 1.3);
    vec_push_float(&ratios, 1.0);

    VecFloat inertias = vec_with_capacity(num_gears);
    vec_push_float(&inertias, 0.2);
    vec_push_float(&inertias, 0.18);
    vec_push_float(&inertias, 0.16);
    vec_push_float(&inertias, 0.15);
    vec_push_float(&inertias, 0.14);
    vec_push_float(&inertias, 0.1);

    Gearbox gb = gearbox_new(ratios, inertias, -1.6, 0.95);
    gb.curr_gear = 1;

    Differential diff = (Differential) { .ratio = 2.4, .inv_inertia = 1.0 / 0.18 };

    Cog cog = cog_from_distribution(0.55, 0.4, body.wheelbase);

    TireModel model = (TireModel) {
        .bx = 1.9,
        .cx = 1.65,
        .dx = 1.1,
        .ex = -1.0,
        .vvx = 0.0,
        .vhx = 0.0,

        .by = 9.0,
        .cy = 1.36,
        .dy = 1.0,
        .ey = 0.96,
        .vvy = 0.0,
        .vhy = 0.0,

        .peak_slip_x = 0.4,
        .peak_slip_y = deg_to_rad(20.0f),
    };

    Vector2f fl_pos = (Vector2f) { .x = cog_distance_to_front(cog),
        .y = cog_distance_to_left(cog, body.front_track_width) };
    Vector2f fr_pos = (Vector2f) { .x = cog_distance_to_front(cog),
        .y = cog_distance_to_right(cog, body.front_track_width) };
    Vector2f rl_pos = (Vector2f) { .x = cog_distance_to_rear(cog, body.wheelbase),
        .y = cog_distance_to_left(cog, body.rear_track_width) };
    Vector2f rr_pos = (Vector2f) { .x = cog_distance_to_rear(cog, body.wheelbase),
        .y = cog_distance_to_right(cog, body.rear_track_width) };

    float min_speed = 0.01;
    Wheel wfl = wheel_new(1.0 / 0.6, 0.344, 0.0, fl_pos, min_speed);
    Wheel wfr = wheel_new(1.0 / 0.6, 0.344, 0.0, fr_pos, min_speed);

    Wheel wrl = wheel_new(1.0 / 0.6, 0.344, 0.0, rl_pos, min_speed);
    Wheel wrr = wheel_new(1.0 / 0.6, 0.344, 0.0, rr_pos, min_speed);

    engine.angular_velocity = min_speed / wrl.effective_radius;

    cJSON* output_json = cJSON_CreateObject();
    cJSON* json_elapsed_time = json_create_arr();
    cJSON_AddItemToObject(output_json, "elapsed_time", json_elapsed_time);

    cJSON* json_throttle = json_create_arr();
    cJSON_AddItemToObject(output_json, "throttle", json_throttle);

    cJSON* json_brake = json_create_arr();
    cJSON_AddItemToObject(output_json, "brake", json_brake);

    cJSON* json_clutch = json_create_arr();
    cJSON_AddItemToObject(output_json, "clutch", json_clutch);

    cJSON* json_steering = json_create_arr();
    cJSON_AddItemToObject(output_json, "steering", json_steering);

    cJSON* json_gear = json_create_arr();
    cJSON_AddItemToObject(output_json, "gear", json_gear);

    JsonVehicle json_v = json_vehicle_new(output_json);

    JsonRotating json_engine = json_rotating_new();
    cJSON_AddItemToObject(output_json, "engine", json_engine.obj);

    JsonWheel json_fl = json_wheel_new();
    cJSON_AddItemToObject(output_json, "fl_wheel", json_fl.obj);

    JsonWheel json_fr = json_wheel_new();
    cJSON_AddItemToObject(output_json, "fr_wheel", json_fr.obj);

    JsonWheel json_rl = json_wheel_new();
    cJSON_AddItemToObject(output_json, "rl_wheel", json_rl.obj);

    JsonWheel json_rr = json_wheel_new();
    cJSON_AddItemToObject(output_json, "rr_wheel", json_rr.obj);

    while (elapsed_time <= 60.0) {
        wheel_change_angle(&wfl, steering_angle);
        wheel_change_angle(&wfr, steering_angle);

        float t_ratio = gearbox_ratio(&gb);

        float internal_throttle = throttle_pos;
        if (engine.angular_velocity > angular_vel_rpm_to_rads(4800.0)) {
            internal_throttle = 0.0;
        }
        float eng_torque = engine_torque(&engine, internal_throttle);

        float trans_torque = eng_torque * t_ratio;
        float inv_inertia = engine.inv_inertia + diff.inv_inertia + gearbox_inertia(&gb);

        float left_torque;
        float right_torque;
        differential_torque(&diff, trans_torque, &left_torque, &right_torque);

        wheel_update(&wfl, velocity, yaw_velocity, 0.0, 0.0f, dt, min_speed);
        wheel_update(&wfr, velocity, yaw_velocity, 0.0, 0.0f, dt, min_speed);

        wheel_update(&wrl, velocity, yaw_velocity, inv_inertia, left_torque, dt, min_speed);
        wheel_update(&wrr, velocity, yaw_velocity, inv_inertia, right_torque, dt, min_speed);

        add_json_wheel(&json_fl, &wfl);
        add_json_wheel(&json_fr, &wfr);
        add_json_wheel(&json_rl, &wrl);
        add_json_wheel(&json_rr, &wrr);

        float fz = mass * gravity * 0.25;

        Vector2f wfl_f = vector2f_rotate(wheel_force(&wfl, &model, fz, 1.0), -wfl.angle);
        Vector2f wfr_f = vector2f_rotate(wheel_force(&wfr, &model, fz, 1.0), -wfr.angle);
        Vector2f wrl_f = vector2f_rotate(wheel_force(&wrl, &model, fz, 1.0), -wrl.angle);
        Vector2f wrr_f = vector2f_rotate(wheel_force(&wrr, &model, fz, 1.0), -wrr.angle);

        float resitance_force_x = body_air_resistance(&body, air_density, velocity.x);
        Vector2f force
            = (Vector2f) { .x = wfl_f.x + wfr_f.x + wrl_f.x + wrr_f.x + resitance_force_x,
                  .y = wfl_f.y + wfr_f.y + wrl_f.y + wrr_f.y };

        printf("Force: %f/%f\n", force.x, force.y);

        velocity.x += integrate(force.x, inv_vehicle_mass, dt);
        velocity.y += integrate(force.y, inv_vehicle_mass, dt);

        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        engine_set_angular_velocity(&engine,
            differential_velocity(&diff, wrl.angular_velocity, wrr.angular_velocity) * t_ratio);

        printf("Engine velocity: %.1frpm. Torque: %f\n",
            angular_vel_rads_to_rpm(engine.angular_velocity), eng_torque);
        Vector2f slip_front = wheel_slip(&wfl);
        Vector2f slip_rear = wheel_slip(&wrl);
        printf("Slip F x/y = %.2f/%.2f | Slip R = %.2f/%.2f | ", slip_front.x, slip_front.y,
            slip_rear.x, slip_rear.y);
        printf("m/s = %f/%f\n", velocity.x, velocity.y);

        add_json_rotating(&json_engine, engine.angular_velocity, eng_torque);
        add_json_vehicle(&json_v, velocity, position, yaw_velocity);

        cJSON_AddItemToArray(json_throttle, cJSON_CreateNumber(throttle_pos));
        cJSON_AddItemToArray(json_brake, cJSON_CreateNumber(brake_pos));
        cJSON_AddItemToArray(json_clutch, cJSON_CreateNumber(clutch_pos));
        cJSON_AddItemToArray(json_steering, cJSON_CreateNumber(steering_angle));
        cJSON_AddItemToArray(json_gear, cJSON_CreateNumber(gb.curr_gear));
        cJSON_AddItemToArray(json_elapsed_time, cJSON_CreateNumber(elapsed_time));
        elapsed_time += dt;
    }

    engine_free(&engine);
    gearbox_free(&gb);

    FILE* fs = fopen("../output.json", "w");
    if (fs == NULL)
        exit(EXIT_FAILURE);

    char* json_str = cJSON_Print(output_json);
    if (json_str == NULL)
        exit(EXIT_FAILURE);

    fprintf(fs, "%s", json_str);

    free(json_str);
    fclose(fs);

    cJSON_Delete(output_json);

    return 0;
}
