#include "racbil.h"
#include <assert.h>
#include <cjson/cJSON.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <zlib.h>

static inline cJSON* json_create_arr(void)
{
    cJSON* arr = cJSON_CreateArray();
    if (arr == NULL) {
        exit(EXIT_FAILURE);
    }
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
    cJSON* input_torque;
    cJSON* brake_torque;
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
    cJSON* input_torque = json_create_arr();
    cJSON* brake_torque = json_create_arr();
    cJSON* reaction_torque = json_create_arr();
    cJSON* slip_ratio = json_create_arr();
    cJSON* slip_angle = json_create_arr();

    cJSON* obj = cJSON_CreateObject();

    cJSON_AddItemToObject(obj, "hub_velocity_x", hub_velocity_x);
    cJSON_AddItemToObject(obj, "hub_velocity_y", hub_velocity_y);
    cJSON_AddItemToObject(obj, "angle", angle);
    cJSON_AddItemToObject(obj, "angular_velocity", angular_velocity);
    cJSON_AddItemToObject(obj, "input_torque", input_torque);
    cJSON_AddItemToObject(obj, "brake_torque", brake_torque);
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
        .input_torque = input_torque,
        .brake_torque = brake_torque,
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
    cJSON_AddItemToArray(j->input_torque, cJSON_CreateNumber(w->input_torque));
    cJSON_AddItemToArray(j->brake_torque, cJSON_CreateNumber(w->external_torque));
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

int main(int argc, char** argv)
{
    bool should_write = false;
    bool is_quiet = false;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--write") == 0) {
            should_write = true;
        } else if (strcmp(argv[i], "--quiet") == 0) {
            is_quiet = true;
        } else {
            fprintf(stderr, "Unknown argument(s)\n");
            exit(EXIT_FAILURE);
        }
    }

    float throttle_pos = 1.0;
    float brake_pos = 0.0;
    float clutch_pos = 1.0;
    float steering_angle = deg_to_rad(0.0);
    float steering_ratio = 1.0 / 16.0;

    float elapsed_time = 0.0;

    float dt = 1.0 / 120.0;
    float mass = 1580.0f;
    float gravity = 9.806f;
    float air_density = 1.2041f;

    // Uses iso8855 coordinates
    Vector2f velocity = vector2f_default();
    Vector2f position = vector2f_default();
    float yaw_velocity = 0.0;
    float rotation = 0.0;

    Body body = body_new(2600.0, 0.36, 0.1, 0.07, 1.9, 3.6f, 1.47f, 1.475f);
    Table torque_map = table_with_capacity(2, 14);
    torque_map.x[0] = 0.0;
    torque_map.x[1] = 1.0;
    torque_map.y[0] = 0.0;
    torque_map.y[1] = 52.35987755982989;
    torque_map.y[2] = 52.35987755982989;
    torque_map.y[3] = 104.71975511965978;
    torque_map.y[4] = 157.07963267948966;
    torque_map.y[5] = 209.43951023931956;
    torque_map.y[6] = 261.79938779914943;
    torque_map.y[7] = 314.1592653589793;
    torque_map.y[8] = 366.5191429188092;
    torque_map.y[9] = 418.8790204786391;
    torque_map.y[10] = 471.23889803846896;
    torque_map.y[11] = 523.5987755982989;
    torque_map.y[12] = 575.9586531581288;
    torque_map.y[13] = 628.3185307179587;
    torque_map.z[0][0] = -50.0;
    torque_map.z[0][1] = -53.1415926535898;
    torque_map.z[0][2] = -53.1415926535898;
    torque_map.z[0][3] = -56.283185307179586;
    torque_map.z[0][4] = -59.424777960769376;
    torque_map.z[0][5] = -62.56637061435917;
    torque_map.z[0][6] = -65.70796326794897;
    torque_map.z[0][7] = -68.84955592153875;
    torque_map.z[0][8] = -71.99114857512855;
    torque_map.z[0][9] = -75.13274122871834;
    torque_map.z[0][10] = -78.27433388230814;
    torque_map.z[0][11] = -81.41592653589794;
    torque_map.z[0][12] = -84.55751918948772;
    torque_map.z[0][13] = -87.69911184307752;
    torque_map.z[1][0] = -50.0;
    torque_map.z[1][1] = -53.1415926535898;
    torque_map.z[1][2] = 15.0;
    torque_map.z[1][3] = 60.0;
    torque_map.z[1][4] = 90.0;
    torque_map.z[1][5] = 120.0;
    torque_map.z[1][6] = 142.5;
    torque_map.z[1][7] = 150.0;
    torque_map.z[1][8] = 148.5;
    torque_map.z[1][9] = 139.5;
    torque_map.z[1][10] = 127.5;
    torque_map.z[1][11] = 112.5;
    torque_map.z[1][12] = 90.0;
    torque_map.z[1][13] = 60.0;

    Engine* engine = engine_new(0.5, torque_map);
    engine->angular_velocity = rpm_to_rads(1200.0);
    RevLimiterHard limiter = rev_limiter_hard_new(rpm_to_rads(4800.0), rpm_to_rads(4650.0));

    int num_gears = 7;
    VecFloat ratios = vec_with_capacity(num_gears);
    vec_push_float(&ratios, -1.6);
    vec_push_float(&ratios, 3.2);
    vec_push_float(&ratios, 2.31);
    vec_push_float(&ratios, 1.82);
    vec_push_float(&ratios, 1.52);
    vec_push_float(&ratios, 1.3);
    vec_push_float(&ratios, 1.0);

    VecFloat inertias = vec_with_capacity(num_gears);
    vec_push_float(&inertias, 0.3);
    vec_push_float(&inertias, 0.2);
    vec_push_float(&inertias, 0.18);
    vec_push_float(&inertias, 0.16);
    vec_push_float(&inertias, 0.15);
    vec_push_float(&inertias, 0.14);
    vec_push_float(&inertias, 0.1);

    float clutch_normal_force;
    Clutch* clutch = clutch_with_torque(&clutch_normal_force, 300.0, 240.0);

    Gearbox* gb = gearbox_new(ratios, inertias);
    gb->curr_gear = 1;

    Differential* diff = differential_new(2.4, 0.18, DiffTypeLocked);

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
    Wheel* wfl = wheel_new(0.6, 0.344, fl_pos, min_speed);
    Wheel* wfr = wheel_new(0.6, 0.344, fr_pos, min_speed);
    Wheel* wrl = wheel_new(0.6, 0.344, rl_pos, min_speed);
    Wheel* wrr = wheel_new(0.6, 0.344, rr_pos, min_speed);

    // brake system
    MasterCylinder master_cyl = master_cylinder_new(10000e3);
    BrakeDisc bd = brake_disc_new(0.3, 0.24);
    Caliper front_calipers = caliper_new(cylinder_from_diameter(0.05), 0.25, 2);
    Caliper rear_calipers = caliper_new(cylinder_from_diameter(0.05), 0.26, 2);

    raTaggedComponent* c_fl = ra_tag_wheel(wfl);
    raTaggedComponent* c_fr = ra_tag_wheel(wfr);
    raTaggedComponent* c_rl = ra_tag_wheel(wrl);
    raTaggedComponent* c_rr = ra_tag_wheel(wrr);

    raTaggedComponent* c_diff = ra_tag_differential(diff);
    raTaggedComponent* c_gearbox = ra_tag_gearbox(gb);
    raTaggedComponent* c_clutch = ra_tag_clutch(clutch);
    raTaggedComponent* c_engine = ra_tag_engine(engine);

    assert(ra_tagged_add_next(c_engine, c_clutch) == 0);
    assert(ra_tagged_add_next(c_clutch, c_gearbox) == 0);
    assert(ra_tagged_add_next(c_gearbox, c_diff) == 0);
    assert(ra_tagged_add_next_left(c_diff, c_rl) == 0);
    assert(ra_tagged_add_next_right(c_diff, c_rr) == 0);

    raOverviewSystem osys = ra_overwiew_system_new(3);
    osys.subsystems[0] = c_fl;
    osys.subsystems[1] = c_fr;
    osys.subsystems[2] = c_engine;

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

    JsonRotating json_gearbox_input = json_rotating_new();
    cJSON_AddItemToObject(output_json, "gearbox_input_shaft", json_gearbox_input.obj);

    JsonWheel json_fl = json_wheel_new();
    cJSON_AddItemToObject(output_json, "fl_wheel", json_fl.obj);

    JsonWheel json_fr = json_wheel_new();
    cJSON_AddItemToObject(output_json, "fr_wheel", json_fr.obj);

    JsonWheel json_rl = json_wheel_new();
    cJSON_AddItemToObject(output_json, "rl_wheel", json_rl.obj);

    JsonWheel json_rr = json_wheel_new();
    cJSON_AddItemToObject(output_json, "rr_wheel", json_rr.obj);

    cJSON_AddNumberToObject(output_json, "dt", dt);

    int stage = 0;
    while (elapsed_time <= 40.0) {
        if (stage == 0 && velocity.x >= 16.0) {
            stage = 1;

            throttle_pos = 0.0;
            brake_pos = 1.0;
        }

        if (clutch_pos > 0.0) {
            clutch_pos = fminf(1.0, 1.0 - fmaxf(elapsed_time * elapsed_time, 0.0));
        }

        set_ackerman_angle(steering_angle * steering_ratio, body.wheelbase, wfl, wfr);

        float eng_torque = engine_torque(engine, rev_limiter_hard(&limiter, engine, throttle_pos));

        ((ClutchTagged*)ra_tagged_component_inner(c_clutch))->curr_normal_force
            = clutch_normal_force * (1.0 - clutch_pos);

        float brake_pressure = master_cyl.max_pressure * brake_pos;

        wfl->external_torque
            = brake_torque(&bd, &front_calipers, brake_pressure, wfl->angular_velocity);
        wfr->external_torque
            = brake_torque(&bd, &front_calipers, brake_pressure, wfr->angular_velocity);
        wrl->external_torque
            = brake_torque(&bd, &rear_calipers, brake_pressure, wrl->angular_velocity);
        wrr->external_torque
            = brake_torque(&bd, &rear_calipers, brake_pressure, wrr->angular_velocity);

        raVelocities comb_vel
            = (raVelocities) { .velocity_cog = velocity, .yaw_velocity_cog = yaw_velocity };
        ra_tagged_send_torque(c_fr, 0.0, comb_vel, dt);
        ra_tagged_send_torque(c_fl, 0.0, comb_vel, dt);
        ra_tagged_send_torque(c_engine, eng_torque, comb_vel, dt);

        add_json_wheel(&json_fl, wfl);
        add_json_wheel(&json_fr, wfr);
        add_json_wheel(&json_rl, wrl);
        add_json_wheel(&json_rr, wrr);

        float fz = mass * gravity * 0.5;
        float fzf_lift = body_lift_front(&body, air_density, velocity.x);
        float fzr_lift = body_lift_rear(&body, air_density, velocity.x);

        float fz_front = (fz + fzf_lift) * 0.5;
        float fz_rear = (fz + fzr_lift) * 0.5;

        Vector2f fl_f = wheel_force(wfl, &model, fz_front, 1.0);
        Vector2f fr_f = wheel_force(wfr, &model, fz_front, 1.0);
        Vector2f rl_f = wheel_force(wrl, &model, fz_rear, 1.0);
        Vector2f rr_f = wheel_force(wrr, &model, fz_rear, 1.0);

        Vector2f wfl_f = vector2f_rotate(fl_f, -wfl->angle);
        Vector2f wfr_f = vector2f_rotate(fr_f, -wfr->angle);
        Vector2f wrl_f = vector2f_rotate(rl_f, -wrl->angle);
        Vector2f wrr_f = vector2f_rotate(rr_f, -wrr->angle);

        Vector2f resistance = body_air_resistance(&body, air_density, velocity.x);
        Vector2f force = VECTOR2F_PLUS(wfl_f, wfr_f, wrl_f, wrr_f, resistance);

        Vector2f old_velocity = velocity;
        velocity.x += integrate(force.x / mass, dt);
        velocity.y += integrate(force.y / mass, dt);

        if (signum(old_velocity.x) != signum(velocity.x)) {
            velocity.x = 0.0;
        }

        Vector2f vel_world = vector2f_rotate(velocity, rotation);
        position.x += vel_world.x * dt;
        position.y += vel_world.y * dt;

        for (size_t i = 0; i < osys.num_subsystems; i++) {
            raTaggedComponent* t = osys.subsystems[i];
            ra_tagged_update_angular_velocity(t);
        }

        float zz_torque = yaw_torque(wfl, wfr, wrl, wrr, wfl_f, wfr_f, wrl_f, wrr_f);
        yaw_velocity += zz_torque / body.i_zz * dt;
        rotation += yaw_velocity * dt;

        if (!is_quiet) {
            printf("--------------------------------\n");
            printf("Force: %f/%f\n", force.x, force.y);
            printf("Engine velocity: %.1frpm. Gearbox input velocity: %.1frpm\n",
                rads_to_rpm(engine->angular_velocity), rads_to_rpm(gb->input_angular_velocity));

            Vector2f sfl = wheel_slip(wfl);
            Vector2f sfr = wheel_slip(wfr);
            Vector2f srl = wheel_slip(wrl);
            Vector2f srr = wheel_slip(wrr);

            printf("Steering = %f, Throttle = %f, Brake: %f, Clutch = %f\n", steering_angle,
                throttle_pos, brake_pos, clutch_pos);
            puts("Wheels:");
            printf("\tAngle Fl = %f | Angle Fr = %f\n", wfl->angle, wfr->angle);
            printf("\tAngle Rl = %f | Angle Rr = %f\n", wrl->angle, wrr->angle);

            printf("\tAngular Vel Fl = %f | Angular Vel Fr = %f\n", wfl->angular_velocity,
                wfr->angular_velocity);
            printf("\tAngular Vel Rl = %f | Angular Vel Rr = %f\n", wrl->angular_velocity,
                wrr->angular_velocity);

            printf("\tHub Vel Fl x/y = %f/%f | Hub Vel Fr = %f/%f\n", wfl->hub_velocity.x,
                wfl->hub_velocity.y, wfr->hub_velocity.x, wfr->hub_velocity.y);
            printf("\tHub Vel Rl x/y = %f/%f | Hub Vel Rr = %f/%f\n", wrl->hub_velocity.x,
                wrl->hub_velocity.y, wrr->hub_velocity.x, wrr->hub_velocity.y);

            printf("\tSlip Fl x/y = %f/%f | Slip Fr = %f/%f\n", sfl.x, sfl.y, sfr.x, sfr.y);
            printf("\tSlip Rl x/y = %f/%f | Slip Rr = %f/%f\n", srl.x, srl.y, srr.x, srr.y);
            printf(
                "\tForce Fl x/y = %f/%f | Force Fr = %f/%f\n", wfl_f.x, wfl_f.y, wfr_f.x, wfr_f.y);
            printf(
                "\tForce Rl x/y = %f/%f | Force Rr = %f/%f\n", wrl_f.x, wrl_f.y, wrr_f.x, wrr_f.y);

            printf("\tTorque Fl x/y = %f | Torque Fr = %f\n",
                wfl->input_torque + wfl->external_torque, wfr->input_torque + wfr->external_torque);
            printf("\tTorque Rl x/y = %f | Torque Rr = %f\n",
                wrl->input_torque + wrl->external_torque, wrr->input_torque + wrr->external_torque);
            printf("Velocity(m/s) = %f/%f | Yaw velocity = %f\n", velocity.x, velocity.y,
                yaw_velocity);
            puts("");
        }

        add_json_rotating(&json_engine, engine->angular_velocity, 0.0);
        add_json_rotating(&json_gearbox_input, gb->input_angular_velocity, 0.0);
        add_json_vehicle(&json_v, velocity, position, yaw_velocity);

        cJSON_AddItemToArray(json_throttle, cJSON_CreateNumber(throttle_pos));
        cJSON_AddItemToArray(json_brake, cJSON_CreateNumber(brake_pos));
        cJSON_AddItemToArray(json_clutch, cJSON_CreateNumber(clutch_pos));
        cJSON_AddItemToArray(json_steering, cJSON_CreateNumber(steering_angle));
        cJSON_AddItemToArray(json_gear, cJSON_CreateNumber(gb->curr_gear));
        cJSON_AddItemToArray(json_elapsed_time, cJSON_CreateNumber(elapsed_time));
        elapsed_time += dt;
    }

    ra_overwiew_system_free(osys);

    if (should_write) {
        gzFile fs = gzopen("../output.json.gz", "wb");
        if (fs == NULL) {
            exit(EXIT_FAILURE);
        }

        char* json_str = cJSON_Print(output_json);
        if (json_str == NULL) {
            exit(EXIT_FAILURE);
        }

        gzwrite(fs, json_str, strlen(json_str));

        free(json_str);
        gzclose(fs);

        puts("Wrote to file output.json.gz");
    }

    cJSON_Delete(output_json);

    return 0;
}
