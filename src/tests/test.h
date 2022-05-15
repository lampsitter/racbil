#ifndef RA_TEST_TEST_H
#define RA_TEST_TEST_H
#include "../powertrain.h"

Engine* test_engine(void)
{
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

    return engine_new(0.5, torque_map);
}

Gearbox* test_gearbox(void)
{
    int num_gears = 7;
    VecFloat ratios = vec_with_capacity(num_gears);
    vec_push_float(&ratios, -3.6);
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

    return gearbox_new(ratios, inertias);
}

#endif // RA_TEST_TEST_H
