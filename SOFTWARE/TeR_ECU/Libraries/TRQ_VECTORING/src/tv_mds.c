#include "tv_mds.h"
#include "v2_vehicle_dynamics.h"
#include "TeR_TRQMANAGER.h"

static v2_params_t v2_params;
static v2_state_t  v2_state;
static uint8_t     v2_ready = 0;

void tv_reset_state(void) {
    v2_reset_state(&v2_state);
}

static float v2_ground_speed_ms(void) {
    float front_kmh = ter_front_v_front_v_decode(TeR.speed.front_v);
    if (front_kmh > 0.1f) return front_kmh / 3.6f;
    return TeR.wheelInfo.speed / 3.6f;
}

trqMap_t trqVectoring(trq_t limit) {
    if (!v2_ready) {
        v2_init_params(&v2_params);
        tv_reset_state();
        v2_ready = 1;
    }

    float apps_pct   = (float)TeR.apps.apps_av / 255.0f;
    float brake_bar  = ter_bpps_bpps_decode(TeR.bpps.bpps);
    float steer_rad  = ter_steer_angle_decode(TeR.steer.angle) * ((float)PI / 180.0f);
    float wz         = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z) * ((float)PI / 180.0f);
    float ay_ms2     = ter_accel_a_y_decode(TeR.accel.a_y);
    float vx         = v2_ground_speed_ms();

    /* Decodificación GPS si está disponible */
    float vy_gps     = TeR.gps.vy_gps;
    uint8_t gps_valid = TeR.gps.fix_valid;

    v2_trq_map_t out = v2_tv_step(
        apps_pct, brake_bar, steer_rad, wz, ay_ms2, vx,
        (float)TeR.wheelInfo.rl_rpm, (float)TeR.wheelInfo.rr_rpm,
        vy_gps, gps_valid,
        (float)limit, regen_allowed(), (float)TeR.config.regen_max_trq,
        0.005f,
        &v2_params, &v2_state
    );

    trqMap_t trqMap;
    trqMap.rLeft  = (trq_t)out.rl_nm;
    trqMap.rRight = (trq_t)out.rr_nm;
    return trqMap;
}

trqMap_t tractionControl(trqMap_t in) {
    float vx = v2_ground_speed_ms();
    v2_trq_map_t v2in  = { .rl_nm = (float)in.rLeft, .rr_nm = (float)in.rRight };
    v2_trq_map_t v2out = v2_traction_control_step(
        v2in, (float)TeR.wheelInfo.rl_rpm, (float)TeR.wheelInfo.rr_rpm,
        vx, &v2_params, &v2_state
    );

    trqMap_t out;
    out.rLeft  = (trq_t)v2out.rl_nm;
    out.rRight = (trq_t)v2out.rr_nm;
    return out;
}