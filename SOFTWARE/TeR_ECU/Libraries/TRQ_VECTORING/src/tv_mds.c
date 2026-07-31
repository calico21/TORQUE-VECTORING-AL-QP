/*
 * tv_mds.c — Integration shim for v1-simple-effective. All TeR.* / regen_allowed()
 * coupling lives HERE; v1_vehicle_dynamics.c has none, by design (see its header).
 * Previously this file re-implemented the entire control law a second time —
 * two sources of truth for one algorithm. It now does nothing but marshal data.
 */
#include "tv_mds.h"
#include "TeR_TRQMANAGER.h"   /* regen_allowed() */

static v1_params_t v1_params;
static v1_state_t  v1_state;
static uint8_t     v1_ready = 0;

void tv_reset_state(void) {
    v1_reset_state(&v1_state);
}

static float v1_ground_speed_ms(void) {
    /* Prefer the non-driven front axle — same source gp_interface.c uses.
     * Falling back to the driven-rear-average means TC's slip denominator is
     * measured off the wheels it's protecting: during real wheelspin, vx is
     * overestimated exactly when accuracy matters most. */
    float front_kmh = ter_front_v_front_v_decode(TeR.speed.front_v);
    if (front_kmh > 0.1f) return front_kmh / 3.6f;
    return TeR.wheelInfo.speed / 3.6f;
}

trqMap_t trqVectoring(trq_t limit) {
    if (!v1_ready) {
        v1_init_params(&v1_params);
        tv_reset_state();
        v1_ready = 1;
    }

    float apps_pct  = (float)TeR.apps.apps_av / 255.0f;
    float brake_bar = ter_bpps_bpps_decode(TeR.bpps.bpps);
    float steer_rad = ter_steer_angle_decode(TeR.steer.angle) * ((float)PI / 180.0f);
    float wz        = ter_ang_rate_yaw_rate_z_decode(TeR.angRate.yaw_rate_z) * ((float)PI / 180.0f);
    float vx        = v1_ground_speed_ms();

    v1_trq_map_t out = v1_tv_step(
        apps_pct, brake_bar, steer_rad, wz, vx,
        (float)TeR.wheelInfo.rl_rpm, (float)TeR.wheelInfo.rr_rpm,
        (float)limit, regen_allowed(), (float)TeR.config.regen_max_trq,
        0.005f, /* matches TeR_TRQMANAGER.c task_period */
        &v1_params, &v1_state
    );

    trqMap_t trqMap;
    trqMap.rLeft  = (trq_t)out.rl_nm;
    trqMap.rRight = (trq_t)out.rr_nm;
    return trqMap;
}

trqMap_t tractionControl(trqMap_t in) {
    float vx = v1_ground_speed_ms();
    v1_trq_map_t v1in  = { .rl_nm = (float)in.rLeft, .rr_nm = (float)in.rRight };
    v1_trq_map_t v1out = v1_traction_control_step(
        v1in, (float)TeR.wheelInfo.rl_rpm, (float)TeR.wheelInfo.rr_rpm,
        vx, &v1_params, &v1_state
    );

    trqMap_t out;
    out.rLeft  = (trq_t)v1out.rl_nm;
    out.rRight = (trq_t)v1out.rr_nm;
    return out;
}