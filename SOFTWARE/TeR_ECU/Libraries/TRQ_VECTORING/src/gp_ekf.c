#include "gp_ekf.h"

#ifndef GP_MU_NOM
#define GP_MU_NOM 1.5f
#endif

void gp_ekf_init(gp_ekf_t* ekf) {
    // Initial State: [vy = 0, bias = 0, mu_rl = GP_MU_NOM, mu_rr = GP_MU_NOM]
    ekf->x[GP_EKF_STATE_VY]    = 0.0f;
    ekf->x[GP_EKF_STATE_BW]    = 0.0f;
    ekf->x[GP_EKF_STATE_MU_RL] = GP_MU_NOM;
    ekf->x[GP_EKF_STATE_MU_RR] = GP_MU_NOM;

    // Initial Covariance Matrix P
    for (int i = 0; i < GP_EKF_NUM_STATES; i++) {
        for (int j = 0; j < GP_EKF_NUM_STATES; j++) {
            ekf->P[i][j] = 0.0f;
        }
    }
    ekf->P[0][0] = 0.10f;   // vy initial variance
    ekf->P[1][1] = 0.01f;   // bias initial variance
    ekf->P[2][2] = 0.10f;   // mu_rl initial variance
    ekf->P[3][3] = 0.10f;   // mu_rr initial variance

    // Process Noise Q (Diagonal)
    ekf->Q[GP_EKF_STATE_VY]    = 0.0500f; // Velocity process noise
    ekf->Q[GP_EKF_STATE_BW]    = 0.0001f; // Gyro drift process noise
    ekf->Q[GP_EKF_STATE_MU_RL] = 0.0010f; // Friction variation rate
    ekf->Q[GP_EKF_STATE_MU_RR] = 0.0010f;

    // Measurement Variances
    ekf->R_gps_vy    = 0.040f; // (0.2 m/s)^2
    ekf->R_pseudo_vy = 0.250f; // (0.45 m/s)^2
    ekf->R_mu        = 0.050f;

    ekf->beta_est     = 0.0f;
    ekf->vy_std       = sqrtf(ekf->P[0][0]);
    ekf->wz_corrected = 0.0f;
}

void gp_ekf_predict(
    gp_ekf_t* ekf, 
    float delta, float ax_filt, float ay_filt, float wz_raw, float vx, 
    float dt
) {
    (void)ax_filt;
    ekf->delta_ref = delta;
    
    float vx_safe = GP_MAX(fabsf(vx), 0.5f);
    ekf->wz_corrected = wz_raw - ekf->x[GP_EKF_STATE_BW];

    // --- Pure Kinematic Propagation (NO ungated vy_ss correction here) ---
    // vy_ss correction happens exclusively in gp_ekf_update_kinematic_ss, 
    // where it is properly saturation-gated.
    float vy_dot = ay_filt - (vx_safe * ekf->wz_corrected);
    ekf->x[GP_EKF_STATE_VY] += vy_dot * dt;
    ekf->x[GP_EKF_STATE_VY] = GP_CLAMP(ekf->x[GP_EKF_STATE_VY], -6.0f, 6.0f);

    // --- Analytical Covariance Prediction for Pure Integrator ---
    float f01 = dt * vx_safe;

    float p00 = ekf->P[0][0];
    float p01 = ekf->P[0][1];
    float p02 = ekf->P[0][2];
    float p03 = ekf->P[0][3];
    float p11 = ekf->P[1][1];

    ekf->P[0][0] = p00 + f01 * (2.0f * ekf->P[1][0]) + (f01 * f01) * p11 + ekf->Q[0];
    ekf->P[0][1] = p01 + f01 * p11;
    ekf->P[0][2] = p02 + f01 * ekf->P[1][2];
    ekf->P[0][3] = p03 + f01 * ekf->P[1][3];
    ekf->P[1][0] = ekf->P[0][1];
    ekf->P[2][0] = ekf->P[0][2];
    ekf->P[3][0] = ekf->P[0][3];

    // Diagonal Process Noise Addition & Hard Covariance Ceiling
    for (int i = 0; i < GP_EKF_NUM_STATES; i++) {
        ekf->P[i][i] += ekf->Q[i];
        ekf->P[i][i] = GP_CLAMP(ekf->P[i][i], 1e-6f, 2.0f);
    }

    ekf->beta_est = atan2f(ekf->x[GP_EKF_STATE_VY], vx_safe);
    
    // PHYSICAL GUARD: Clamp sideslip angle to realistic limits (±30° / ±0.523 rad)
    // to prevent low-speed velocity dips from blowing up beta and re-injecting fake torque demands.
    ekf->beta_est = GP_CLAMP(ekf->beta_est, -0.523f, 0.523f);
    
    ekf->vy_std   = sqrtf(ekf->P[0][0]);
}
// ── Robust Sequential Scalar Measurement Update with Innovation Gating ──
static inline void gp_ekf_scalar_update(gp_ekf_t* ekf, uint8_t state_idx, float z, float R) {
    float y = z - ekf->x[state_idx];             // Innovation (Measurement Residual)
    float S = ekf->P[state_idx][state_idx] + R;  // Innovation Variance (Scalar)

    if (S < 1e-6f) return;

    float std_dev = sqrtf(S);

    // --- ROBUSTNESS GUARD: INNOVATION GATING (Outlier Rejection) ---
    if (fabsf(y) > 3.0f * std_dev) {
        return; 
    }

    float inv_S = 1.0f / S;
    float K[GP_EKF_NUM_STATES];

    // Compute Kalman Gain Vector K = P * H^T / S
    for (int i = 0; i < GP_EKF_NUM_STATES; i++) {
        K[i] = ekf->P[i][state_idx] * inv_S;
    }

    // Update State Vector: x = x + K * y
    for (int i = 0; i < GP_EKF_NUM_STATES; i++) {
        ekf->x[i] += K[i] * y;
    }

    // Update Covariance Matrix: P = (I - K * H) * P
    float P_temp[GP_EKF_NUM_STATES][GP_EKF_NUM_STATES];
    for (int i = 0; i < GP_EKF_NUM_STATES; i++) {
        for (int j = 0; j < GP_EKF_NUM_STATES; j++) {
            P_temp[i][j] = ekf->P[i][j] - K[i] * ekf->P[state_idx][j];
        }
    }

    // Enforce Symmetry and Positive Semi-Definiteness
    for (int i = 0; i < GP_EKF_NUM_STATES; i++) {
        for (int j = 0; j < GP_EKF_NUM_STATES; j++) {
            ekf->P[i][j] = 0.5f * (P_temp[i][j] + P_temp[j][i]);
        }
        ekf->P[i][i] = GP_MAX(ekf->P[i][i], 1e-6f);
    }
}

void gp_ekf_update_gps(gp_ekf_t* ekf, float vy_gps, uint8_t gps_valid) {
    if (!gps_valid) return;
    gp_ekf_scalar_update(ekf, GP_EKF_STATE_VY, vy_gps, ekf->R_gps_vy);
}

void gp_ekf_update_kinematic_ss(gp_ekf_t* ekf, float ay_filt, float wz_raw, float vx) {
    float vx_safe = GP_MAX(fabsf(vx), 0.5f);
    float wz_corr = wz_raw - ekf->x[GP_EKF_STATE_BW];
    
    float vy_ss = (GP_LR * wz_corr) - ((GP_MASS * ay_filt * GP_LF * vx_safe) / (GP_WB * GP_C_ALPHA_R));

    // SANITY GUARD: Hard-bound vy_ss to physical limits (±3.0 m/s) 
    // to prevent synthetic scenario anomalies or sensor glitches from corrupting the estimator.
    vy_ss = GP_CLAMP(vy_ss, -3.0f, 3.0f);

    // Saturation penalty — distrust linear formula past 0.5g
    float ay_norm = fabsf(ay_filt) / 9.81f;
    float saturation_penalty = 1.0f + 8.0f * GP_CLAMP(ay_norm - 0.5f, 0.0f, 2.0f) * GP_CLAMP(ay_norm - 0.5f, 0.0f, 2.0f);
    float r_effective = ekf->R_pseudo_vy * saturation_penalty;

    gp_ekf_scalar_update(ekf, GP_EKF_STATE_VY, vy_ss, r_effective);
    
    // Hard safety bound on state update
    ekf->x[GP_EKF_STATE_VY] = GP_CLAMP(ekf->x[GP_EKF_STATE_VY], -6.0f, 6.0f);
}

void gp_ekf_update_friction(gp_ekf_t* ekf, float mu_meas_rl, float mu_meas_rr, float t_mean_abs) {
    if (t_mean_abs < 20.0f) return;

    gp_ekf_scalar_update(ekf, GP_EKF_STATE_MU_RL, GP_CLAMP(mu_meas_rl, GP_TC_MU_LO, GP_TC_MU_HI), ekf->R_mu);
    gp_ekf_scalar_update(ekf, GP_EKF_STATE_MU_RR, GP_CLAMP(mu_meas_rr, GP_TC_MU_LO, GP_TC_MU_HI), ekf->R_mu);

    // Hard bounds safety guard
    ekf->x[GP_EKF_STATE_MU_RL] = GP_CLAMP(ekf->x[GP_EKF_STATE_MU_RL], GP_TC_MU_LO, GP_TC_MU_HI);
    ekf->x[GP_EKF_STATE_MU_RR] = GP_CLAMP(ekf->x[GP_EKF_STATE_MU_RR], GP_TC_MU_LO, GP_TC_MU_HI);
    ekf->x[GP_EKF_STATE_BW]    = GP_CLAMP(ekf->x[GP_EKF_STATE_BW], -0.10f, 0.10f);
}