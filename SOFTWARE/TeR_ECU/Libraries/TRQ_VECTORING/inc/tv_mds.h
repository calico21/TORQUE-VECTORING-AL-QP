#ifndef TV_MDS_H
#define TV_MDS_H

#include <stdint.h>
#include <stdbool.h>
#include "TeR_CONSTANTS.h"
#include "TeR_CAN.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Unified Vehicle Parameters for Branch 1 */
#define V1_VEHICLE_WB        1.530f   /* Wheelbase [m] */
#define V1_VEHICLE_TW        1.220f   /* Track Width [m] */
#define V1_K_UNDERSTEER      0.0025f  /* Understeer Gradient [rad/(m/s^2)] */

typedef struct {
    float kp_yaw;
    float ki_yaw;
    float k_ff;
    float max_yaw_moment_nm;
    float pi_windup_limit_nm;
    float peak_mu;
    float max_allowable_slip;
    float slip_cut_gain;
} v1_params_t;

typedef struct {
    float error_integral_nm;
    bool  cut_active_rl;
    bool  cut_active_rr;
    bool  initialized;
} v1_state_t;

/* Standard TeR_TRQMANAGER Entry Points (100% Signature Match) */
trqMap_t trqVectoring(trq_t limit);
trqMap_t tractionControl(trqMap_t in);
void tv_reset_state(void);

#ifdef __cplusplus
}
#endif

#endif /* TV_MDS_H */