"""
v2_sanity_checks.py — Expanded SIL Battery for Branch 2 (EKF + SMC)
Build:
    gcc -shared -fPIC -O2 -o v2_core.so src/v2_vehicle_dynamics.c -Iinc -I../../TeR/Inc -lm
Run:
    python3 v2_sanity_checks.py
"""
import ctypes
import os
import numpy as np

LIB_PATH = os.path.abspath("./v2_core.so")
v2 = ctypes.CDLL(LIB_PATH)

class V2Params(ctypes.Structure):
    _fields_ = [
        ("q_vy", ctypes.c_float), ("q_bz", ctypes.c_float),
        ("r_pseudo_vy", ctypes.c_float), ("r_gps_vy", ctypes.c_float),
        ("lambda_beta", ctypes.c_float), ("lambda_int", ctypes.c_float),
        ("k_smc", ctypes.c_float), ("phi_boundary_base", ctypes.c_float),
        ("k_ff", ctypes.c_float), ("k_ffd", ctypes.c_float),
        ("max_yaw_moment_nm", ctypes.c_float), ("smc_integral_limit", ctypes.c_float),
        ("peak_mu", ctypes.c_float), ("max_allowable_slip", ctypes.c_float),
        ("slip_cut_gain", ctypes.c_float), ("max_slew_nm_per_s", ctypes.c_float),
        ("steer_deadzone_rad", ctypes.c_float), ("yaw_deadzone_rads", ctypes.c_float),
    ]

class V2State(ctypes.Structure):
    _fields_ = [
        ("x_vy", ctypes.c_float), ("x_bz", ctypes.c_float),
        ("P", (ctypes.c_float * 2) * 2), ("beta_est_rad", ctypes.c_float),
        ("smc_surface_integral", ctypes.c_float), ("trq_prev_rl_nm", ctypes.c_float),
        ("trq_prev_rr_nm", ctypes.c_float), ("steer_prev_rad", ctypes.c_float),
        ("steer_dot_filt_rads", ctypes.c_float), ("cut_active_rl", ctypes.c_uint8),
        ("cut_active_rr", ctypes.c_uint8), ("initialized", ctypes.c_uint8),
    ]

class V2TrqMap(ctypes.Structure):
    _fields_ = [("rl_nm", ctypes.c_float), ("rr_nm", ctypes.c_float)]

v2.v2_state_sizeof.restype = ctypes.c_size_t
assert ctypes.sizeof(V2State) == v2.v2_state_sizeof()

v2.v2_tv_step.restype = V2TrqMap
v2.v2_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8,
    ctypes.c_float, ctypes.c_uint8, ctypes.c_float, ctypes.c_float,
    ctypes.POINTER(V2Params), ctypes.POINTER(V2State),
]
v2.v2_traction_control_step.restype = V2TrqMap
v2.v2_traction_control_step.argtypes = [
    V2TrqMap, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.POINTER(V2Params), ctypes.POINTER(V2State),
]

R_WHEEL = 0.2032

def _wheel_rpm(v_ms):
    return (v_ms / R_WHEEL) * (60.0 / (2.0 * np.pi))

def make_default_params():
    p = V2Params()
    v2.v2_init_params(ctypes.byref(p))
    return p

def run_v2_scenario(time_array, input_generator, params=None):
    state = V2State()
    v2.v2_reset_state(ctypes.byref(state))
    p = params if params is not None else make_default_params()
    dt = time_array[1] - time_array[0] if len(time_array) > 1 else 0.005

    rl_log, rr_log, diff_log, beta_log, bz_log = [], [], [], [], []
    for t in time_array:
        (apps_pct, brake_bar, steer_rad, wz, ay_ms2, vx, rpm_rl, rpm_rr,
         vy_gps, gps_valid, limit_nm, regen_en, regen_max) = input_generator(t)

        tv_out = v2.v2_tv_step(apps_pct, brake_bar, steer_rad, wz, ay_ms2, vx, rpm_rl, rpm_rr,
                                vy_gps, gps_valid, limit_nm, regen_en, regen_max, dt,
                                ctypes.byref(p), ctypes.byref(state))
        tc_out = v2.v2_traction_control_step(tv_out, rpm_rl, rpm_rr, vx,
                                              ctypes.byref(p), ctypes.byref(state))
        rl_log.append(tc_out.rl_nm)
        rr_log.append(tc_out.rr_nm)
        diff_log.append(tc_out.rr_nm - tc_out.rl_nm)
        beta_log.append(state.beta_est_rad)
        bz_log.append(state.x_bz)

    return np.array(rl_log), np.array(rr_log), np.array(diff_log), np.array(beta_log), np.array(bz_log)

# --- Scenarios ---

def scenario_dead_stop_launch(t):
    vx = max(t * 5.0, 0.5)
    rpm = _wheel_rpm(vx)
    return (1.0, 0.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 0.0, 0, 180.0, 1, 40.0)

def scenario_oversteer_beta_suppression(t):
    vx = 22.0
    rpm = _wheel_rpm(vx)
    wz = 1.2 if 1.0 < t < 1.8 else 0.2
    ay = 12.0 if 1.0 < t < 1.8 else 1.0
    steer = 0.1 if t < 1.0 else -0.35
    return (0.6, 0.0, steer, wz, ay, vx, rpm, rpm, 0.0, 0, 180.0, 1, 40.0)

def scenario_ekf_gps_fusion(t):
    vx = 20.0
    rpm = _wheel_rpm(vx)
    vy_true = 1.5
    ay = 6.0
    wz = ay / vx
    gps_valid = 1 if t > 1.5 else 0
    return (0.7, 0.0, 0.2, wz, ay, vx, rpm, rpm, vy_true, gps_valid, 180.0, 1, 40.0)

def scenario_gyro_bias_rejection(t):
    """Injects +0.05 rad/s offset in raw IMU yaw rate; EKF must estimate x_bz -> 0.05."""
    vx = 20.0
    rpm = _wheel_rpm(vx)
    bias_true = 0.05
    wz_meas = 0.0 + bias_true
    return (0.5, 0.0, 0.0, wz_meas, 0.0, vx, rpm, rpm, 0.0, 0, 180.0, 1, 40.0)

def scenario_gps_loss_dropout(t):
    """GPS signal drops abruptly at t = 1.5s during a 1.2g turn."""
    vx = 22.0
    rpm = _wheel_rpm(vx)
    ay = 12.0
    wz = ay / vx
    gps_valid = 1 if t < 1.5 else 0
    return (0.8, 0.0, 0.25, wz, ay, vx, rpm, rpm, 1.2, gps_valid, 180.0, 1, 40.0)

def evaluate(name, rl, dt, ok=True, extra=""):
    slew = np.max(np.abs(np.diff(rl) / dt)) if len(rl) > 1 else 0.0
    max_trq = np.max(np.abs(rl)) if len(rl) else 0.0
    ok = ok and (max_trq <= 300.0)
    color, status = ("\033[92m", "PASS") if ok else ("\033[91m", "FAIL")
    print(f"{color}{status:<6}\033[0m | {name:<48} | max|T|={max_trq:6.1f} Nm | "
          f"max slew={slew:8.1f} Nm/s {extra}")
    return ok

if __name__ == "__main__":
    time_steps = np.linspace(0, 3.0, 600)
    dt = time_steps[1] - time_steps[0]

    print("=" * 92)
    print("  V2-INTERMEDIATE (EXPANDED SUITE) — SIL SANITY BATTERY")
    print("=" * 92)

    rl, rr, diff, beta, bz = run_v2_scenario(time_steps, scenario_dead_stop_launch)
    evaluate("A: Dead-Stop Launch (Div/0 Protection)", rl, dt)

    rl, rr, diff, beta, bz = run_v2_scenario(time_steps, scenario_oversteer_beta_suppression)
    beta_deg = np.degrees(np.max(np.abs(beta)))
    ok_beta = beta_deg < 12.0
    evaluate("B: Oversteer Beta Suppression (SMC Control)", rl, dt, ok=ok_beta,
             extra=f"| Max Beta={beta_deg:.2f}° | Max ΔT={np.max(np.abs(diff)):.1f} Nm")

    rl, rr, diff, beta, bz = run_v2_scenario(time_steps, scenario_ekf_gps_fusion)
    vy_est_end = np.tan(beta[-1]) * 20.0
    ok_ekf = abs(vy_est_end - 1.5) < 0.3
    evaluate("C: EKF 2-State Fusion (GPS + Pseudo Vy)", rl, dt, ok=ok_ekf,
             extra=f"| vy_est={vy_est_end:.2f} m/s")

    rl, rr, diff, beta, bz = run_v2_scenario(time_steps, scenario_gyro_bias_rejection)
    bz_conv = bz[-1]
    ok_bz = abs(bz_conv - 0.05) < 0.015
    evaluate("D: EKF Gyro Bias Rejection (bz Estimation)", rl, dt, ok=ok_bz,
             extra=f"| bz_est={bz_conv:.4f} rad/s (Target=0.0500)")

    rl, rr, diff, beta, bz = run_v2_scenario(time_steps, scenario_gps_loss_dropout)
    slew_at_drop = abs(diff[300] - diff[299]) / dt
    ok_drop = slew_at_drop < 3252.3
    evaluate("E: GPS Dropout Smooth Fallback", rl, dt, ok=ok_drop,
             extra=f"| ΔT Slew at Dropout={slew_at_drop:.1f} Nm/s")

    print("=" * 92)
    print("All Branch 2 expanded SIL sanity checks completed.\n")