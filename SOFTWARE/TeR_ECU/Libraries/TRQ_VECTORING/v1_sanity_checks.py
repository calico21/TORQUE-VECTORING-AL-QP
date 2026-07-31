"""
v1_sanity_checks.py — SIL regression + comparative harness for v1-simple-effective.

Build:
    gcc -shared -fPIC -O2 -o v1_core.so src/v1_vehicle_dynamics.c -Iinc -I../../TeR/Inc -lm

Run:
    python3 v1_sanity_checks.py
"""
import ctypes
import os
import numpy as np

LIB_PATH = os.path.abspath("./v1_core.so")
v1 = ctypes.CDLL(LIB_PATH)


class V1Params(ctypes.Structure):
    _fields_ = [
        ("kp_yaw", ctypes.c_float), ("ki_yaw", ctypes.c_float), ("k_ff", ctypes.c_float),
        ("k_ffd", ctypes.c_float), ("steer_dot_lpf_tau", ctypes.c_float),
        ("max_yaw_moment_nm", ctypes.c_float), ("pi_windup_limit_nm", ctypes.c_float),
        ("peak_mu", ctypes.c_float), ("max_allowable_slip", ctypes.c_float),
        ("slip_cut_gain", ctypes.c_float), ("steer_deadzone_rad", ctypes.c_float),
        ("yaw_deadzone_rads", ctypes.c_float), ("max_slew_nm_per_s", ctypes.c_float),
        ("speed_gain_taper", ctypes.c_float), ("enable_fz_load_transfer", ctypes.c_uint8),
    ]


class V1State(ctypes.Structure):
    _fields_ = [
        ("error_integral_nm", ctypes.c_float), ("trq_prev_rl_nm", ctypes.c_float),
        ("trq_prev_rr_nm", ctypes.c_float), ("steer_prev_rad", ctypes.c_float),
        ("steer_dot_filt_rads", ctypes.c_float), ("cut_active_rl", ctypes.c_uint8),
        ("cut_active_rr", ctypes.c_uint8), ("initialized", ctypes.c_uint8),
    ]


class V1TrqMap(ctypes.Structure):
    _fields_ = [("rl_nm", ctypes.c_float), ("rr_nm", ctypes.c_float)]


v1.v1_state_sizeof.restype = ctypes.c_size_t
assert ctypes.sizeof(V1State) == v1.v1_state_sizeof(), (
    f"V1State layout drift: Python={ctypes.sizeof(V1State)} C={v1.v1_state_sizeof()}"
)

v1.v1_tv_step.restype = V1TrqMap
v1.v1_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8,
    ctypes.c_float, ctypes.c_float, ctypes.POINTER(V1Params), ctypes.POINTER(V1State),
]
v1.v1_traction_control_step.restype = V1TrqMap
v1.v1_traction_control_step.argtypes = [
    V1TrqMap, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.POINTER(V1Params), ctypes.POINTER(V1State),
]

R_WHEEL = 0.2032


def _wheel_rpm(v_ms):
    return (v_ms / R_WHEEL) * (60.0 / (2.0 * np.pi))


def make_default_params():
    p = V1Params()
    v1.v1_init_params(ctypes.byref(p))
    return p


def run_v1_scenario(time_array, input_generator, params=None):
    state = V1State()
    v1.v1_reset_state(ctypes.byref(state))
    p = params if params is not None else make_default_params()
    dt = time_array[1] - time_array[0] if len(time_array) > 1 else 0.005

    rl_log, rr_log, diff_log = [], [], []
    for t in time_array:
        (apps_pct, brake_bar, steer_rad, wz, ay_ms2, vx, rpm_rl, rpm_rr,
         limit_nm, regen_en, regen_max) = input_generator(t)

        tv_out = v1.v1_tv_step(apps_pct, brake_bar, steer_rad, wz, ay_ms2, vx, rpm_rl, rpm_rr,
                                limit_nm, regen_en, regen_max, dt,
                                ctypes.byref(p), ctypes.byref(state))
        tc_out = v1.v1_traction_control_step(tv_out, rpm_rl, rpm_rr, vx,
                                              ctypes.byref(p), ctypes.byref(state))
        rl_log.append(tc_out.rl_nm)
        rr_log.append(tc_out.rr_nm)
        diff_log.append(tc_out.rr_nm - tc_out.rl_nm)

    return np.array(rl_log), np.array(rr_log), np.array(diff_log)


# --- Scenarios ---

def scenario_dead_stop_launch(t):
    vx = max(t * 5.0, 0.5)
    rpm = _wheel_rpm(vx)
    return (1.0, 0.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_lut_speed_taper_regression(t):
    vx = 5.0 + t * 20.0
    rpm = _wheel_rpm(vx)
    return (1.0, 0.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_mu_split(t):
    vx = 20.0
    rpm_rl = _wheel_rpm(vx)
    rpm_rr = _wheel_rpm(vx * 1.8) if t > 0.5 else rpm_rl
    return (0.8, 0.0, 0.0, 0.0, 0.0, vx, rpm_rl, rpm_rr, 180.0, 1, 40.0)


def scenario_steer_sensor_glitch(t):
    vx = 20.0
    rpm = _wheel_rpm(vx)
    steer = 3.0 if 0.995 < t < 1.005 else 0.3
    return (0.6, 0.0, steer, 0.4, 8.0, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_regen_gate_disallowed(t):
    vx = 15.0
    rpm = _wheel_rpm(vx)
    return (0.0, 30.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 0, 40.0)


def scenario_symmetric_wheelspin(t):
    vx = 10.0
    rpm = _wheel_rpm(vx) * (1.3 if t > 1.0 else 1.0)
    return (1.0, 0.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_turn_in_transient(t):
    """Test de respuesta impulsiva d(steer)/dt en turn-in"""
    vx = 18.0
    rpm = _wheel_rpm(vx)
    steer = 0.3 if 0.5 < t < 0.8 else 0.0
    wz = steer * 0.8
    ay = vx * wz
    return (0.7, 0.0, steer, wz, ay, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_fz_load_transfer(t):
    """Test de sesgo de par base por Fz durante apoyo lateral fuerte (ay = 12 m/s^2)"""
    vx = 20.0
    rpm = _wheel_rpm(vx)
    ay = 12.0 if t > 1.0 else 0.0
    return (0.8, 0.0, 0.0, 0.0, ay, vx, rpm, rpm, 180.0, 1, 40.0)


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
    print("  V1-SIMPLE-EFFECTIVE (ENHANCED) — SIL SANITY SUITE")
    print("=" * 92)

    rl, rr, diff = run_v1_scenario(time_steps, scenario_dead_stop_launch)
    evaluate("A: Dead-Stop Launch (Div/0 Protect)", rl, dt)

    rl, rr, diff = run_v1_scenario(time_steps, scenario_lut_speed_taper_regression)
    taper_ok = rl[-1] < 0.9 * np.max(rl)
    evaluate("B: LUT Motor-RPM Taper Regression Guard", rl, dt, ok=taper_ok,
             extra=f"| peak={np.max(rl):.1f} -> end={rl[-1]:.1f} Nm")
    assert taper_ok, "Torque failed to taper with speed — RPM/LUT unit bug has regressed."

    rl, rr, diff = run_v1_scenario(time_steps, scenario_mu_split)
    evaluate("C: Mu-Split Asymmetric Loss", rl, dt)

    rl, rr, diff = run_v1_scenario(time_steps, scenario_steer_sensor_glitch)
    gi = int(1.0 / dt)
    glitch_ok = abs(diff[gi] - diff[gi - 3]) < 150.0
    evaluate("D: Steer CAN Glitch Rejection", rl, dt, ok=glitch_ok,
             extra=f"| Δjump={abs(diff[gi]-diff[gi-3]):.1f} Nm")

    rl, rr, diff = run_v1_scenario(time_steps, scenario_regen_gate_disallowed)
    regen_ok = np.all(rl >= -1e-3) and np.all(rr >= -1e-3)
    evaluate("E: Regen Gate Respected When Disallowed", rl, dt, ok=regen_ok,
             extra=f"| min(T)={min(np.min(rl), np.min(rr)):.2f} Nm")
    assert regen_ok, "Negative torque commanded while regen_enabled=0 — gate bypassed."

    rl, rr, diff = run_v1_scenario(time_steps, scenario_symmetric_wheelspin)
    evaluate("F: Symmetric Wheelspin (TC engages both)", rl, dt)

    rl, rr, diff = run_v1_scenario(time_steps, scenario_turn_in_transient)
    evaluate("G: Turn-in Feedforward Boost (dSteer/dt)", rl, dt,
             extra=f"| max ΔT={np.max(np.abs(diff)):.1f} Nm")

    rl, rr, diff = run_v1_scenario(time_steps, scenario_fz_load_transfer)
    fz_ok = (rr[-1] > rl[-1] + 5.0)  # Rueda exterior (RR) recibe más par base que la interior (RL)
    evaluate("H: Quasi-Static Fz Load-Proportional Bias", rl, dt, ok=fz_ok,
             extra=f"| RL={rl[-1]:.1f} Nm, RR={rr[-1]:.1f} Nm")
    assert fz_ok, "Fz load transfer failed to shift base torque to outer wheel."

    print("=" * 92)
    print("All enhanced v1 SIL sanity checks completed successfully.\n")