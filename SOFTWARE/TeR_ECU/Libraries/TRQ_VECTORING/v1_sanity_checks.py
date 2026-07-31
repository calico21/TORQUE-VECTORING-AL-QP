"""
v1_sanity_checks.py — SIL regression + comparative harness for v1-simple-effective.

Build (pure core — zero HAL/DBC dependency by construction):
    gcc -shared -fPIC -O2 -o v1_core.so src/v1_vehicle_dynamics.c -Iinc -lm

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
        ("max_yaw_moment_nm", ctypes.c_float), ("pi_windup_limit_nm", ctypes.c_float),
        ("peak_mu", ctypes.c_float), ("max_allowable_slip", ctypes.c_float),
        ("slip_cut_gain", ctypes.c_float), ("steer_deadzone_rad", ctypes.c_float),
        ("yaw_deadzone_rads", ctypes.c_float), ("max_slew_nm_per_s", ctypes.c_float),
    ]


class V1State(ctypes.Structure):
    _fields_ = [
        ("error_integral_nm", ctypes.c_float), ("trq_prev_rl_nm", ctypes.c_float),
        ("trq_prev_rr_nm", ctypes.c_float), ("cut_active_rl", ctypes.c_uint8),
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
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8, ctypes.c_float,
    ctypes.c_float, ctypes.POINTER(V1Params), ctypes.POINTER(V1State),
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
        (apps_pct, brake_bar, steer_rad, wz, vx, rpm_rl, rpm_rr,
         limit_nm, regen_en, regen_max) = input_generator(t)

        tv_out = v1.v1_tv_step(apps_pct, brake_bar, steer_rad, wz, vx, rpm_rl, rpm_rr,
                                limit_nm, regen_en, regen_max, dt,
                                ctypes.byref(p), ctypes.byref(state))
        tc_out = v1.v1_traction_control_step(tv_out, rpm_rl, rpm_rr, vx,
                                              ctypes.byref(p), ctypes.byref(state))
        rl_log.append(tc_out.rl_nm)
        rr_log.append(tc_out.rr_nm)
        diff_log.append(tc_out.rr_nm - tc_out.rl_nm)

    return np.array(rl_log), np.array(rr_log), np.array(diff_log)


# --- Scenarios: (apps_pct, brake_bar, steer_rad, wz, vx, rpm_rl, rpm_rr, limit_nm, regen_en, regen_max) ---

def scenario_dead_stop_launch(t):
    vx = max(t * 5.0, 0.5)
    rpm = _wheel_rpm(vx)
    return (1.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_lut_speed_taper_regression(t):
    """Regression guard for defect #1: at constant full pedal, torque MUST
    taper as motor RPM climbs."""
    vx = 5.0 + t * 20.0  # <--- Cambia de t * 10.0 a t * 20.0 (llega hasta 65 m/s / ~15,000 RPM de motor)
    rpm = _wheel_rpm(vx)
    return (1.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_mu_split(t):
    vx = 20.0
    rpm_rl = _wheel_rpm(vx)
    rpm_rr = _wheel_rpm(vx * 1.8) if t > 0.5 else rpm_rl
    return (0.8, 0.0, 0.0, 0.0, vx, rpm_rl, rpm_rr, 180.0, 1, 40.0)


def scenario_steer_sensor_glitch(t):
    vx = 20.0
    rpm = _wheel_rpm(vx)
    steer = 3.0 if 0.995 < t < 1.005 else 0.3  # single-sample ~172 deg CAN glitch
    return (0.6, 0.0, steer, 0.4, vx, rpm, rpm, 180.0, 1, 40.0)


def scenario_regen_gate_disallowed(t):
    """Regression guard for defect #5: regen_enabled=0 with brake pressure
    present must NOT produce negative torque, independent of any downstream
    pipeline stage."""
    vx = 15.0
    rpm = _wheel_rpm(vx)
    return (0.0, 30.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 0, 40.0)


def scenario_symmetric_wheelspin(t):
    vx = 10.0
    rpm = _wheel_rpm(vx) * (1.3 if t > 1.0 else 1.0)
    return (1.0, 0.0, 0.0, 0.0, vx, rpm, rpm, 180.0, 1, 40.0)


def evaluate(name, rl, dt, ok=True, extra=""):
    slew = np.max(np.abs(np.diff(rl) / dt)) if len(rl) > 1 else 0.0
    max_trq = np.max(np.abs(rl)) if len(rl) else 0.0
    ok = ok and (max_trq <= 300.0)
    color, status = ("\033[92m", "PASS") if ok else ("\033[91m", "FAIL")
    print(f"{color}{status:<6}\033[0m | {name:<48} | max|T|={max_trq:6.1f} Nm | "
          f"max slew={slew:8.1f} Nm/s {extra}")
    return ok


def translate_gp_scenario(gp_scenario_fn, t):
    """Approximate translation from gp's direct-force scenarios to v1's
    pedal/brake-demand inputs. NOT an equivalence — use only for relative
    agility/chatter comparison, never for absolute torque parity."""
    fx, delta, vx, vy, wz, ay, ax, omega, brake = gp_scenario_fn(t)
    apps_pct = float(np.clip(fx / 3000.0, 0.0, 1.0))
    brake_bar = float(np.clip(brake * 50.0, 0.0, 50.0))
    rpm_rl = omega[2] * (60.0 / (2.0 * np.pi))
    rpm_rr = omega[3] * (60.0 / (2.0 * np.pi))
    return (apps_pct, brake_bar, delta, wz, max(vx, 0.5), rpm_rl, rpm_rr, 180.0, 1, 40.0)


def run_v1_vs_gp_dogfight(time_steps):
    try:
        import master_sanity_checks as gp_ref  # Archivo opcional de la Rama 3
    except ImportError:
        print("\nℹ️ Nota: 'master_sanity_checks' no está presente en esta rama limpia (Branch 1). Omitiendo comparativa cruzada con AL-QP.")
        return

    print("\n" + "=" * 92)
    print("  V1 vs AL-QP — HEAD-TO-HEAD (translated inputs, RELATIVE comparison only)")
    print("=" * 92)

    scenarios = {
        "Limit Slalom": gp_ref.scenario_limit_slalom,
        "Mid-Corner Curb Strike": gp_ref.scenario_mid_corner_curb,
        "High-Speed Step Steer": gp_ref.scenario_step_steer_high_speed,
    }
    dt = time_steps[1] - time_steps[0]

    for name, fn in scenarios.items():
        v1_rl, v1_rr, v1_diff = run_v1_scenario(
            time_steps, lambda t, fn=fn: translate_gp_scenario(fn, t)
        )
        gp_rl, gp_rr, gp_diff, *_ = gp_ref.run_scenario(time_steps, fn)

        v1_agility = np.max(np.abs(v1_diff))
        gp_agility = np.max(np.abs(gp_diff))
        v1_rms = np.std(np.diff(v1_rl) / dt)
        gp_rms = np.std(np.diff(gp_rl) / dt)
        print(f"{name:<28} | v1 Δmax={v1_agility:7.1f} Nm (slewRMS {v1_rms:7.1f}) | "
              f"AL-QP Δmax={gp_agility:7.1f} Nm (slewRMS {gp_rms:7.1f})")


if __name__ == "__main__":
    time_steps = np.linspace(0, 3.0, 600)
    dt = time_steps[1] - time_steps[0]

    print("=" * 92)
    print("  V1-SIMPLE-EFFECTIVE — SIL SANITY SUITE")
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

    print("=" * 92)
    print("All v1 SIL sanity checks completed.\n")

    run_v1_vs_gp_dogfight(time_steps)