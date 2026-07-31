"""
compare_v1_v2.py — Head-to-Head Comparative Benchmark: Branch 1 vs. Branch 2

Prerequisites:
    1. Build v1_core.so: gcc -shared -fPIC -O2 -o v1_core.so src/v1_vehicle_dynamics.c -Iinc -I../../TeR/Inc -lm
    2. Build v2_core.so: gcc -shared -fPIC -O2 -o v2_core.so src/v2_vehicle_dynamics.c -Iinc -I../../TeR/Inc -lm
Run:
    python3 compare_v1_v2.py
"""
import ctypes
import os
import numpy as np
import matplotlib.pyplot as plt

# Load Both Shared Libraries
v1_lib = ctypes.CDLL(os.path.abspath("./v1_core.so"))
v2_lib = ctypes.CDLL(os.path.abspath("./v2_core.so"))

# Ctypes Definitions for V1
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

v1_lib.v1_tv_step.restype = V1TrqMap
v1_lib.v1_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8,
    ctypes.c_float, ctypes.c_float, ctypes.POINTER(V1Params), ctypes.POINTER(V1State),
]

# Ctypes Definitions for V2
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

v2_lib.v2_tv_step.restype = V2TrqMap
v2_lib.v2_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8,
    ctypes.c_float, ctypes.c_uint8, ctypes.c_float, ctypes.c_float,
    ctypes.POINTER(V2Params), ctypes.POINTER(V2State),
]

R_WHEEL = 0.2032

def scenario_oversteer_rescue(t):
    """High-speed entry with sudden oversteer snap at t = 1.0s"""
    vx = 22.0
    rpm = (vx / R_WHEEL) * (60.0 / (2.0 * np.pi))
    wz = 0.3 if t < 1.0 else 1.4      # Severe oversteer rotational spike
    ay = 4.0 if t < 1.0 else 14.0     # High lateral g
    steer = 0.15 if t < 1.0 else -0.4  # Counter-steering by driver
    return (0.7, 0.0, steer, wz, ay, vx, rpm, rpm)

def run_head_to_head_benchmark():
    time_steps = np.linspace(0, 3.0, 600)
    dt = time_steps[1] - time_steps[0]

    # Initialize V1
    v1_p = V1Params(); v1_lib.v1_init_params(ctypes.byref(v1_p))
    v1_s = V1State(); v1_lib.v1_reset_state(ctypes.byref(v1_s))

    # Initialize V2
    v2_p = V2Params(); v2_lib.v2_init_params(ctypes.byref(v2_p))
    v2_s = V2State(); v2_lib.v2_reset_state(ctypes.byref(v2_s))

    v1_diff, v2_diff = [], []
    v2_beta_deg = []

    for t in time_steps:
        apps, brake, steer, wz, ay, vx, rpm_rl, rpm_rr = scenario_oversteer_rescue(t)

        # Run V1
        out1 = v1_lib.v1_tv_step(apps, brake, steer, wz, ay, vx, rpm_rl, rpm_rr,
                                 180.0, 1, 40.0, dt, ctypes.byref(v1_p), ctypes.byref(v1_s))
        v1_diff.append(out1.rr_nm - out1.rl_nm)

        # Run V2
        out2 = v2_lib.v2_tv_step(apps, brake, steer, wz, ay, vx, rpm_rl, rpm_rr,
                                 0.0, 0, 180.0, 1, 40.0, dt, ctypes.byref(v2_p), ctypes.byref(v2_s))
        v2_diff.append(out2.rr_nm - out2.rl_nm)
        v2_beta_deg.append(np.degrees(v2_s.beta_est_rad))

    v1_diff = np.array(v1_diff)
    v2_diff = np.array(v2_diff)

    print("\n" + "=" * 80)
    print("  HEAD-TO-HEAD COMPARISON: BRANCH 1 (PI+FF) vs. BRANCH 2 (EKF+SMC)")
    print("=" * 80)
    print(f"Branch 1 Max Delta Torque: {np.max(np.abs(v1_diff)):.1f} Nm")
    print(f"Branch 2 Max Delta Torque: {np.max(np.abs(v2_diff)):.1f} Nm")
    print(f"Branch 2 Peak Estimated Sideslip (Beta): {np.max(np.abs(v2_beta_deg)):.2f}°")
    print(f"Branch 1 Actuator Slew RMS: {np.std(np.diff(v1_diff)/dt):.1f} Nm/s")
    print(f"Branch 2 Actuator Slew RMS: {np.std(np.diff(v2_diff)/dt):.1f} Nm/s")
    print("=" * 80)

    # Plot Results
    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axs[0].plot(time_steps, v1_diff, label="Branch 1 (v1: PI + FF + Fz Bias)", color="#1f77b4", linewidth=2)
    axs[0].plot(time_steps, v2_diff, label="Branch 2 (v2: EKF + SMC)", color="#ff7f0e", linewidth=2)
    axs[0].set_ylabel("Torque Delta [RR - RL] (Nm)")
    axs[0].set_title("Oversteer Rescue Response: Torque Vectoring Action")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(time_steps, v2_beta_deg, label="Branch 2 EKF Estimated Sideslip Angle (Beta)", color="#2ca02c", linewidth=2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Sideslip Angle Beta (Deg)")
    axs[1].set_title("Branch 2 Live State Observer Trajectory")
    axs[1].grid(True)
    axs[1].legend()

    plt.tight_layout()
    os.makedirs("output/graphs", exist_ok=True)
    plt.savefig("output/graphs/head_to_head_v1_vs_v2.png", dpi=300)
    print("Graph saved to output/graphs/head_to_head_v1_vs_v2.png\n")

if __name__ == "__main__":
    run_head_to_head_benchmark()