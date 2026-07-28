import ctypes
import os
import math
import pytest

LIB_PATH = os.path.abspath("./gp_core.so")
gp = ctypes.CDLL(LIB_PATH)

# Oversized C-struct memory buffer allocated to hold tv_state_t safely
class TVStateBuffer(ctypes.Structure):
    _fields_ = [("raw_bytes", ctypes.c_uint8 * 4096)]

class GPRegenLimits(ctypes.Structure):
    _fields_ = [
        ("enable",             ctypes.c_uint8),
        ("max_total_trq",      ctypes.c_float),
        ("max_charge_power_w", ctypes.c_float),
    ]

gp.gp_tv_init.argtypes = [ctypes.POINTER(TVStateBuffer)]

def step_3dof_vehicle_physics(vx, vy, wz, t_rl, t_rr, delta, dt):
    """
    3-DOF Non-linear Single-Track Vehicle Plant Simulation.
    Calculates physical accelerations and integrates state velocities.
    """
    mass = 230.0       # Vehicle mass [kg]
    iz = 120.0         # Yaw moment of inertia [kg*m^2]
    lf = 0.806         # Distance CoG to front axle [m]
    lr = 0.744         # Distance CoG to rear axle [m]
    r_wheel = 0.2032   # Wheel radius [m]
    c_alpha = 25000.0  # Cornering stiffness [N/rad]

    # Driven longitudinal force
    fx = (t_rl + t_rr) / r_wheel
    
    # Calculate wheel slip angles
    vx_safe = max(vx, 0.5)
    alpha_f = delta - (vy + lf * wz) / vx_safe
    alpha_r = (-vy + lr * wz) / vx_safe

    # Lateral forces
    fy_f = c_alpha * alpha_f
    fy_r = c_alpha * alpha_r

    # Equations of Motion
    ax = (fx - fy_f * math.sin(delta)) / mass + wz * vy
    ay = (fy_f * math.cos(delta) + fy_r) / mass - wz * vx
    wz_dot = (fy_f * lf * math.cos(delta) - fy_r * lr + (t_rr - t_rl) * 0.6) / iz

    # Explicit Euler integration
    vx_next = max(vx + ax * dt, 0.1)
    vy_next = vy + ay * dt
    wz_next = wz + wz_dot * dt

    return vx_next, vy_next, wz_next, ax, ay

def test_20min_closed_loop_endurance():
    """
    20-Minute Endurance Test (240,000 continuous control steps at 200 Hz).
    Verifies that filters, integrators, and covariance matrices remain stable.
    """
    state_buf = TVStateBuffer()
    gp.gp_tv_init(ctypes.byref(state_buf))

    # Initial physical vehicle states
    vx, vy, wz = 15.0, 0.0, 0.0
    dt = 0.005          # 200 Hz step size (5 ms)
    duration_s = 1200   # 20 Minutes (1200 seconds)
    total_steps = int(duration_s / dt)

    t_out = (ctypes.c_float * 4)()
    omega = (ctypes.c_float * 4)()
    regen_limits = GPRegenLimits(1, 400.0, 40000.0)  # permissive — this test targets numerical stability, not budget enforcement

    for step_i in range(total_steps):
        t = step_i * dt
        
        # Continuous sinusoidal driver inputs (steering weave + throttle sweep)
        delta_cmd = 0.12 * math.sin(2.0 * math.pi * 0.2 * t)
        fx_cmd = 1000.0 + 500.0 * math.sin(2.0 * math.pi * 0.05 * t)

        w_wheel = vx / 0.2032
        omega[2] = w_wheel  # RL wheel speed
        omega[3] = w_wheel  # RR wheel speed

        # Step C-code controller
        gp.gp_tv_step(
            ctypes.c_float(fx_cmd),
            ctypes.c_float(delta_cmd),
            ctypes.c_float(vx),
            ctypes.c_float(vy),
            ctypes.c_float(wz),
            ctypes.c_float(0.0),            # ay
            ctypes.c_float(0.0),            # ax
            ctypes.byref(omega),
            ctypes.c_float(0.0),            # brake
            ctypes.c_float(50.0),           # temp RL
            ctypes.c_float(50.0),           # temp RR
            ctypes.c_float(0.0),            # vy_gps
            ctypes.c_uint8(0),              # gps_valid
            ctypes.byref(regen_limits),
            ctypes.c_float(dt),
            ctypes.byref(state_buf),
            t_out
        )

        # Step plant physics model
        vx, vy, wz, ax, ay = step_3dof_vehicle_physics(
            vx, vy, wz, t_out[2], t_out[3], delta_cmd, dt
        )

        # Numerical integrity checks
        assert math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz), (
            f"Numerical breakdown at t={t:.2f}s: vx={vx}, vy={vy}, wz={wz}"
        )
        assert abs(vy) < 4.0, f"Vehicle sideslip blew up at t={t:.2f}s: vy={vy:.2f} m/s"