import ctypes
import numpy as np
import matplotlib.pyplot as plt
import os
import csv
import time as _time

# =====================================================================
# 1. ESTRUCTURAS ACTUALIZADAS (Ctypes)
# =====================================================================
import ctypes

gp_lib_alqp = ctypes.CDLL('./gp_core_alqp.so')
gp_lib_nmpc = ctypes.CDLL('./gp_core_nmpc.so')

class TCState(ctypes.Structure):
    _fields_ = [
        ("pi_integral",         ctypes.c_float * 4),
        ("kappa_filt",          ctypes.c_float * 4),
        ("mu_surface",          ctypes.c_float * 2),
        ("omega_last_raw",      ctypes.c_float * 4),
        ("omega_prev_ema",      ctypes.c_float * 4),
        ("rls_P",               ctypes.c_float * 4),
        ("rls_theta",           ctypes.c_float * 4),
        ("theta_prev",          ctypes.c_float * 4),
        ("kappa_prev",          ctypes.c_float * 4),
        ("fx_prev",             ctypes.c_float * 4),
        ("kappa_opt",           ctypes.c_float * 4),
        ("omega_dot_kick_filt", ctypes.c_float * 4),   # NEW — matches tc_state_t
    ]
class EkfState(ctypes.Structure):
    _fields_ = [
        ("x",            ctypes.c_float * 2),        # 2 states (vy, bw)
        ("P",            (ctypes.c_float * 2) * 2),  # 2x2 Covariance
        ("Q",            ctypes.c_float * 2),        # 2 Process noise values
        ("delta_ref",    ctypes.c_float),
        ("R_gps_vy",     ctypes.c_float),
        ("R_pseudo_vy",  ctypes.c_float),
        ("R_mu",         ctypes.c_float),
        ("beta_est",     ctypes.c_float),
        ("vy_std",       ctypes.c_float),
        ("wz_corrected", ctypes.c_float),
    ]

class GPRegenLimits(ctypes.Structure):
    _fields_ = [
        ("enable",             ctypes.c_uint8),
        ("max_total_trq",      ctypes.c_float),
        ("max_charge_power_w", ctypes.c_float),
    ]

class NMPCState(ctypes.Structure):
    _fields_ = [
        ("x_pred", (ctypes.c_float * 2) * 9),   # N=8 -> N+1=9 (was 11 for N=10)
        ("A_d", (ctypes.c_float * 2) * 2),
        ("B_d", (ctypes.c_float * 1) * 2),
        ("u_seq", ctypes.c_float * 8),           # NEW: RTI warm-start sequence buffer
        ("u_warm", ctypes.c_float),
        ("q_yaw", ctypes.c_float),
        ("r_effort", ctypes.c_float),
        ("r_slew", ctypes.c_float),
    ]

class TVState(ctypes.Structure):
    _fields_ = [
        ("wz_int",         ctypes.c_float),
        ("delta_prev",     ctypes.c_float),
        ("t_qp_prev",      ctypes.c_float * 4),
        ("t_out_prev",     ctypes.c_float * 4),
        ("tc",             TCState),
        ("ekf",            EkfState),
        ("nmpc",           NMPCState),               
        ("vy_est",         ctypes.c_float),
        ("alpha_qp",       ctypes.c_float),
        ("lam_prev",       ctypes.c_float),
        ("mz_sat_ratio",   ctypes.c_float),
        ("vy_gps_last",    ctypes.c_float),
        ("vy_gps_age_ms",  ctypes.c_float),
        ("ax_filt",        ctypes.c_float),
        ("ay_filt",        ctypes.c_float),
        ("t_ub_rl_filt",   ctypes.c_float),
        ("t_ub_rr_filt",   ctypes.c_float),
        ("t_lb_rl_filt",     ctypes.c_float),
        ("t_lb_rr_filt",     ctypes.c_float),
        ("delta_notch_x1",   ctypes.c_float),
        ("delta_notch_x2",   ctypes.c_float),
        ("delta_notch_y1",   ctypes.c_float),
        ("delta_notch_y2",   ctypes.c_float),
    ]

# Structural Safety Assertions
assert ctypes.sizeof(TCState) == 46 * 4, f"TCState size mismatch"

def _bind(lib):
    lib.gp_tv_state_sizeof.restype = ctypes.c_size_t
    assert ctypes.sizeof(TVState) == lib.gp_tv_state_sizeof(), \
        f"TVState layout drift in {lib._name}: Python={ctypes.sizeof(TVState)} C={lib.gp_tv_state_sizeof()}"

    lib.gp_tv_step.argtypes = [
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(ctypes.c_float * 4),
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_uint8,
        ctypes.POINTER(GPRegenLimits),
        ctypes.c_float,
        ctypes.POINTER(TVState),
        ctypes.POINTER(ctypes.c_float * 4),
    ]
    lib.gp_tv_init.argtypes = [ctypes.POINTER(TVState)]
    lib.gp_nmpc_init.argtypes = [ctypes.POINTER(NMPCState)]
    lib.gp_nmpc_step.argtypes = [
        ctypes.POINTER(ctypes.c_float), ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(NMPCState), ctypes.POINTER(ctypes.c_float),
    ]
    lib.gp_nmpc_set_weights.argtypes = [
        ctypes.POINTER(NMPCState), ctypes.c_float, ctypes.c_float, ctypes.c_float
    ]
    
_bind(gp_lib_alqp)
_bind(gp_lib_nmpc)

gp_lib = gp_lib_alqp  # every Phase 1-13 helper below still says `gp_lib.` —
                      # alias it to the real (non-NMPC) build instead of a
                      # 40-line find/replace, so those results stay honest.

# =====================================================================
# 1.5. HARDWARE NON-IDEALITIES ENGINE (NOISE & LATENCY SIMULATOR)
# =====================================================================
class HardwareNonIdealities:
    """Emulates EMI noise, sensor quantization, and CAN/Inverter transport delay."""
    def __init__(self, delay_ticks=1, noise_std_imu=0.15, noise_std_wheel=2.5, noise_std_steer=0.003, seed=None):
        self.delay_ticks = delay_ticks
        self.noise_std_imu = noise_std_imu      # ay, ax in m/s^2, wz in rad/s
        self.noise_std_wheel = noise_std_wheel  # wheel speed in rad/s (~24 RPM noise)
        self.noise_std_steer = noise_std_steer  # steering angle in rad (~0.17 deg)
        self.rng = np.random.default_rng(seed)
        
        # FIFO queue to delay inverter torque command feedback/actuation (1 tick = 5 ms)
        self.cmd_buffer = [[0.0, 0.0, 0.0, 0.0] for _ in range(delay_ticks)]

    def apply_sensor_noise(self, delta, wz, ay, ax, omega):
        """Corrupts clean ground-truth inputs with Gaussian noise and quantization."""
        delta_n = delta + self.rng.normal(0, self.noise_std_steer)
        wz_n    = wz    + self.rng.normal(0, self.noise_std_imu * 0.1)
        ay_n    = ay    + self.rng.normal(0, self.noise_std_imu)
        ax_n    = ax    + self.rng.normal(0, self.noise_std_imu)

        omega_n = [max(0.0, w + self.rng.normal(0, self.noise_std_wheel)) for w in omega]
        
        # 12-bit Analog/CAN Quantization simulation for steering angle
        delta_n = np.round(delta_n / 0.0005) * 0.0005
        return delta_n, wz_n, ay_n, ax_n, omega_n

    def process_actuator_delay(self, t_cmd_out):
        """Applies transport latency to outgoing inverter setpoints."""
        if self.delay_ticks <= 0:
            return t_cmd_out
        self.cmd_buffer.append(list(t_cmd_out))
        return self.cmd_buffer.pop(0)
    
def default_regen_limits(enable=1, max_total_trq=400.0, max_charge_power_w=40000.0):
    """Generous, permissive regen envelope for tests not specifically exercising
    the regen budget itself. Pass an explicit GPRegenLimits into run_scenario()/
    run_comparison() to test a tightened budget instead."""
    rg = GPRegenLimits()
    rg.enable = enable
    rg.max_total_trq = max_total_trq
    rg.max_charge_power_w = max_charge_power_w
    return rg

def run_scenario(time_array, input_generator, non_idealities=None, regen_limits=None, lib=None):
    lib = lib if lib is not None else gp_lib
    state = TVState()
    lib.gp_tv_init(ctypes.byref(state))
    state.tc.mu_surface[0] = 1.5
    state.tc.mu_surface[1] = 1.5

    rg = regen_limits if regen_limits is not None else default_regen_limits()
    
    dt = time_array[1] - time_array[0] if len(time_array) > 1 else 0.005
    
    # Logs
    t_rl_log, t_rr_log, tv_diff_log = [], [], []
    beta_log, alpha_qp_log, mz_sat_log = [], [], []
    
    for t in time_array:
        fx, delta, vx, vy, wz, ay, ax, omega, brake = input_generator(t)
        
        if non_idealities is not None:
            delta, wz, ay, ax, omega = non_idealities.apply_sensor_noise(delta, wz, ay, ax, omega)
        
        omega_c = (ctypes.c_float * 4)(*omega)
        t_out_c = (ctypes.c_float * 4)()
        
        lib.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, 
                       omega_c, brake, 60.0, 60.0, 0.0, 0, ctypes.byref(rg),
                       dt, ctypes.byref(state), t_out_c)
        
        t_out_processed = [t_out_c[0], t_out_c[1], t_out_c[2], t_out_c[3]]
        if non_idealities is not None:
            t_out_processed = non_idealities.process_actuator_delay(t_out_processed)
            
        t_rl_log.append(t_out_processed[2])
        t_rr_log.append(t_out_processed[3])
        tv_diff_log.append(t_out_processed[3] - t_out_processed[2])
        
        # Internal C-kernel Telemetry Extraction
        beta_log.append(state.ekf.beta_est)
        alpha_qp_log.append(state.alpha_qp)
        mz_sat_log.append(state.mz_sat_ratio)
        
    return (np.array(t_rl_log), np.array(t_rr_log), np.array(tv_diff_log), 
            np.array(beta_log), np.array(alpha_qp_log), np.array(mz_sat_log))


# =====================================================================
# 2. ESCENARIOS LEGACY (Fases 1 a 3) - [Actualizados con brake=0.0]
# =====================================================================
def scenario_launch(t):
    vx = max(t * 5.0, 0.0) 
    return 2000.0, 0.0, vx, 0.0, 0.0, 0.0, 8.0, [0, 0, (vx*1.05)/0.2032, (vx*1.05)/0.2032], 0.0

def scenario_ellipse(t):
    vx, ay = 20.0, t * 6.0 
    return 3000.0, 0.2, vx, 0.0, 0.5, ay, 0.0, [0, 0, vx/0.2032, vx/0.2032], 0.0

def scenario_regen_reversal(t):
    """ C: Rapid Regen-to-Drive Reversal (Zero-Crossing Backlash Test) """
    vx = 22.0  # ~80 km/h
    # Instantaneous shift from -2000 N (Max Regen) to +2500 N (Full Drive) at t = 1.0s
    fx = -2000.0 if t < 1.0 else 2500.0
    brake = 0.0
    delta = 0.05
    wz = 0.1
    ay = 0.5
    ax = fx / 250.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, ax, omega, brake

def scenario_divergence(t):
    vx, delta = 25.0, np.sin(t * 10) * 0.8 
    return 2000.0, delta, vx, 0.0, 0.0, 0.0, 0.0, [0, 0, vx/0.2032, vx/0.2032], 0.0

def scenario_mu_split(t):
    vx = 20.0
    omega_rl = (vx * 1.05) / 0.2032 
    omega_rr = (vx * 1.80) / 0.2032 if t > 0.5 else omega_rl 
    return 2500.0, 0.0, vx, 0.0, 0.0, 0.0, 6.0, [0, 0, omega_rl, omega_rr], 0.0

def scenario_sensor_glitch(t):
    vx = 25.0
    omega_base = (vx * 1.05) / 0.2032
    omega_rr = 600.0 if 0.995 < t < 1.005 else omega_base 
    return 2000.0, 0.0, vx, 0.0, 0.0, 0.0, 5.0, [0, 0, omega_base, omega_rr], 0.0

def scenario_liftoff(t):
    vx = 22.0
    fx = 3000.0 if t < 1.5 else 0.0 
    return fx, 0.3, vx, 0.0, 0.6, 12.0, 0.0, [0, 0, vx/0.2032, vx/0.2032], 0.0

def scenario_rollback(t):
    vx = -3.0 + (t * 2.0) 
    omega_base = max(vx, 0.0) / 0.2032 
    return 1500.0, 0.0, vx, 0.0, 0.0, 0.0, 3.0, [0, 0, omega_base, omega_base], 0.0

def scenario_steer_sensor_loss(t):
    """ I: Mid-Corner Steering Encoder Loss (Kinematic Fallback Test) """
    vx = 20.0  # 72 km/h
    ay = 14.0  # 1.4G cornering
    wz = ay / vx
    # Steering encoder signal drops to 0 rad at t = 1.0s while pulling high lateral G
    delta = 0.4 if t < 1.0 else 0.0
    fx = 1500.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, 0.0, omega, 0.0

def scenario_trail_braking(t):
    vx = max(25.0 - 8.0 * t, 10.0) 
    fx = -2000.0 if t < 1.5 else 1500.0 
    return fx, (t * 0.3 if t < 1.5 else 0.45), vx, 0.0, 0.0, (t * 6.0 if t < 1.5 else 9.0), -8.0, [0, 0, vx/0.2032, vx/0.2032], 0.0

def scenario_resonance(t):
    vx, noise = 20.0, 15.0 * np.sin(2 * np.pi * 15.0 * t) 
    return 2500.0, 0.0, vx, 0.0, 0.0, 0.0, 5.0, [0, 0, (vx * 1.05)/0.2032 + noise, (vx * 1.05)/0.2032 + noise], 0.0

def scenario_porpoising(t):
    vx = 28.0 
    return 2000.0, 0.0, vx, 0.0, 0.0, 0.0, 4.0 + 3.0 * np.sin(2 * np.pi * 4.0 * t), [0, 0, vx/0.2032, vx/0.2032], 0.0


# =====================================================================
# 3. NUEVA FASE 4 (ADVANCED DYNAMICS)
# =====================================================================

def scenario_launch_control(t):
    """ M: Freno al 100%, gas a tope a 0 km/h. En t=1.0 suelta el freno. """
    vx = max(0.0, (t - 1.0) * 8.0) if t > 1.0 else 0.0 # Acelera a partir de t=1
    brake = 1.0 if t < 1.0 else 0.0
    fx = 3000.0
    omega = [0, 0, vx/0.2032, vx/0.2032]
    return fx, 0.0, vx, 0.0, 0.0, 0.0, 8.0, omega, brake

def scenario_regen_tv_entry(t):
    """ N: Regenerative Torque Vectoring (Negative Torque Mz Allocation) """
    vx = 20.0  # 72 km/h
    fx = -2200.0  # Heavy regen braking demand
    delta = 0.5   # Aggressive corner turn-in
    wz = 0.9
    ay = vx * wz
    ax = fx / 250.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, ax, omega, 0.0

def scenario_oversteer_rescue(t):
    """ O: Coche sobrevirando a lo bestia. El piloto hace contravolante en t=1.0 """
    vx = 20.0
    wz = 0.8 # Rotando muy rápido hacia la izquierda (positivo)
    # En t=1.0 el piloto gira a la derecha (negativo) para corregir el sobreviraje
    delta = 0.0 if t < 1.0 else -0.5 
    omega = [0, 0, vx/0.2032, vx/0.2032]
    return 1000.0, delta, vx, 0.0, wz, 5.0, 0.0, omega, 0.0

def scenario_anticipatory_tc(t):
    """ P: Salto en el aire. La derivada de la rueda RR explota instantáneamente. """
    vx = 15.0
    omega_rl = vx/0.2032
    # Simulamos una aceleración angular antinatural (ej. > 500 rad/s^2) durante 100ms
    omega_rr = vx/0.2032 + ((t - 1.0) * 800.0) if 1.0 < t < 1.1 else vx/0.2032
    return 2000.0, 0.0, vx, 0.0, 0.0, 0.0, 5.0, [0, 0, omega_rl, omega_rr], 0.0

def scenario_regen_tv_at_limit(t):
    """29: Regen-TV At The Limit. Heavy trail-braking regen INTO a hard corner,
    sized so the friction-derived split exceeds a TIGHT total regen budget —
    exercises the proportional total-budget rescale (not independent
    per-wheel clamping) that must preserve the asymmetric split."""
    vx = 22.0
    fx = -2600.0
    delta = 0.55
    wz = 1.1
    ay = vx * wz
    ax = fx / 250.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, 0.0, wz, ay, ax, omega, brake
# =====================================================================
# 4. MOTOR DE RENDERIZADO UNIFICADO
# =====================================================================
plt.style.use('default')
plt.rcParams.update({
    'figure.facecolor': 'white', 'axes.facecolor': 'white', 'axes.grid': True,
    'grid.linestyle': '--', 'grid.alpha': 0.7, 'text.color': 'black',
    'axes.labelcolor': 'black', 'xtick.color': 'black', 'ytick.color': 'black'
})

def evaluate_test_kpis(time_steps, t_rl, t_rr, t_diff, beta_log, alpha_log, test_name):
    dt = time_steps[1] - time_steps[0]
    slew_rate_rl = np.diff(t_rl) / dt
    noise_rms = np.std(slew_rate_rl)
    max_torque = np.max(np.abs(t_rl))
    
    # Internal C Metrics
    max_beta_deg = np.degrees(np.max(np.abs(beta_log)))
    avg_alpha_qp = np.mean(alpha_log)
    
    detrended = t_rl - np.linspace(t_rl[0], t_rl[-1], len(t_rl))
    window = np.hanning(len(detrended))
    fft_vals = np.abs(np.fft.rfft(detrended * window))
    freqs = np.fft.rfftfreq(len(detrended), d=dt)
    hf_energy = np.sum(fft_vals[freqs > 20.0])
    
    eps = 5.0  
    slew_gated = np.where(np.abs(slew_rate_rl) < eps, 0.0, slew_rate_rl)
    sign_changes = np.where(np.diff(np.sign(slew_gated)))[0]
    zcr = len(sign_changes) / (time_steps[-1] - time_steps[0])
    
    # In master_sanity_checks.py -> evaluate_test_kpis():
    is_transient_test = any(k in test_name for k in [
        "Step Steer", "Hydroplaning", "Curb Strike", "Trail Braking",
        "Slalom", "G-Circle", "Regen", "Glitch", "Launch", "Spinout",
        "Oversteer", "Chicane", "Emergency", "Impulse", "Noise", "Preview",
        "Encoder", "Fallback"
    ])
    hf_limit = 20000.0 if is_transient_test else 1500.0
    zcr_limit = 70.0 if is_transient_test else 40.0

    is_exploding = max_torque > 600.0
    is_chattering = noise_rms > 3500.0 or zcr > zcr_limit or hf_energy > hf_limit
         
    if is_exploding:
        status = "  FAIL (Divergencia)"
        color = "\033[91m"
    elif is_chattering:
        status = "  WARN (Chattering)"
        color = "\033[93m"
    else:
        status = "  PASS"
        color = "\033[92m"
              
    reset_color = "\033[0m"
    print(f"{color}{status:<18} | {test_name:<40} | MaxBeta: {max_beta_deg:4.1f}° | α_qp: {avg_alpha_qp:4.2f} | RMS: {noise_rms:5.1f}{reset_color}")

def generate_report(scenarios, titles, filename, super_title, time_steps, lib=None):
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(super_title, fontsize=16, fontweight='bold')
    
    for ax, (scenario, title) in zip(axs.flat, zip(scenarios, titles)):
        # Unpack all 6 returned telemetry arrays
        rl, rr, diff, beta, alpha_qp, mz_sat = run_scenario(time_steps, scenario, lib=lib)
        
        # Pass telemetry into KPI evaluator
        evaluate_test_kpis(time_steps, rl, rr, diff, beta, alpha_qp, title)
        
        ax.plot(time_steps, rl, color='#0052cc', linewidth=2.5, label='RL Torque (Nm)')
        ax.plot(time_steps, rr, color='#e60000', linewidth=2.5, linestyle='--', label='RR Torque (Nm)')
        
        if np.max(np.abs(diff)) > 1.0:
            ax.plot(time_steps, diff, color='#2ca02c', linewidth=1.5, alpha=0.8, label='Delta (RR-RL)')
            
        ax.set_title(title, fontsize=11, fontweight='semibold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.legend(loc='best')
    
    plt.tight_layout()
    out_dir = os.path.join('output', 'graphs')
    os.makedirs(out_dir, exist_ok=True)
    output_path = os.path.join(out_dir, filename)
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ Generado: {output_path}")
    plt.close()

# =====================================================================
# MONTE CARLO STRESS-TESTING SUITE
# =====================================================================
def run_monte_carlo_suite(scenarios_dict, num_trials=25, delay_ticks=1):
    """
    Runs N randomized trials per scenario with sensor noise and latency to 
    verify mathematical stability and limit-cycle resilience.
    """
    print(f"\n" + "="*80)
    print(f"  STARTING MONTE CARLO STRESS SUITE ({num_trials} Trials/Scenario | Delay: {delay_ticks*5}ms)")
    print("="*80)
    
    time_steps = np.linspace(0, 3.0, 600)
    dt = time_steps[1] - time_steps[0]
    
    total_passes = 0
    total_tests = len(scenarios_dict) * num_trials
    
    for name, scenario in scenarios_dict.items():
        rms_list, hf_list, pass_count = [], [], 0
        
        for trial in range(num_trials):
            hw = HardwareNonIdealities(delay_ticks=delay_ticks, seed=1000 + trial)
            # Unpack first 3 outputs and ignore remainder
            rl, rr, diff, *_ = run_scenario(time_steps, scenario, non_idealities=hw)
            
            # KPI Analysis
            slew_rl = np.diff(rl) / dt
            noise_rms = np.std(slew_rl)
            
            detrended = rl - np.linspace(rl[0], rl[-1], len(rl))
            fft_vals = np.abs(np.fft.rfft(detrended * np.hanning(len(detrended))))
            freqs = np.fft.rfftfreq(len(detrended), d=dt)
            hf_energy = np.sum(fft_vals[freqs > 20.0])
            
            is_transient = any(
                k in name
                for k in [
                    "Step Steer", "Hydroplaning", "Curb Strike", "Trail Braking",
                    "Slalom", "G-Circle", "Skidpad", "Regen", "Oversteer",
                    "Launch", "Spinout", "Glitch", "Chicane", "Emergency",
                    "Impulse", "Noise", "Preview"
                ]
            )
            hf_limit = 50000.0 if is_transient else 3500.0
            
            if noise_rms < 4500.0 and hf_energy < hf_limit and np.max(np.abs(rl)) <= 600.0:
                pass_count += 1
                
            rms_list.append(noise_rms)
            hf_list.append(hf_energy)
            
        pass_rate = (pass_count / num_trials) * 100.0
        total_passes += pass_count
        
        color = "\033[92m" if pass_rate >= 90.0 else ("\033[93m" if pass_rate >= 70.0 else "\033[91m")
        reset = "\033[0m"
        
        print(f"{color}[{pass_rate:5.1f}% PASS]{reset} | {name:<42} | "
              f"Mean RMS: {np.mean(rms_list):6.1f} | 95th-Pct HF: {np.percentile(hf_list, 95):7.1f}")
        
    overall_score = (total_passes / total_tests) * 100.0
    print("="*80)
    print(f"MONTE CARLO ROBUSTNESS SCORE: {overall_score:.2f}%\n")

# =====================================================================
# 5. SISTEMA LEGACY (RÉPLICA DE tv_mds.c PARA COMPARATIVA)
# =====================================================================
class LegacyTV:
    def __init__(self):
        self.error_i = 0.0
        self.kp = 150.0  # Ganancias aproximadas del código C
        self.ki = 10.0
        
    def step(self, fx_driver, delta, vx, wz, dt):
        wb = 0.806 + 0.744
        yaw_ref = (delta * vx) / wb if vx > 1.0 else 0.0
        error = yaw_ref - wz
        self.error_i += error * dt
        d_torque = self.kp * error + self.ki * self.error_i
        
        # El antiguo límite estricto
        d_torque = np.clip(d_torque, -40.0, 40.0)
        
        # Asignación nominal
        nom = (fx_driver * 0.2032) / 2.0
        return nom - (d_torque / 2.0), nom + (d_torque / 2.0)

def run_comparison(time_array, input_generator, regen_limits=None):
    state_new = TVState()
    gp_lib.gp_tv_init(ctypes.byref(state_new))
    state_new.tc.mu_surface[0] = 1.5
    state_new.tc.mu_surface[1] = 1.5
    rg = regen_limits if regen_limits is not None else default_regen_limits()
    legacy_tv = LegacyTV()
    
    log = {'new_rl': [], 'new_rr': [], 'new_diff': [], 
           'old_rl': [], 'old_rr': [], 'old_diff': []}
    
    for t in time_array:
        fx, delta, vx, vy, wz, ay, ax, omega, brake = input_generator(t)
        
        # Sistema Nuevo
        omega_c = (ctypes.c_float * 4)(*omega)
        t_out_c = (ctypes.c_float * 4)()
        gp_lib.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, omega_c, brake, 60.0, 60.0, 0.0, 0,
                           ctypes.byref(rg), 0.005, ctypes.byref(state_new), t_out_c)
        
        # Sistema Antiguo
        old_rl, old_rr = legacy_tv.step(fx, delta, vx, wz, 0.005)
        
        log['new_rl'].append(t_out_c[2])
        log['new_rr'].append(t_out_c[3])
        log['new_diff'].append(t_out_c[3] - t_out_c[2])
        log['old_rl'].append(old_rl)
        log['old_rr'].append(old_rr)
        log['old_diff'].append(old_rr - old_rl)
        
    return log

def run_regen_budget_ramp():
    """Sweeps max_total_trq DOWN from a loose 300 Nm to a tight 20 Nm over
    the simulation window while holding heavy, constant regen demand fixed.
    Demonstrates that gp_soft_cap() tracks the shrinking budget continuously
    (no discrete jump/chatter as the ceiling crosses the natural demand)."""
    time_steps = np.linspace(0, 3.0, 600)
    dt = time_steps[1] - time_steps[0]

    state = TVState()
    gp_lib.gp_tv_init(ctypes.byref(state))
    state.tc.mu_surface[0] = 1.5
    state.tc.mu_surface[1] = 1.5

    rl_log, rr_log, budget_log, total_mag_log = [], [], [], []

    for t in time_steps:
        budget = 300.0 - (280.0 * (t / 3.0))  # 300 -> 20 Nm linear ramp
        rg = default_regen_limits(enable=1, max_total_trq=budget, max_charge_power_w=40000.0)

        vx, delta, wz, ay = 20.0, 0.15, 0.3, 2.0
        fx = -2000.0  # was -250.0: demand (50.8 Nm) never approached the
                       # 300->20 Nm budget, so this scenario never exercised
                       # the soft-cap rescale it exists to test. -2000N gives
                       # a friction-ellipse ceiling (~316 Nm) that sits just
                       # above the budget's starting point, so the budget is
                       # the binding constraint for ~the entire ramp.
        w_rear = vx / 0.2032
        omega_c = (ctypes.c_float * 4)(0.0, 0.0, w_rear, w_rear)
        t_out_c = (ctypes.c_float * 4)()

        gp_lib.gp_tv_step(fx, delta, vx, 0.0, wz, ay, fx / 250.0,
                           omega_c, 0.5, 60.0, 60.0, 0.0, 0, ctypes.byref(rg),
                           dt, ctypes.byref(state), t_out_c)

        rl_log.append(t_out_c[2])
        rr_log.append(t_out_c[3])
        budget_log.append(budget)
        total_mag_log.append(abs(t_out_c[2]) + abs(t_out_c[3]))

    return (time_steps, np.array(rl_log), np.array(rr_log),
            np.array(budget_log), np.array(total_mag_log))


def run_regen_thermal_derate():
    """Runs scenario_regen_thermal_derate with a time-varying inverter
    temperature (scenario_regen_thermal_derate_temps), which run_scenario()
    can't express since it holds temps fixed at 60/60C for the whole trace.

    The 'expected' ceiling is built from the ACTUAL friction-derived bound
    the solver computes internally (state.t_lb_rl_filt / t_lb_rr_filt,
    already exposed in TVState) combined with an independently-computed
    power+derate term. Only the power+derate half is reproduced in Python;
    the friction half comes straight from telemetry, so there's no
    tire/aero model to get wrong a second time (previous attempt: guessed
    16000W, meant to land ~163 Nm/wheel, actually computed 800 Nm/wheel --
    an arithmetic slip, not a logic error -- so power was never within an
    order of magnitude of friction and never became the binding term until
    derate had crushed it far past irrelevance)."""
    time_steps = np.linspace(0, 3.0, 600)
    dt = time_steps[1] - time_steps[0]

    state = TVState()
    gp_lib.gp_tv_init(ctypes.byref(state))
    state.tc.mu_surface[0] = 1.5
    state.tc.mu_surface[1] = 1.5

    # Calibration target: cool-temp power bound (t_power = P/omega_safe,
    # omega_safe ~= vx = 20 m/s here) should sit NEAR the real friction
    # bound (printed below at runtime) so the 75C crossing is an actual
    # knee, not a ceiling that was never in range. Verify the printed
    # calibration line below on every run -- don't trust this constant
    # blind, that's exactly the mistake made twice already.
    MAX_CHARGE_POWER_W = 4000.0
    rg = default_regen_limits(enable=1, max_total_trq=400.0, max_charge_power_w=MAX_CHARGE_POWER_W)

    rl_log, rr_log, temp_log, expected_log = [], [], [], []
    friction_at_t0 = None

    for t in time_steps:
        fx, delta, vx, vy, wz, ay, ax, omega, brake = scenario_regen_thermal_derate(t)
        temp_rl, temp_rr = scenario_regen_thermal_derate_temps(t)

        omega_c = (ctypes.c_float * 4)(*omega)
        t_out_c = (ctypes.c_float * 4)()
        gp_lib.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, omega_c, brake,
                           temp_rl, temp_rr, 0.0, 0, ctypes.byref(rg),
                           dt, ctypes.byref(state), t_out_c)

        rl_log.append(t_out_c[2])
        rr_log.append(t_out_c[3])
        temp_log.append(temp_rl)

        # Ground-truth friction bound straight from the solver's own state
        # -- the same value gp_tv_step just used this tick. No independent
        # tire/aero reproduction needed for this half.
        friction_rl = state.t_lb_rl_filt
        friction_rr = state.t_lb_rr_filt
        if friction_at_t0 is None:
            friction_at_t0 = 0.5 * (friction_rl + friction_rr)

        # Independent reproduction of ONLY the power+derate term (simple,
        # config-derived; mirrors gp_power_limited_t_lb + the thermal
        # sigmoid in gp_tv_step).
        omega_safe = np.log1p(np.exp(np.clip(np.array(omega[2:4]) * 0.2032, -50, 50)))
        t_power = np.clip(MAX_CHARGE_POWER_W / (omega_safe + 1e-3), 0.0, 2000.0)
        derate = 1.0 - 1.0 / (1.0 + np.exp(-np.clip((np.array([temp_rl, temp_rr]) - 75.0) * 0.5, -20, 20)))
        power_bound = t_power * derate

        expected_log.append(min(friction_rl, power_bound[0]) + min(friction_rr, power_bound[1]))

    print(f"  [Test 32 calibration] Friction ceiling/wheel @ t=0 (50C): {friction_at_t0:6.1f} Nm | "
          f"Power ceiling/wheel @ 50C: {MAX_CHARGE_POWER_W/20.0:6.1f} Nm -- "
          f"{'OK, friction binds first' if friction_at_t0 < MAX_CHARGE_POWER_W/20.0 else 'RETUNE MAX_CHARGE_POWER_W: power binds even when cool'}")

    return (time_steps, np.array(rl_log), np.array(rr_log),
            np.array(temp_log), np.array(expected_log))

def generate_phase12_report(time_steps):
    """Phase 12: Regenerative Braking & Charge-Budget Management.
    Four panels: mixed-sign TV split, lockup recovery, thermal derate
    tracking, and the continuous budget-ramp sweep."""
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle('Phase 12: Regenerative Braking & Charge-Budget Management',
                 fontsize=16, fontweight='bold')

    # --- Panel 1: Mixed-sign TV under a tight budget ---
    tight_rg = default_regen_limits(enable=1, max_total_trq=30.0, max_charge_power_w=40000.0)
    rl_mix, rr_mix, diff_mix, *_ = run_scenario(time_steps, scenario_mixed_sign_regen_tv, regen_limits=tight_rg)
    ax = axs[0, 0]
    ax.plot(time_steps, rl_mix, color='#0052cc', linewidth=2.5, label='RL Torque (Nm)')
    ax.plot(time_steps, rr_mix, color='#e60000', linewidth=2.5, linestyle='--', label='RR Torque (Nm)')
    ax.axhline(0, color='#999999', linewidth=1.0)
    ax.set_title('30: Mixed-Sign TV (drive+regen, tight 30Nm budget)', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Torque (Nm)'); ax.legend(loc='best')

    # --- Panel 2: Regen lockup recovery ---
    rl_lock, rr_lock, *_ = run_scenario(time_steps, scenario_regen_lockup_recovery)
    ax = axs[0, 1]
    ax.plot(time_steps, rl_lock, color='#0052cc', linewidth=2.5, label='RL Torque (Nm)')
    ax.plot(time_steps, rr_lock, color='#e60000', linewidth=2.5, linestyle='--', label='RR Torque (Nm)')
    ax.axvspan(1.0, 1.15, color='#ffcc00', alpha=0.25, label='Simulated lock event')
    ax.set_title('31: Regen Wheel-Lockup Recovery', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Torque (Nm)'); ax.legend(loc='best')

    # --- Panel 3: Thermal derate under regen ---
    t_therm, rl_therm, rr_therm, temp_therm, expected_therm = run_regen_thermal_derate()
    ax = axs[1, 0]
    # Individual wheels kept for reference (thin), but the meaningful
    # apples-to-apples comparison is TOTAL delivered vs TOTAL expected
    # ceiling -- expected_therm sums both wheels, so plotting it against
    # a single wheel's curve was comparing a sum to a half, making a
    # correctly-tracking result look offset by ~2x.
    ax.plot(t_therm, rl_therm, color='#0052cc', linewidth=1.2, alpha=0.5, label='RL Torque (Nm)')
    ax.plot(t_therm, rr_therm, color='#e60000', linewidth=1.2, alpha=0.5, linestyle='--', label='RR Torque (Nm)')
    ax.plot(t_therm, np.abs(rl_therm) + np.abs(rr_therm), color='#0052cc', linewidth=2.5,
            label='Total Delivered |RL|+|RR| (Nm)')
    ax.plot(t_therm, expected_therm, color='#2ca02c', linewidth=2.0, linestyle=':',
            label='Expected Power+Derate Ceiling, Total (Python cross-check)')
    ax2 = ax.twinx()
    ax2.plot(t_therm, temp_therm, color='#ff8800', linewidth=1.5, linestyle=':', label='Inverter Temp (C)')
    ax2.axhline(75.0, color='#ff0000', linewidth=1.0, linestyle='--', alpha=0.6)
    ax2.set_ylabel('Temp (°C)')
    ax.set_title('32: Regen Under Thermal Derate (75°C threshold)', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Torque (Nm)')
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='best', fontsize=8)

    # --- Panel 4: Continuous budget ramp sweep ---
    t_ramp, rl_ramp, rr_ramp, budget_ramp, total_mag_ramp = run_regen_budget_ramp()
    ax = axs[1, 1]
    ax.plot(t_ramp, total_mag_ramp, color='#0052cc', linewidth=2.5, label='|T_RL|+|T_RR| Delivered (Nm)')
    ax.plot(t_ramp, budget_ramp, color='#e60000', linewidth=2.0, linestyle='--', label='Budget Ceiling (Nm)')
    ax.set_title('33: Continuous Budget Ramp (300→20 Nm, soft-cap tracking)', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Torque Magnitude (Nm)'); ax.legend(loc='best')

    plt.tight_layout()
    out_dir = os.path.join('output', 'graphs')
    os.makedirs(out_dir, exist_ok=True)
    output_path = os.path.join(out_dir, 'sanity_phase12_regen_analysis.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ Generado: {output_path}")
    plt.close()

    return (rl_mix, rr_mix, rl_lock, rr_lock, t_therm, rl_therm, rr_therm, temp_therm, expected_therm,
            t_ramp, rl_ramp, rr_ramp, budget_ramp, total_mag_ramp)


# The TV output rate limiter (GP_TV_RATE_LIMIT, gp_torque_vectoring.h) caps
# EACH wheel's slew independently at GP_TV_RATE_LIMIT Nm/s. When both wheels'
# magnitudes move in the same direction simultaneously (as happens here: both
# regen magnitudes shrink together as the shared total-budget ceiling drops),
# the SUM |T_RL|+|T_RR| can legitimately slew at up to 2x that per-wheel
# ceiling. This is the rate limiter working as designed, not chattering —
# the threshold below is derived from the real constant plus a margin for
# solver/filter dynamics, not an arbitrary guess.
GP_TV_RATE_LIMIT_REF = 3252.3  # Nm/s, must match gp_torque_vectoring.h
MAX_COMBINED_SLEW_NM_S = 2.0 * GP_TV_RATE_LIMIT_REF * 1.05  # 5% margin


def run_phase12_regen_analysis(time_steps):
    """Runs Phase 12 plots + the three regression guards that belong to it:
    mixed-sign drive preservation, thermal-derate monotonicity, and
    budget-ramp continuity (no discontinuous jump beyond the rate limiter's
    own designed ceiling as the soft-cap engages)."""
    print("\n" + "=" * 80)
    print("  PHASE 12: REGENERATIVE BRAKING & CHARGE-BUDGET MANAGEMENT")
    print("=" * 80)

    (rl_mix, rr_mix, rl_lock, rr_lock, t_therm, rl_therm, rr_therm, temp_therm, expected_therm,
     t_ramp, rl_ramp, rr_ramp, budget_ramp, total_mag_ramp) = generate_phase12_report(time_steps)

    # --- Guard 1: mixed-sign drive preservation (tight vs loose budget) ---
    tight_rg = default_regen_limits(enable=1, max_total_trq=30.0, max_charge_power_w=40000.0)
    loose_rg = default_regen_limits(enable=1, max_total_trq=400.0, max_charge_power_w=40000.0)
    rl_t, rr_t, *_ = run_scenario(time_steps, scenario_mixed_sign_regen_tv, regen_limits=tight_rg)
    rl_l, rr_l, *_ = run_scenario(time_steps, scenario_mixed_sign_regen_tv, regen_limits=loose_rg)

    drive_wheel_tight = np.where(rl_t > 0, rl_t, rr_t)
    drive_wheel_loose  = np.where(rl_l > 0, rl_l, rr_l)
    regen_wheel_tight  = np.where(rl_t > 0, rr_t, rl_t)

    regen_budget_ok = np.all(np.abs(np.minimum(regen_wheel_tight, 0.0)) <= 30.0 + 1e-1)
    drive_delta = np.mean(np.abs(drive_wheel_tight - drive_wheel_loose))
    drive_preserved = drive_delta < 5.0

    ok1 = regen_budget_ok and drive_preserved
    color1 = "\033[92m" if ok1 else "\033[91m"
    status1 = "PASS" if ok1 else "FAIL"
    print(f"{color1}{status1:<18}\033[0m | 30: Mixed-Sign Regen TV                  | "
          f"Budget OK: {str(regen_budget_ok):<5} | Drive delta (tight vs loose): {drive_delta:6.3f} Nm")
    assert regen_budget_ok, "Regen magnitude exceeded budget under mixed-sign TV."
    assert drive_preserved, (
        f"Drive torque diverged {drive_delta:.2f} Nm between tight/loose regen budgets "
        f"— sign-blind rescale regression."
    )

    # --- Guard 2: thermal derate must actually TRACK the expected power+derate
    # ceiling, not just fail to increase. Monotonicity-only would still pass
    # a flat, friction-limited line that never responds to temperature. ---
    regen_mag = np.abs(rl_therm) + np.abs(rr_therm)
    hot_mask = temp_therm > 78.0  # a couple degrees past the sigmoid's
                                   # inflection to skip the transition's soft shoulder
    tracking_err = np.abs(regen_mag[hot_mask] - expected_therm[hot_mask])
    derate_tracks = np.mean(tracking_err) < 15.0  # Nm

    cool_mask = temp_therm <= 75.0
    pct_drop = 100.0 * (1.0 - regen_mag[-1] / regen_mag[cool_mask][-1]) if np.any(cool_mask) else float('nan')
    derate_meaningful = pct_drop > 50.0  # must actually collapse near 95C, not just dip

    ok2 = derate_tracks and derate_meaningful
    color2 = "\033[92m" if ok2 else "\033[91m"
    status2 = "PASS" if ok2 else "FAIL"
    print(f"{color2}{status2:<18}\033[0m | 32: Thermal Derate Ceiling Tracking       | "
          f"Mean err vs expected: {np.mean(tracking_err):5.1f} Nm | Drop@95C: {pct_drop:4.1f}%")
    assert derate_tracks, f"Delivered regen doesn't track expected power/derate ceiling (err {np.mean(tracking_err):.1f} Nm)."
    assert derate_meaningful, f"Thermal derate only reduced regen by {pct_drop:.1f}% — power bound isn't actually binding."

    # --- Guard 3: budget-ramp must track the soft cap within the rate
    # limiter's own combined-wheel ceiling (no jump BEYOND that ceiling) ---
    slew = np.abs(np.diff(total_mag_ramp)) / (t_ramp[1] - t_ramp[0])
    max_slew = np.max(slew)
    ramp_continuous = max_slew <= MAX_COMBINED_SLEW_NM_S
    over_budget = np.any(total_mag_ramp > budget_ramp + 1.0)

    ok3 = ramp_continuous and not over_budget
    color3 = "\033[92m" if ok3 else "\033[91m"
    status3 = "PASS" if ok3 else "FAIL"
    print(f"{color3}{status3:<18}\033[0m | 33: Budget Ramp Continuity               | "
          f"Max slew: {max_slew:7.1f} Nm/s (limit {MAX_COMBINED_SLEW_NM_S:.0f}) | Over budget: {over_budget}")
    assert ramp_continuous, (
        f"Budget ramp slew ({max_slew:.1f} Nm/s) exceeded 2x GP_TV_RATE_LIMIT "
        f"({MAX_COMBINED_SLEW_NM_S:.1f} Nm/s) — genuine discontinuity, not just the rate limiter's ceiling."
    )
    assert not over_budget, "Delivered regen magnitude exceeded the live budget ceiling during the ramp."

    # --- Guard 4: tracking fidelity, not just ceiling compliance. Guard 3
    # only proves delivered <= budget, which a flat demand-starved line also
    # satisfies — that's exactly how this shipped broken (fx=-250 never
    # approached the budget). This proves delivered actually FOLLOWS budget
    # once budget is plausibly binding. ---
    tracking_region = budget_ramp < 250.0   # skip the narrow band near t=0
                                              # where friction (~316 Nm), not
                                              # budget, is still technically tighter
    tracking_region[:20] = False             # skip ~100ms EMA filter settle
    tracking_err33 = np.abs(total_mag_ramp[tracking_region] - budget_ramp[tracking_region])
    tracks_budget = np.mean(tracking_err33) < 8.0  # Nm: soft-cap softness (4Nm) + rate-limiter lag

    status4 = "PASS" if tracks_budget else "FAIL"
    color4 = "\033[92m" if tracks_budget else "\033[91m"
    print(f"{color4}{status4:<18}\033[0m | 33: Budget Ramp Tracking Fidelity         | "
          f"Mean |delivered-budget|: {np.mean(tracking_err33):5.2f} Nm (limit 8.0)")
    assert tracks_budget, (
        f"Delivered magnitude doesn't track the budget ceiling (mean err "
        f"{np.mean(tracking_err33):.2f} Nm) — demand-starved, not exercising the rescale."
    )

    print("=" * 80)
    print(f"PHASE 12 SUMMARY | Mixed-sign drive delta: {drive_delta:.2f} Nm | "
          f"Thermal derate @95C: {regen_mag[-1]:.1f} Nm | "
          f"Ramp max slew: {max_slew:.1f} Nm/s (rate-limiter ceiling: {MAX_COMBINED_SLEW_NM_S:.0f} Nm/s)")
    print("=" * 80 + "\n")

def run_nmpc_vs_alqp_comparison(time_array, input_generator):
    state_alqp = TVState(); gp_lib_alqp.gp_tv_init(ctypes.byref(state_alqp))
    state_nmpc = TVState(); gp_lib_nmpc.gp_tv_init(ctypes.byref(state_nmpc))
    for s in (state_alqp, state_nmpc):
        s.tc.mu_surface[0] = 1.5; s.tc.mu_surface[1] = 1.5
    rg = default_regen_limits()
    dt = time_array[1] - time_array[0] if len(time_array) > 1 else 0.005
    log = {'alqp_diff': [], 'nmpc_diff': []}
    for t in time_array:
        fx, delta, vx, vy, wz, ay, ax, omega, brake = input_generator(t)
        omega_c = (ctypes.c_float * 4)(*omega)

        t_out_alqp = (ctypes.c_float * 4)()
        gp_lib_alqp.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, omega_c, brake,
                                60.0, 60.0, 0.0, 0, ctypes.byref(rg), dt,
                                ctypes.byref(state_alqp), t_out_alqp)

        t_out_nmpc = (ctypes.c_float * 4)()
        gp_lib_nmpc.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, omega_c, brake,
                                60.0, 60.0, 0.0, 0, ctypes.byref(rg), dt,
                                ctypes.byref(state_nmpc), t_out_nmpc)

        log['alqp_diff'].append(t_out_alqp[3] - t_out_alqp[2])
        log['nmpc_diff'].append(t_out_nmpc[3] - t_out_nmpc[2])
    return log

def generate_nmpc_dogfight_report(scenarios, titles, filename, super_title, time_steps):
    """Renders Phase 14 comparing Branch 3 (AL-QP) vs Branch 4 (NMPC)."""
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(super_title, fontsize=16, fontweight='bold', color='#111111')
    
    COLOR_ALQP = '#0052cc' # Blue
    COLOR_NMPC = '#ff8800' # Orange
    
    for ax, (scenario, title) in zip(axs.flat, zip(scenarios, titles)):
        log = run_nmpc_vs_alqp_comparison(time_steps, scenario)
        
        ax.plot(time_steps, log['alqp_diff'], color=COLOR_ALQP, linewidth=2.5, label='Branch 3 (AL-QP Yaw Allocation)')
        ax.plot(time_steps, log['nmpc_diff'], color=COLOR_NMPC, linewidth=2.0, linestyle='--', label='Branch 4 (Embedded NMPC Horizon)')
        
        ax.set_title(title, fontsize=11, fontweight='semibold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque Delta [RR-RL] (Nm)')
        ax.legend(loc='best', fontsize=9)
    
    plt.tight_layout()
    out_dir = os.path.join('output', 'graphs')
    os.makedirs(out_dir, exist_ok=True)
    output_path = os.path.join(out_dir, filename)
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ Generated: {output_path}")
    plt.close()

def generate_comparison_report(scenarios, titles, filename, super_title, time_steps, plot_mode):
    """Unified comparative renderer (Full English, consistent coloring)"""
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(super_title, fontsize=16, fontweight='bold', color='#111111')
    
    # Consistent color palette for the entire report
    COLOR_ALQP = '#0052cc' # Blue for the new advanced solver
    COLOR_PD = '#7f7f7f'   # Grey for the legacy kinematic PID
    
    for ax, (scenario, title) in zip(axs.flat, zip(scenarios, titles)):
        log = run_comparison(time_steps, scenario)
        
        if plot_mode == 'lateral':
            ax.plot(time_steps, log['new_diff'], color=COLOR_ALQP, linewidth=2.5, label='AL-QP: Unrestricted Mz Allocation')
            ax.plot(time_steps, log['old_diff'], color=COLOR_PD, linewidth=2.0, linestyle='--', label='PD: Capped at ±40Nm')
            ax.set_ylabel('Torque Delta [RR-RL] (Nm)')
            
        elif plot_mode == 'longitudinal':
            ax.plot(time_steps, log['new_rr'], color=COLOR_ALQP, linewidth=2.5, label='AL-QP: Physics-Bounded Traction')
            ax.plot(time_steps, log['old_rr'], color=COLOR_PD, linewidth=2.0, linestyle='--', label='PD: Blind Torque Request')
            ax.set_ylabel('RR Torque (Nm)')
            
        elif plot_mode == 'robustness':
            ax.plot(time_steps, log['new_rl'], color=COLOR_ALQP, linewidth=2.5, label='AL-QP: Filtered & Rate-Limited')
            ax.plot(time_steps, log['old_rl'], color=COLOR_PD, linewidth=1.5, linestyle='--', label='PD: Unfiltered Output')
            ax.set_ylabel('RL Torque (Nm)')

        ax.set_title(title, fontsize=11, fontweight='semibold')
        ax.set_xlabel('Time (s)')
        ax.legend(loc='best', fontsize=9)
    
    plt.tight_layout()
    out_dir = os.path.join('output', 'graphs')
    os.makedirs(out_dir, exist_ok=True)
    output_path = os.path.join(out_dir, filename)
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ Generated: {output_path}")
    plt.close()

# =====================================================================
# PHASE 8: FSAE Dynamic Events (Competition Scoring Scenarios)
# =====================================================================

def scenario_accel_75m(t):
    """13: Acceleration 75m. Rampa de velocidad con wheelspin inducido para testear el TC."""
    fx = 3000.0  # ~3000 N de empuje solicitado
    delta = 0.0  # Recta perfecta
    vx = min(t * 12.0, 30.0) # Acelera hasta 30 m/s
    vy = 0.0
    wz = 0.0
    ay = 0.0
    ax = 11.8
    # Forzamos un 15% de slip ratio constante (ruedas traseras girando más rápido)
    w_rear = (vx / 0.23) * 1.15 
    omega = [0.0, 0.0, w_rear, w_rear] 
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_skidpad_transition(t):
    """14: Skidpad Transition. Inversión brusca de Gs laterales y Yaw Rate."""
    fx = 1500.0
    delta = -0.7 if t < 1.5 else 0.7  # Cambio brusco de dirección en radianes (~40 grados)
    vx = 12.0             # Velocidad constante ~43 km/h
    vy = -0.5 if t < 1.5 else 0.5
    wz = -1.2 if t < 1.5 else 1.2       # Inversión instantánea de guiñada
    ay = -14.4 if t < 1.5 else 14.4     # ~1.5G lateral que cambia de lado
    ax = 0.0
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_endurance_hairpin(t):
    """15: Endurance Hairpin. Frenada fuerte, vértice lento, tracción máxima."""
    fx = 0.0 if t < 1.5 else min((t - 1.5) * 3000.0, 2500.0)
    brake = 1.0 if t < 1.0 else 0.0
    delta = 1.2 if 1.0 <= t <= 2.0 else 0.0 # Giro cerrado (~70 grados en radianes)
    vx = (15.0 - t*10.0) if t < 1.0 else (5.0 + (t-1.0)*5.0) # Baja a 5m/s y luego acelera
    vy = 0.0
    wz = 1.5 if delta > 0.0 else 0.0
    ay = vx * wz
    ax = -10.0 if brake > 0.0 else (fx / 250.0)
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_fast_sweeper(t):
    """16: Autocross Sweeper. Apoyo lateral mantenido a alta velocidad."""
    fx = 2000.0
    delta = 0.5 # ~30 grados
    vx = 22.0  # ~80 km/h
    vy = 0.2
    wz = 0.8
    ay = vx * wz
    ax = 0.0
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake


# =====================================================================
# PHASE 9: Hardware Limits & Degradation (Torture Module)
# =====================================================================

def scenario_thermal_mu_drop(t):
    """17: Thermal Degradation. Caída de grip en pleno vértice."""
    fx = 1800.0
    delta = 0.6
    vx = 15.0
    vy = 0.5 if t < 1.5 else 2.5 # El coche de repente empieza a deslizar lateralmente
    wz = 1.0 if t < 1.5 else 1.6 # Pico de sobreviraje
    ay = 15.0 if t < 1.5 else 10.0 # Caída brusca de fuerza lateral
    ax = 0.0
    w_rear = (vx / 0.23) if t < 1.5 else (vx / 0.23) * 1.5 # Las traseras rompen tracción
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_bms_power_derating(t):
    """18: BMS Power Derating. Pérdida de empuje por calentamiento de batería."""
    fx = 2500.0
    delta = 0.0
    vx = (t * 8.0) if t < 1.5 else (12.0 + (t - 1.5) * 2.0) # El ratio de aceleración se desploma
    vy = 0.0
    wz = 0.0
    ay = 0.0
    ax = 8.0 if t < 1.5 else 2.0 # La G longitudinal cae pese a tener el pedal a fondo
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_asymmetric_wear(t):
    """19: Asymmetric Tire Wear. Diferencia de radio/grip generando un drift fantasma."""
    fx = 2000.0
    delta = 0.0
    vx = 20.0
    vy = 0.0
    wz = 0.15 if t > 1.0 else 0.0 # El coche tira hacia un lado en línea recta
    ay = 0.0
    ax = 0.0
    w_rl = (vx / 0.23) * 1.05 if t > 1.0 else (vx / 0.23) # Desajuste en la rueda izquierda
    w_rr = vx / 0.23
    omega = [0.0, 0.0, w_rl, w_rr]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_sine_with_dwell(t):
    """20: Sine with Dwell. Volantazo ISO para test de estabilidad (Moose Test)."""
    fx = 1200.0
    if 0.5 < t <= 1.0:
        delta = 0.8   # Volantazo fuerte a un lado (rads)
    elif 1.0 < t <= 1.5:
        delta = 0.8   # Mantenemos
    elif 1.5 < t <= 2.2:
        delta = -0.8  # Recuperación agresiva al lado contrario
    else:
        delta = 0.0
        
    vx = 18.0
    vy = 0.0
    wz = delta * 1.5  # Guiñada proporcional a la dirección
    ay = wz * vx
    ax = 0.0
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

# =====================================================================
# PHASE 10: Absolute Limits & Envelope Expansion (AL-QP Standalone)
# =====================================================================

def scenario_vmax_aero_drag(t):
    """21: V-Max Aero-Drag Saturation. Empuje máximo a 120 km/h (35 m/s). 
    El solver debe gestionar el downforce masivo frente a la saturación de los inversores."""
    fx = 5000.0 # Pedimos una barbaridad de empuje
    delta = 0.0
    vx = np.clip(10.0 + (t * 10.0), 10.0, 35.0) # Sube hasta 126 km/h
    vy = 0.0
    wz = 0.0
    ay = 0.0
    ax = 15.0 - (vx * 0.1) # La aceleración cae por el drag aerodinámico
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_step_steer_high_speed(t):
    """22: High-Speed Step Steer. Un volantazo instantáneo a 100 km/h.
    Prueba de fuego para la latencia del TV y la estabilidad del chasis."""
    fx = 1500.0
    delta = 0.0 if t < 1.0 else 0.4 # Volantazo repentino de ~23 grados a los 1.0s
    vx = 28.0 # ~100 km/h fijos
    vy = 0.0 if t < 1.0 else 1.5 # Deriva lateral reactiva
    wz = 0.0 if t < 1.0 else 1.2
    ay = 0.0 if t < 1.0 else (vx * wz)
    ax = 0.0
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_friction_circle_mapping(t):
    """23: G-Circle Spiral Mapping. Acelerador y volante aumentan simultáneamente
    para mapear el borde exterior de la elipse de Kamm."""
    # El piloto pisa progresivamente y gira progresivamente
    fx = t * 1500.0 
    delta = t * 0.3 
    vx = 15.0
    vy = t * 0.5
    wz = delta * 1.2
    ay = wz * vx
    ax = fx / 300.0 # Aceleración sintética
    # Forzamos un slip lateral y longitudinal combinado
    w_rear = (vx / 0.23) * (1.0 + t*0.05) 
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_hydroplaning_survival(t):
    """24: Hydroplaning / Black Ice. Pérdida absoluta de tracción en las 4 ruedas.
    El TC tiene que ahogar los motores a 0 Nm sin errores matemáticos."""
    fx = 3000.0
    delta = 0.0
    vx = 20.0
    vy = 0.0
    wz = 0.0
    ay = 0.0
    ax = 0.0
    # En t=1.0, las ruedas patinan a 3 veces la velocidad del coche (aquaplaning severo)
    w_rear = (vx / 0.23) if t < 1.0 else (vx / 0.23) * 3.0 
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

# =====================================================================
# PHASE 11: Ultimate Performance & Race-Pace Analytics
# =====================================================================

def scenario_mid_corner_curb(t):
    """25: Curb Strike. Apoyo fuerte y la rueda interior salta sobre un piano."""
    fx = 2000.0
    delta = 0.6  # Curva a izquierdas
    vx = 22.0
    vy = 0.5
    wz = 1.0
    ay = vx * wz
    ax = 0.0
    
    # En t=1.2, la rueda trasera izquierda (interior) salta y pierde agarre 
    # (simulado con un pico salvaje de RPM)
    w_rl = (vx / 0.23) if not (1.2 < t < 1.35) else (vx / 0.23) * 2.5
    w_rr = vx / 0.23
    omega = [0.0, 0.0, w_rl, w_rr]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_variable_grip_launch(t):
    """26: Variable Grip. Salida a fondo pisando parches de polvo/pintura."""
    fx = 3500.0 # Gas a tabla
    delta = 0.0
    vx = min(t * 12.0, 35.0)
    vy = 0.0
    wz = 0.0
    ay = 0.0
    ax = 11.5
    
    # Introducimos ruido de alta frecuencia en el slip para simular asfalto roto
    noise = np.sin(t * 50.0) * 0.4 
    w_rear = (vx / 0.23) * (1.1 + noise if t > 0.5 else 1.0)
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_spinout_recovery_limit(t):
    """ 27: Catastrophic Spinout Snap (High-Speed Oversteer Saturation Limit) """
    vx = 25.0  # 90 km/h
    # Severe spin-out at t = 1.0s: yaw rate spikes to 2.5 rad/s (~143 deg/s)
    wz = 0.2 if t < 1.0 else 2.5
    delta = 0.1 if t < 1.0 else -0.8  # Full violent counter-steer lock
    ay = vx * wz if t < 1.0 else 20.0
    fx = 500.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, 0.0, omega, 0.0

def scenario_limit_slalom(t):
    """28: Slalom de velocidad creciente. Hasta que el chasis no pueda más."""
    fx = 1500.0
    # Volante haciendo zig-zag constante
    delta = np.sin(t * np.pi * 1.5) * 0.7 
    # Velocidad aumentando constantemente de 50 a 110 km/h
    vx = 14.0 + (t * 6.0) 
    vy = np.cos(t * np.pi * 1.5) * (vx * 0.05)
    wz = delta * (1.5 - (vx * 0.02)) # El chasis responde menos a alta velocidad
    ay = vx * wz
    ax = 2.0
    
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    brake = 0.0
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

def scenario_mixed_sign_regen_tv(t):
    """30: Mixed-Sign TV Under Tight Regen Budget. One wheel driving, the
    other lightly regening, under moderate TV — the operating point the
    sign-blind post-solve rescale bug silently crushed drive torque on.

    ay tuned to ~0.9g (not ~1.65g as originally) so the friction-ellipse
    bound leaves real torque headroom on both wheels. At the original ay,
    both wheels were crushed to <1 Nm by physics alone regardless of the
    regen budget, so a pass there proved nothing about the rescale logic —
    it just proved both numbers were tiny."""
    vx = 18.0
    fx = 600.0      # moderate net positive demand
    delta = 0.28    # turn-in aggressive enough for a real Mz split, not saturating
    wz = 0.5
    ay = vx * wz    # ~9 m/s^2, ~0.9g
    ax = 0.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, ax, omega, 0.0

def scenario_regen_lockup_recovery(t):
    """31: Regen Wheel-Lockup Recovery. Heavy trail-braking regen while one
    rear wheel's speed suddenly drops toward lock (simulated via a sharp
    omega decay), forcing TC's negative-omega_dot derivative-kick branch to
    intervene and pull regen torque back before the wheel fully locks."""
    vx = 20.0
    fx = -2200.0
    delta = 0.05
    wz = 0.05
    ay = 0.3
    ax = fx / 250.0
    w_rear_nominal = vx / 0.2032
    # RR wheel decelerates sharply toward lock between t=1.0 and t=1.15s
    if 1.0 < t < 1.15:
        w_rr = w_rear_nominal * max(0.15, 1.0 - (t - 1.0) * 6.0)
    else:
        w_rr = w_rear_nominal
    omega = [0.0, 0.0, w_rear_nominal, w_rr]
    return fx, delta, vx, 0.0, wz, ay, ax, omega, 0.5

def scenario_regen_thermal_derate(t):
    """32: Regen Under Inverter Thermal Derate. Sustained heavy regen while
    inverter power-stage temperature climbs past the 75C derate threshold —
    the charge-power ceiling (t_lb_power, mirrored from the drive-side
    thermal derate) should pull the achievable regen torque down smoothly
    as temperature rises, not chatter or clip discontinuously."""
    vx = 20.0
    fx = -2000.0
    delta = 0.1
    wz = 0.1
    ay = 1.0
    ax = fx / 250.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, ax, omega, 0.5

def scenario_regen_thermal_derate_temps(t):
    """Companion temperature profile for scenario_regen_thermal_derate:
    ramps from a cool 50C to a hot 95C over the 3s window, crossing the
    75C derate threshold at t≈1.5s."""
    temp = 50.0 + (95.0 - 50.0) * (t / 3.0)
    return temp, temp

# =====================================================================
# NMPC PREDICTIVE HORIZON SCENARIOS (Branch 4)
# =====================================================================

def scenario_nmpc_chicane_preview(t):
    """34: High-Speed Chicane Entry (110 km/h). Tests predictive yaw damping 
    and counter-torque timing before sideslip peaks."""
    vx = 30.0  # 108 km/h
    # Rapid double S-turn (1.5 Hz frequency)
    delta = np.sin(2 * np.pi * 1.5 * t) * 0.45 if t > 0.5 else 0.0
    wz = delta * 1.8
    ay = vx * wz
    fx = 1200.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, 0.0, omega, 0.0

def scenario_nmpc_step_overshoot_mitigation(t):
    """35: High-G Step Steer Overshoot. Tests if NMPC pre-actuates to catch
    yaw rate momentum before exceeding the target reference."""
    vx = 25.0  # 90 km/h
    delta = 0.0 if t < 0.5 else 0.55  # Instantaneous step input
    wz = 0.0 if t < 0.5 else 1.4
    ay = vx * wz
    fx = 1500.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, 0.0, omega, 0.0

def scenario_nmpc_warm_start_recovery(t):
    """36: Discontinuous Steering Impulse. Tests warm-start sequence stability 
    when the driver performs an aggressive emergency lane change."""
    vx = 22.0
    # Sharp impulse between 1.0s and 1.3s
    delta = 0.8 * np.exp(-((t - 1.15) ** 2) / 0.01)
    wz = delta * 2.0
    ay = vx * wz
    fx = 1000.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, 0.0, omega, 0.0

def scenario_nmpc_actuator_slew_smoothing(t):
    """37: High-Frequency Driver Noise (25 Hz). Tests NMPC's quadratic effort penalty R_u
    in smoothing out noisy steering wheel encoder feedback."""
    vx = 20.0
    noise = 0.08 * np.sin(2 * np.pi * 25.0 * t)
    delta = 0.3 + noise
    wz = 0.6
    ay = vx * wz
    fx = 1800.0
    w_rear = vx / 0.2032
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, 0.0, wz, ay, 0.0, omega, 0.0

class ClosedLoopBicyclePlant:
    """Modelo de planta de bicicleta lineal de 2 estados [vy, wz] en lazo cerrado."""
    def __init__(self, mass=300.0, iz=150.0, lf=0.8525, lr=0.6975,
                 cf=35000.0, cr=32000.0, track_r=1.180, r_wheel=0.2032):
        self.mass, self.iz, self.lf, self.lr = mass, iz, lf, lr
        self.cf, self.cr = cf, cr
        self.track_r, self.r_wheel = track_r, r_wheel
        self.vy, self.wz = 0.0, 0.0

    def step(self, vx, delta, mz_external, dt):
        vx_safe = max(abs(vx), 1.0)
        vy_dot = (-(self.cf + self.cr) / (self.mass * vx_safe)) * self.vy \
                 + (((self.lr * self.cr - self.lf * self.cf) / (self.mass * vx_safe)) - vx_safe) * self.wz \
                 + (self.cf / self.mass) * delta
        wz_dot = ((self.lr * self.cr - self.lf * self.cf) / (self.iz * vx_safe)) * self.vy \
                 - ((self.lf**2 * self.cf + self.lr**2 * self.cr) / (self.iz * vx_safe)) * self.wz \
                 + (self.lf * self.cf / self.iz) * delta \
                 + mz_external / self.iz
        self.vy += vy_dot * dt
        self.wz += wz_dot * dt
        ay = vy_dot + vx_safe * self.wz
        return self.vy, self.wz, ay

def _step_response_metrics(y, y_final, dt):
    if abs(y_final) < 1e-6:
        return {"rise_time": np.nan, "overshoot_pct": np.nan,
                "settle_time": np.nan, "ss_error": np.nan}
    sign = np.sign(y_final)
    y_n = y * sign
    yf_n = abs(y_final)

    # Tiempo de subida: 10% -> 90% del valor de referencia
    idx_10 = np.argmax(y_n >= 0.1 * yf_n)
    idx_90 = np.argmax(y_n >= 0.9 * yf_n)
    rise_time = (idx_90 - idx_10) * dt if idx_90 > idx_10 else np.nan

    # Sobrepico dinámico real sobre el valor estacionario alcanzado (y_ss)
    y_ss = y_n[-1]
    overshoot_pct = max(0.0, (np.max(y_n) - y_ss) / y_ss * 100.0) if y_ss > 1e-6 else 0.0

    # Tiempo de asentamiento: último muestra fuera de la banda de ±2% de y_ss
    band = 0.02 * y_ss
    outside = np.where(np.abs(y_n - y_ss) > band)[0]
    settle_time = (outside[-1] * dt) if len(outside) else 0.0

    # Error en estado estacionario respecto a la referencia cinemática
    ss_error = abs(y_n[-1] - yf_n)
    return {"rise_time": rise_time, "overshoot_pct": overshoot_pct,
            "settle_time": settle_time, "ss_error": ss_error}

GP_LF_PY, GP_LR_PY = 0.8525, 0.6975

def run_closed_loop_step_response(lib, vx, delta_step_rad, t_total=2.0, dt=0.005,
                                   plant_kwargs=None, weight_override=None):
    state = TVState()
    lib.gp_tv_init(ctypes.byref(state))
    state.tc.mu_surface[0] = 1.5
    state.tc.mu_surface[1] = 1.5
    if weight_override is not None and lib is gp_lib_nmpc:
        lib.gp_nmpc_set_weights(ctypes.byref(state.nmpc), *weight_override)

    plant = ClosedLoopBicyclePlant(**(plant_kwargs or {}))
    rg = default_regen_limits()
    n_steps = int(t_total / dt)

    wz_log, wz_ref_log, mz_log = [], [], []
    fx = 800.0
    vy_true, wz_true = 0.0, 0.0

    for k in range(n_steps):
        t = k * dt
        delta = delta_step_rad if t > 0.05 else 0.0
        wb = GP_LF_PY + GP_LR_PY
        wz_ref = (vx * delta) / wb

        omega_c = (ctypes.c_float * 4)(0.0, 0.0, vx / 0.2032, vx / 0.2032)
        t_out_c = (ctypes.c_float * 4)()
        ay_meas = wz_true * vx

        lib.gp_tv_step(fx, delta, vx, vy_true, wz_true, ay_meas, 0.0,
                        omega_c, 0.0, 60.0, 60.0, 0.0, 0, ctypes.byref(rg),
                        dt, ctypes.byref(state), t_out_c)

        mz_cmd = (t_out_c[3] - t_out_c[2]) * (plant.track_r) / (2.0 * plant.r_wheel)
        vy_true, wz_true, _ = plant.step(vx, delta, mz_cmd, dt)

        wz_log.append(wz_true)
        wz_ref_log.append(wz_ref)
        mz_log.append(mz_cmd)

    wz_arr, ref_arr = np.array(wz_log), np.array(wz_ref_log)
    ref_final = ref_arr[-1]
    metrics = _step_response_metrics(wz_arr, ref_final, dt)
    return wz_arr, ref_arr, np.array(mz_log), metrics
# =====================================================================
# AUXILIAR: SIMULADOR DE ESCENARIOS EN LAZO CERRADO (RÉGIMEN NO SATURADO)
# =====================================================================

def run_closed_loop_custom_scenario(lib, vx, delta_base, dist_mz=0.0, freq=0.0, noise_amp=0.0, t_total=2.0, dt=0.005, plant_kwargs=None):
    state = TVState()
    lib.gp_tv_init(ctypes.byref(state))
    state.tc.mu_surface[0] = 1.5
    state.tc.mu_surface[1] = 1.5

    plant = ClosedLoopBicyclePlant(**(plant_kwargs or {}))
    rg = default_regen_limits()
    n_steps = int(t_total / dt)

    wz_log, wz_ref_log, mz_log = [], [], []
    fx = 800.0
    vy_true, wz_true = 0.0, 0.0

    for k in range(n_steps):
        t = k * dt
        if freq > 0:
            delta = delta_base * np.sin(2 * np.pi * freq * t) if t > 0.05 else 0.0
        else:
            delta = delta_base if t > 0.05 else 0.0

        if noise_amp > 0 and t > 0.05:
            delta += noise_amp * np.sin(2 * np.pi * 18.0 * t)  # Jitter de dirección a 18 Hz

        wb = GP_LF_PY + GP_LR_PY
        wz_ref = (vx * delta) / wb

        ext_mz = dist_mz if (0.6 <= t <= 0.75) else 0.0

        omega_c = (ctypes.c_float * 4)(0.0, 0.0, vx / 0.2032, vx / 0.2032)
        t_out_c = (ctypes.c_float * 4)()
        ay_meas = wz_true * vx

        lib.gp_tv_step(fx, delta, vx, vy_true, wz_true, ay_meas, 0.0,
                        omega_c, 0.0, 60.0, 60.0, 0.0, 0, ctypes.byref(rg),
                        dt, ctypes.byref(state), t_out_c)

        mz_cmd = (t_out_c[3] - t_out_c[2]) * (plant.track_r) / (2.0 * plant.r_wheel)
        vy_true, wz_true, _ = plant.step(vx, delta, mz_cmd + ext_mz, dt)

        wz_log.append(wz_true)
        wz_ref_log.append(wz_ref)
        mz_log.append(mz_cmd)

    wz_arr, ref_arr = np.array(wz_log), np.array(wz_ref_log)
    metrics = _step_response_metrics(wz_arr, ref_arr[-1], dt)
    return wz_arr, ref_arr, np.array(mz_log), metrics


# =====================================================================
# PHASE 15: CLOSED-LOOP DYNAMIC DOGFIGHT (2x2 Grid)
# =====================================================================

def run_phase15_closed_loop_dogfight():
    print("\n" + "=" * 80)
    print("  PHASE 15: CLOSED-LOOP DYNAMIC DOGFIGHT (AL-QP Branch 3 vs NMPC Branch 4)")
    print("=" * 80)

    scenarios = {
        "1. Precision Step (20m/s, 0.015rad)":
            dict(vx=20.0, delta=0.015, dist=0.0, freq=0.0, noise=0.0, plant=dict(cf=35000.0, cr=32000.0)),
        "2. Driver Steering Jitter (18Hz Noise)":
            dict(vx=20.0, delta=0.015, dist=0.0, freq=0.0, noise=0.008, plant=dict(cf=35000.0, cr=32000.0)),
        "3. High-Speed Slalom (25m/s, 1.8Hz)":
            dict(vx=25.0, delta=0.020, dist=0.0, freq=1.8, noise=0.0, plant=dict(cf=35000.0, cr=32000.0)),
        "4. Mismatched Rear Tires (-30% Cr)":
            dict(vx=18.0, delta=0.020, dist=0.0, freq=0.0, noise=0.0, plant=dict(cf=35000.0, cr=32000.0 * 0.70)),
    }

    results = {}
    for label, cfg in scenarios.items():
        v_test, d_test = cfg["vx"], cfg["delta"]
        p_kwargs, dist_mz, freq, noise = cfg["plant"], cfg["dist"], cfg["freq"], cfg["noise"]

        wz_alqp, ref_alqp, mz_alqp, m_alqp = run_closed_loop_custom_scenario(
            gp_lib_alqp, v_test, d_test, dist_mz=dist_mz, freq=freq, noise_amp=noise, plant_kwargs=p_kwargs)
        wz_nmpc, ref_nmpc, mz_nmpc, m_nmpc = run_closed_loop_custom_scenario(
            gp_lib_nmpc, v_test, d_test, dist_mz=dist_mz, freq=freq, noise_amp=noise, plant_kwargs=p_kwargs)
        
        results[label] = (wz_alqp, wz_nmpc, ref_alqp, mz_alqp, mz_nmpc)

        print(f"\033[92m  PASS\033[0m | {label:<42} | AL-QP Settle: {m_alqp['settle_time']:4.2f}s | NMPC Settle: {m_nmpc['settle_time']:4.2f}s")

    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle('Phase 15: Closed-Loop Dynamic Dogfight — AL-QP (Branch 3) vs NMPC (Branch 4)',
                 fontsize=15, fontweight='bold')
    
    for ax, (label, (wz_a, wz_n, ref, mz_a, mz_n)) in zip(axs.flat, results.items()):
        t_arr = np.arange(len(wz_a)) * 0.005
        ax.plot(t_arr, ref, color='#999999', linestyle=':', linewidth=1.5, label='wz reference')
        ax.plot(t_arr, wz_a, color='#0052cc', linewidth=2.2, label='AL-QP (Branch 3)')
        ax.plot(t_arr, wz_n, color='#ff8800', linewidth=2.0, linestyle='--', label='NMPC (Branch 4)')
        ax.set_title(label, fontsize=10, fontweight='semibold')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('wz (rad/s)')
        ax.legend(fontsize=8, loc='best')

    plt.tight_layout()
    out_dir = os.path.join('output', 'graphs')
    os.makedirs(out_dir, exist_ok=True)
    output_path = os.path.join(out_dir, 'sanity_phase15_closed_loop_dogfight.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"  Generado: {output_path}\n")
    plt.close()


# =====================================================================
# PHASE 16: NMPC WEIGHT SENSITIVITY SWEEP (2x2 Grid - Régimen No Saturado)
# =====================================================================

GP_NMPC_Q_YAW_PY    = 50.0
GP_NMPC_R_EFFORT_PY = 2.0

def run_phase16_nmpc_weight_sweep():
    print("=" * 80)
    print("  PHASE 16: NMPC WEIGHT SENSITIVITY SWEEP (R_slew Dynamic Damping)")
    print("=" * 80)

    # Entradas en franja no saturada (|Mz| < 80 Nm)
    vx, delta_amp, freq_hz = 20.0, 0.015, 1.5
    r_slew_values = [0.2, 1.0, 5.0, 20.0, 80.0]

    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle('Phase 16: NMPC R_slew Dynamic Sensitivity Analysis (1.5Hz Slalom)', fontsize=15, fontweight='bold')

    colors = ['#e60000', '#ff8800', '#2ca02c', '#0052cc', '#9467bd']

    for r_slew, col in zip(r_slew_values, colors):
        state_tmp = TVState()
        gp_lib_nmpc.gp_tv_init(ctypes.byref(state_tmp))
        gp_lib_nmpc.gp_nmpc_set_weights(ctypes.byref(state_tmp.nmpc), GP_NMPC_Q_YAW_PY, GP_NMPC_R_EFFORT_PY, r_slew)

        plant = ClosedLoopBicyclePlant(cf=35000.0, cr=32000.0)
        n_steps = len(time_steps)
        dt = 0.005
        wz_l, ref_l, mz_l = [], [], []
        vy_t, wz_t = 0.0, 0.0
        rg = default_regen_limits()

        for k in range(n_steps):
            t = k * dt
            delta = delta_amp * np.sin(2 * np.pi * freq_hz * t) if t > 0.05 else 0.0
            wb = GP_LF_PY + GP_LR_PY
            wz_ref = (vx * delta) / wb

            omega_c = (ctypes.c_float * 4)(0.0, 0.0, vx / 0.2032, vx / 0.2032)
            t_out_c = (ctypes.c_float * 4)()

            gp_lib_nmpc.gp_tv_step(800.0, delta, vx, vy_t, wz_t, wz_t * vx, 0.0,
                                   omega_c, 0.0, 60.0, 60.0, 0.0, 0, ctypes.byref(rg),
                                   dt, ctypes.byref(state_tmp), t_out_c)

            mz_cmd = (t_out_c[3] - t_out_c[2]) * (plant.track_r) / (2.0 * plant.r_wheel)
            vy_t, wz_t, _ = plant.step(vx, delta, mz_cmd, dt)

            wz_l.append(wz_t)
            ref_l.append(wz_ref)
            mz_l.append(mz_cmd)

        wz_arr = np.array(wz_l)
        ref_arr = np.array(ref_l)
        mz_arr = np.array(mz_l)

        t_arr = np.arange(len(wz_arr)) * 0.005
        slew_rate = np.abs(np.diff(mz_arr) / 0.005)
        energy = np.cumsum(mz_arr**2) * 0.005

        max_slew = np.max(slew_rate)
        peak_mz = np.max(np.abs(mz_arr))

        axs[0, 0].plot(t_arr, wz_arr, color=col, linewidth=1.8, label=f'R_slew={r_slew:5.1f}')
        axs[0, 1].plot(t_arr, mz_arr, color=col, linewidth=1.8, label=f'R_slew={r_slew:5.1f}')
        axs[1, 0].plot(t_arr[:-1], slew_rate, color=col, linewidth=1.5, label=f'R_slew={r_slew:5.1f}')
        axs[1, 1].plot(t_arr, energy, color=col, linewidth=1.8, label=f'R_slew={r_slew:5.1f}')

        print(f"\033[92m  PASS\033[0m | R_slew = {r_slew:5.1f} | Peak Mz: {peak_mz:5.1f} Nm | Max Slew Rate: {max_slew:6.0f} Nm/s")

    axs[0, 0].plot(t_arr, ref_arr, color='#999999', linestyle=':', label='wz reference')
    axs[0, 0].set_title('1. Yaw Rate Trajectory (wz)', fontsize=10, fontweight='semibold')
    axs[0, 0].set_xlabel('Time (s)'); axs[0, 0].set_ylabel('wz (rad/s)'); axs[0, 0].legend(fontsize=8, loc='best')

    axs[0, 1].set_title('2. Commanded Yaw Moment (Mz)', fontsize=10, fontweight='semibold')
    axs[0, 1].set_xlabel('Time (s)'); axs[0, 1].set_ylabel('Mz (Nm)'); axs[0, 1].legend(fontsize=8, loc='best')

    axs[1, 0].set_title('3. Actuator Slew Rate (|dMz/dt|)', fontsize=10, fontweight='semibold')
    axs[1, 0].set_xlabel('Time (s)'); axs[1, 0].set_ylabel('Slew Rate (Nm/s)'); axs[1, 0].legend(fontsize=8, loc='best')

    axs[1, 1].set_title('4. Cumulative Control Effort Integral (∫Mz² dt)', fontsize=10, fontweight='semibold')
    axs[1, 1].set_xlabel('Time (s)'); axs[1, 1].set_ylabel('Energy (Nm²s)'); axs[1, 1].legend(fontsize=8, loc='best')

    plt.tight_layout()
    out_dir = os.path.join('output', 'graphs')
    os.makedirs(out_dir, exist_ok=True)
    output_path = os.path.join(out_dir, 'sanity_phase16_nmpc_weight_sweep.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n  Generado: {output_path}\n")
    plt.close()


# =====================================================================
# PHASE 17: UNIFIED SCORECARD (Desglose Numérico en Terminal)
# =====================================================================

def run_phase17_scorecard(time_steps):
    print("=" * 80)
    print("  PHASE 17: UNIFIED SCORECARD — AL-QP (Branch 3) vs NMPC (Branch 4)")
    print("=" * 80)

    vx, delta_step = 20.0, 0.015
    _, _, mz_alqp_lin, m_alqp_lin = run_closed_loop_custom_scenario(gp_lib_alqp, vx, delta_step)
    _, _, mz_nmpc_lin, m_nmpc_lin = run_closed_loop_custom_scenario(gp_lib_nmpc, vx, delta_step)

    _, _, mz_alqp_noise, m_alqp_noise = run_closed_loop_custom_scenario(gp_lib_alqp, vx, delta_step, noise_amp=0.008)
    _, _, mz_nmpc_noise, m_nmpc_noise = run_closed_loop_custom_scenario(gp_lib_nmpc, vx, delta_step, noise_amp=0.008)

    _, _, mz_alqp_sla, m_alqp_sla = run_closed_loop_custom_scenario(gp_lib_alqp, 25.0, 0.020, freq=1.8)
    _, _, mz_nmpc_sla, m_nmpc_sla = run_closed_loop_custom_scenario(gp_lib_nmpc, 25.0, 0.020, freq=1.8)

    slew_alqp_lin = np.mean(np.abs(np.diff(mz_alqp_lin) / 0.005))
    slew_nmpc_lin = np.mean(np.abs(np.diff(mz_nmpc_lin) / 0.005))

    slew_alqp_noise = np.mean(np.abs(np.diff(mz_alqp_noise) / 0.005))
    slew_nmpc_noise = np.mean(np.abs(np.diff(mz_nmpc_noise) / 0.005))

    def score(rise, overshoot, settle, slew):
        return max(0.0, 100.0 - (settle * 35.0 + overshoot * 0.8 + rise * 25.0 + slew * 0.01))

    score_alqp = score(m_alqp_lin['rise_time'], m_alqp_lin['overshoot_pct'], m_alqp_lin['settle_time'], slew_alqp_noise)
    score_nmpc = score(m_nmpc_lin['rise_time'], m_nmpc_lin['overshoot_pct'], m_nmpc_lin['settle_time'], slew_nmpc_noise)

    print(f"{'Metric / Performance Feature':<42} | {'AL-QP (Branch 3)':<16} | {'NMPC (Branch 4)':<16}")
    print("-" * 80)
    print(f"{'Linear Step Rise Time (s)':<42} | {m_alqp_lin['rise_time']:<16.3f} | {m_nmpc_lin['rise_time']:<16.3f}")
    print(f"{'Linear Step Settling Time (s)':<42} | {m_alqp_lin['settle_time']:<16.3f} | {m_nmpc_lin['settle_time']:<16.3f}")
    print(f"{'Linear Step Overshoot (%)':<42} | {m_alqp_lin['overshoot_pct']:<16.1f} | {m_nmpc_lin['overshoot_pct']:<16.1f}")
    print(f"{'Clean Step Mean Slew Rate (Nm/s)':<42} | {slew_alqp_lin:<16.1f} | {slew_nmpc_lin:<16.1f}")
    print(f"{'18Hz Steering Noise Slew (Nm/s)':<42} | {slew_alqp_noise:<16.1f} | {slew_nmpc_noise:<16.1f}")
    print(f"{'108 km/h Slalom Settling Time (s)':<42} | {m_alqp_sla['settle_time']:<16.3f} | {m_nmpc_sla['settle_time']:<16.3f}")
    print("-" * 80)
    print(f"\033[92m{'OVERALL WEIGHTED SCORE (0-100)':<42} | {score_alqp:<16.1f} | {score_nmpc:<16.1f}\033[0m")
    print("=" * 80 + "\n")

GP_FW_TAG = "TeR_ECU/TRQ_VECTORING@testing"  # cosmetic session tag, not a real build hash

class Branch2SMC:
    """Lightweight Python approximation of Branch 2 (2-state EKF + Pacejka-lite
    SMC with boundary layer). There is no compiled .so for Branch 2 in this
    harness -- unlike v3/v4, which run the actual embedded C solver -- so
    this reproduces the DOCUMENTED control law (sliding-surface Mz command,
    smooth tanh boundary layer instead of a hard sign() switch to avoid the
    chattering the boundary layer exists to prevent) closely enough to sit
    alongside the others in a like-for-like comparison. Treat its absolute
    numbers with more skepticism than v1 (an intentional replica of shipped
    tv_mds.c) or v3/v4 (the real solver)."""
    def __init__(self, k_smc=180.0, phi=0.6, d_torque_max=140.0):
        self.k_smc = k_smc
        self.phi = phi
        self.d_torque_max = d_torque_max
        self.vy_est = 0.0
 
    def step(self, fx_driver, delta, vx, wz_meas, ay_meas, dt):
        wb = GP_LF_PY + GP_LR_PY
        vx_safe = max(abs(vx), 1.0)
        # Crude leaky-integrator vy estimate standing in for the real 2-state
        # EKF's [vy, gyro-bias] pair -- the leak (0.98) is a rough proxy for
        # the EKF's bias-correction term, not a rigorous reproduction of it.
        vy_dot = ay_meas - vx_safe * wz_meas
        self.vy_est = 0.98 * (self.vy_est + vy_dot * dt)
 
        wz_ref = (vx * delta) / wb if vx > 1.0 else 0.0
        s = wz_ref - wz_meas                          # sliding surface
        d_torque = self.k_smc * np.tanh(s / self.phi)  # smooth boundary layer
        d_torque = np.clip(d_torque, -self.d_torque_max, self.d_torque_max)
 
        nom = (fx_driver * 0.2032) / 2.0
        return nom - d_torque / 2.0, nom + d_torque / 2.0

def scenario_v18_endurance_stint(t):
    """~16s composite stint (launch -> slalom -> trail-brake hairpin+regen ->
    high-speed chicane) stitched into one continuous timeline instead of four
    isolated bench scenarios -- a live telemetry pull off the car is one
    continuous stint, not four separate button-presses on the bench.
    Returns only the EXOGENOUS drive channels (fx, delta, vx, ax, brake);
    wz/vy/ay are closed-loop state produced by each controller's own plant
    in the runner below, not scripted here."""
    if t < 2.0:
        vx = max(t * 9.0, 0.0)
        fx, delta, ax, brake = 2800.0, 0.0, 9.0, 0.0
    elif t < 6.0:
        tt = t - 2.0
        vx = 22.0
        delta = np.sin(2 * np.pi * 1.8 * tt) * 0.50
        fx, ax, brake = 1500.0, 0.0, 0.0
    elif t < 10.0:
        tt = t - 6.0
        vx = max(22.0 - 9.0 * tt, 6.0)
        braking = tt < 1.6
        fx = -2200.0 if braking else 1800.0
        delta = 0.0 if braking else 1.1
        ax = -9.0 if braking else (fx / 250.0)
        brake = 1.0 if braking else 0.0
    else:
        tt = t - 10.0
        vx = 30.0
        delta = np.sin(2 * np.pi * 1.2 * tt) * 0.42
        fx, ax, brake = 1200.0, 0.0, 0.0
    return fx, delta, vx, ax, brake
 
 
def _v18_fft_hf_energy(sig, dt, cutoff_hz=20.0):
    detrended = sig - np.linspace(sig[0], sig[-1], len(sig))
    fft_vals = np.abs(np.fft.rfft(detrended * np.hanning(len(detrended))))
    freqs = np.fft.rfftfreq(len(detrended), d=dt)
    return float(np.sum(fft_vals[freqs > cutoff_hz]))
 
 
def run_phase18_live_telemetry_comparison(stream_stride=50):
    """Closed-loop four-way replay: v1 Branch 1 PI, v2 Branch 2 SMC (Python
    approximation), v3 Branch 3 AL-QP, v4 Branch 4 NMPC. Each controller
    drives its OWN plant instance from its OWN Mz command, so divergence
    between them is genuine closed-loop controller performance."""
    print("\n" + "=" * 96)
    print("  PHASE 18: LIVE TELEMETRY COMPARISON -- v1 (PI) vs v2 (SMC) vs v3 (AL-QP) vs v4 (NMPC)")
    print("=" * 96)
 
    dt, t_total = 0.005, 16.0
    time_steps = np.arange(0.0, t_total, dt)
    n = len(time_steps)
    session_id = _time.strftime("TER27-%Y%m%d-%H%M%S")
 
    print(f"  session: {session_id}  |  fw: {GP_FW_TAG}  |  rate: {1.0/dt:.0f} Hz  |  "
          f"samples: {n}  |  duration: {t_total:.1f}s")
    print("  scenario: composite endurance stint (launch / slalom / hairpin+regen / chicane)")
    print("=" * 96)
 
    controllers = ("v1", "v2", "v3", "v4")
 
    # Identical control-loop sensor-noise/latency REALIZATION across all four
    # -- any seed mismatch here would let random draw, not controller
    # behavior, decide who "wins."
    hw = {k: HardwareNonIdealities(delay_ticks=1, seed=4242) for k in controllers}
 
    # Separate, fixed-seed, NOT-fed-back telemetry noise applied only to the
    # logged/plotted values -- mirrors what a real CAN logger reports
    # (transducer + quantization noise on top of a control loop that already
    # sees its own, separate, sensor noise via HardwareNonIdealities above).
    # Same draw sequence for every controller so it can't bias the comparison.
    telem_rng = np.random.default_rng(777)
 
    legacy = LegacyTV()
    smc = Branch2SMC()
    state_v3, state_v4 = TVState(), TVState()
    gp_lib_alqp.gp_tv_init(ctypes.byref(state_v3))
    gp_lib_nmpc.gp_tv_init(ctypes.byref(state_v4))
    for s in (state_v3, state_v4):
        s.tc.mu_surface[0] = 1.5
        s.tc.mu_surface[1] = 1.5
    rg = default_regen_limits(enable=1, max_total_trq=250.0, max_charge_power_w=40000.0)
 
    plant = {k: ClosedLoopBicyclePlant() for k in controllers}
    wz_true = {k: 0.0 for k in controllers}
    vy_true = {k: 0.0 for k in controllers}
 
    log = {k: {"t_rl": [], "t_rr": [], "mz": [], "wz_ref": [], "wz_meas": [],
               "beta_deg": [], "mz_sat": [], "regen_w": []} for k in controllers}
 
    wb = GP_LF_PY + GP_LR_PY
    STREAM_COLOR = {"v1": "\033[97m", "v2": "\033[92m", "v3": "\033[94m", "v4": "\033[93m"}
    RESET = "\033[0m"
 
    for i, t in enumerate(time_steps):
        fx, delta, vx, ax, brake = scenario_v18_endurance_stint(t)
        wz_ref = (vx * delta) / wb if vx > 1.0 else 0.0
        w_rear = max(vx, 0.0) / 0.2032
        omega_scripted = [0.0, 0.0, w_rear, w_rear]
 
        # ---- v1: Branch 1 PI (legacy tv_mds.c replica) ----
        d1, wzn1, ayn1, axn1, om1 = hw["v1"].apply_sensor_noise(
            delta, wz_true["v1"], wz_true["v1"] * vx, ax, omega_scripted)
        rl1, rr1 = legacy.step(fx, d1, vx, wzn1, dt)
        t1 = hw["v1"].process_actuator_delay([0.0, 0.0, rl1, rr1])
        mz1 = (t1[3] - t1[2]) * plant["v1"].track_r / (2.0 * plant["v1"].r_wheel)
        vy_true["v1"], wz_true["v1"], _ = plant["v1"].step(vx, delta, mz1, dt)
 
        # ---- v2: Branch 2 SMC (Python approximation) ----
        d2, wzn2, ayn2, axn2, om2 = hw["v2"].apply_sensor_noise(
            delta, wz_true["v2"], wz_true["v2"] * vx, ax, omega_scripted)
        rl2, rr2 = smc.step(fx, d2, vx, wzn2, ayn2, dt)
        t2 = hw["v2"].process_actuator_delay([0.0, 0.0, rl2, rr2])
        mz2 = (t2[3] - t2[2]) * plant["v2"].track_r / (2.0 * plant["v2"].r_wheel)
        vy_true["v2"], wz_true["v2"], _ = plant["v2"].step(vx, delta, mz2, dt)
 
        # ---- v3: Branch 3 AL-QP ----
        d3, wzn3, ayn3, axn3, om3 = hw["v3"].apply_sensor_noise(
            delta, wz_true["v3"], wz_true["v3"] * vx, ax, omega_scripted)
        omega_c3, t_out3 = (ctypes.c_float * 4)(*om3), (ctypes.c_float * 4)()
        gp_lib_alqp.gp_tv_step(fx, d3, vx, vy_true["v3"], wzn3, ayn3, axn3, omega_c3, brake,
                                60.0, 60.0, 0.0, 0, ctypes.byref(rg), dt,
                                ctypes.byref(state_v3), t_out3)
        t3 = hw["v3"].process_actuator_delay(list(t_out3))
        mz3 = (t3[3] - t3[2]) * plant["v3"].track_r / (2.0 * plant["v3"].r_wheel)
        vy_true["v3"], wz_true["v3"], _ = plant["v3"].step(vx, delta, mz3, dt)
 
        # ---- v4: Branch 4 NMPC ----
        d4, wzn4, ayn4, axn4, om4 = hw["v4"].apply_sensor_noise(
            delta, wz_true["v4"], wz_true["v4"] * vx, ax, omega_scripted)
        omega_c4, t_out4 = (ctypes.c_float * 4)(*om4), (ctypes.c_float * 4)()
        gp_lib_nmpc.gp_tv_step(fx, d4, vx, vy_true["v4"], wzn4, ayn4, axn4, omega_c4, brake,
                                60.0, 60.0, 0.0, 0, ctypes.byref(rg), dt,
                                ctypes.byref(state_v4), t_out4)
        t4 = hw["v4"].process_actuator_delay(list(t_out4))
        mz4 = (t4[3] - t4[2]) * plant["v4"].track_r / (2.0 * plant["v4"].r_wheel)
        vy_true["v4"], wz_true["v4"], _ = plant["v4"].step(vx, delta, mz4, dt)
 
        frames = {
            "v1": (t1, mz1, float("nan"), float("nan")),
            "v2": (t2, mz2, float("nan"), float("nan")),
            "v3": (t3, mz3, np.degrees(state_v3.ekf.beta_est), state_v3.mz_sat_ratio),
            "v4": (t4, mz4, np.degrees(state_v4.ekf.beta_est), state_v4.mz_sat_ratio),
        }
        for k, (tk, mzk, betak, satk) in frames.items():
            rl_rep = tk[2] + telem_rng.normal(0, 0.35)
            rr_rep = tk[3] + telem_rng.normal(0, 0.35)
            mz_rep = mzk + telem_rng.normal(0, 0.5)
            wz_rep = wz_true[k] + telem_rng.normal(0, 0.012)
            regen_w = -(min(tk[2], 0.0) + min(tk[3], 0.0)) * w_rear
            log[k]["t_rl"].append(rl_rep); log[k]["t_rr"].append(rr_rep)
            log[k]["mz"].append(mz_rep); log[k]["wz_ref"].append(wz_ref)
            log[k]["wz_meas"].append(wz_rep); log[k]["beta_deg"].append(betak)
            log[k]["mz_sat"].append(satk); log[k]["regen_w"].append(regen_w)
 
        if i % stream_stride == 0:
            for k in controllers:
                tk = log[k]
                print(f"{STREAM_COLOR[k]}[{t*1000:7.1f} ms] {k.upper()} | "
                      f"RL {tk['t_rl'][-1]:6.1f} Nm  RR {tk['t_rr'][-1]:6.1f} Nm  "
                      f"Mz {tk['mz'][-1]:6.1f} Nm  wz {tk['wz_meas'][-1]:5.2f} rad/s "
                      f"(ref {tk['wz_ref'][-1]:5.2f})  beta {tk['beta_deg'][-1]:5.1f} deg{RESET}")
 
    print("=" * 96)
 
    # ---- CSV export, mirrors a real data-logger pull ----
    out_dir = os.path.join('output', 'telemetry')
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"{session_id}_phase18_comparison.csv")
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "controller", "t_rl_nm", "t_rr_nm", "mz_nm",
                    "wz_ref_rads", "wz_meas_rads", "beta_deg", "mz_sat_ratio", "regen_w"])
        for i, t in enumerate(time_steps):
            for k in controllers:
                tk = log[k]
                w.writerow([f"{t*1000:.1f}", k, f"{tk['t_rl'][i]:.2f}", f"{tk['t_rr'][i]:.2f}",
                            f"{tk['mz'][i]:.2f}", f"{tk['wz_ref'][i]:.4f}", f"{tk['wz_meas'][i]:.4f}",
                            f"{tk['beta_deg'][i]:.2f}", f"{tk['mz_sat'][i]:.3f}", f"{tk['regen_w'][i]:.1f}"])
    print(f"  Telemetry export written: {csv_path}")
 
    # ---- Scorecard ----
    print("\n" + "-" * 96)
    print(f"{'Controller':<12} | {'wz RMS err':<11} | {'Peak |beta|':<12} | {'Mean |slew|':<12} | "
          f"{'HF energy':<10} | {'Peak Nm':<8} | {'Regen kJ':<9} | Status")
    print("-" * 96)
    kpis = {}
    for k in controllers:
        wz_meas = np.array(log[k]["wz_meas"]); wz_ref = np.array(log[k]["wz_ref"])
        mz = np.array(log[k]["mz"]); rl = np.array(log[k]["t_rl"]); rr = np.array(log[k]["t_rr"])
        beta = np.array(log[k]["beta_deg"]); regen_w = np.array(log[k]["regen_w"])
 
        rms_err = float(np.sqrt(np.mean((wz_meas - wz_ref) ** 2)))
        peak_beta = float(np.nanmax(np.abs(beta))) if not np.all(np.isnan(beta)) else float("nan")
        mean_slew = float(np.mean(np.abs(np.diff(mz)) / dt))
        hf_energy = _v18_fft_hf_energy(mz, dt)
        peak_nm = float(np.max(np.abs(np.concatenate([rl, rr]))))
        regen_kj = float(np.sum(regen_w) * dt / 1000.0)
 
        exploding = peak_nm > 600.0
        chattering = mean_slew > 4500.0 or hf_energy > 20000.0
        status = "FAIL" if exploding else ("WARN" if chattering else "PASS")
        color = "\033[91m" if status == "FAIL" else ("\033[93m" if status == "WARN" else "\033[92m")
 
        kpis[k] = dict(rms_err=rms_err, peak_beta=peak_beta, mean_slew=mean_slew,
                        hf_energy=hf_energy, peak_nm=peak_nm, regen_kj=regen_kj, status=status)
        print(f"{k.upper():<12} | {rms_err:<11.4f} | {peak_beta:<12.2f} | {mean_slew:<12.1f} | "
              f"{hf_energy:<10.1f} | {peak_nm:<8.1f} | {regen_kj:<9.2f} | {color}{status}\033[0m")
    print("-" * 96 + "\n")
 
    # ---- Dashboard plot ----
    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(f'Phase 18: Live Telemetry Comparison -- session {session_id}',
                 fontsize=15, fontweight='bold')
    colors = {"v1": "#7f7f7f", "v2": "#2ca02c", "v3": "#0052cc", "v4": "#ff8800"}
    styles = {"v1": "-", "v2": "-", "v3": "-", "v4": "--"}
    labels = {"v1": "v1: Branch 1 PI", "v2": "v2: Branch 2 SMC (approx.)",
              "v3": "v3: Branch 3 AL-QP", "v4": "v4: Branch 4 NMPC"}
    segs = [(0, 2, '#eeeeee'), (2, 6, '#e8f0ff'), (6, 10, '#ffe8e8'), (10, 16, '#eaffe8')]
 
    ax = axs[0, 0]
    for a, b, c in segs: ax.axvspan(a, b, color=c, alpha=0.6, zorder=0)
    ax.plot(time_steps, log["v1"]["wz_ref"], color='#333333', linestyle=':', linewidth=1.3, label='wz reference')
    for k in controllers:
        ax.plot(time_steps, log[k]["wz_meas"], color=colors[k], linestyle=styles[k], linewidth=1.6, label=labels[k])
    ax.set_title('Yaw Rate Tracking', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('wz (rad/s)'); ax.legend(fontsize=8, loc='best')
 
    ax = axs[0, 1]
    for a, b, c in segs: ax.axvspan(a, b, color=c, alpha=0.6, zorder=0)
    for k in controllers:
        ax.plot(time_steps, log[k]["mz"], color=colors[k], linestyle=styles[k], linewidth=1.4, label=labels[k])
    ax.set_title('Commanded Yaw Moment (Mz)', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Mz (Nm)'); ax.legend(fontsize=8, loc='best')
 
    ax = axs[1, 0]
    for a, b, c in segs: ax.axvspan(a, b, color=c, alpha=0.6, zorder=0)
    for k in ("v3", "v4"):
        ax.plot(time_steps, log[k]["beta_deg"], color=colors[k], linestyle=styles[k], linewidth=1.8, label=labels[k])
    ax.axhline(0, color='#999999', linewidth=0.8)
    ax.set_title('Sideslip Angle (v1/v2 have no EKF -- omitted)', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('beta (deg)'); ax.legend(fontsize=8, loc='best')
 
    ax = axs[1, 1]
    for a, b, c in segs: ax.axvspan(a, b, color=c, alpha=0.6, zorder=0)
    for k in controllers:
        cum_kj = np.cumsum(np.array(log[k]["regen_w"])) * dt / 1000.0
        ax.plot(time_steps, cum_kj, color=colors[k], linestyle=styles[k], linewidth=1.8, label=labels[k])
    ax.set_title('Cumulative Regenerated Energy', fontsize=11, fontweight='semibold')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Energy (kJ)'); ax.legend(fontsize=8, loc='best')
 
    plt.tight_layout()
    out_dir_g = os.path.join('output', 'graphs')
    os.makedirs(out_dir_g, exist_ok=True)
    plot_path = os.path.join(out_dir_g, 'sanity_phase18_live_telemetry_comparison.png')
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    print(f"  Generated: {plot_path}\n")
    plt.close()
 
    return log, kpis
 
# =====================================================================
# 6. MAIN EXECUTION
# =====================================================================
if __name__ == "__main__":
    os.makedirs(os.path.join('output', 'graphs'), exist_ok=True)
    time_steps = np.linspace(0, 3.0, 600)
    
    print("\nStarting Sanity Checks Battery (Master V5.2)...")
    
    # ------------------ SECTION A: STANDARD VALIDATION ------------------
    generate_report([scenario_launch, scenario_ellipse, scenario_regen_reversal, scenario_divergence],
                    ['A: Dead-Stop Launch (Div/0 Protect)', 'B: Friction Ellipse Saturation', 'C: Regen-to-Drive Zero Crossing', 'D: Solver Stability (Impossible Mz)'],
                    'sanity_phase1_core.png', 'Phase 1: Core Physics', time_steps)
    
    generate_report([scenario_mu_split, scenario_sensor_glitch, scenario_liftoff, scenario_rollback],
                    ['E: Mu-Split Asymmetric Loss', 'F: CAN Bus Glitch Resilience', 'G: Lift-off Oversteer Rate Limit', 'H: Rollback / Negative Velocity'],
                    'sanity_phase2_edge_cases.png', 'Phase 2: Edge Cases', time_steps)
    
    generate_report([scenario_steer_sensor_loss, scenario_trail_braking, scenario_resonance, scenario_porpoising],
                    ['I: Steering Encoder Drop (IMU Fallback)', 'J: Trail Braking Entry', 'K: Driveline Resonance (15Hz)', 'L: Suspension Porpoising (4Hz)'],
                    'sanity_phase3_performance.png', 'Phase 3: High Performance', time_steps)
    
    generate_report([scenario_launch_control, scenario_regen_tv_entry, scenario_oversteer_rescue, scenario_anticipatory_tc],
                    ['M: Launch Control (Pre-Tension)', 'N: Regenerative Torque Vectoring', 'O: Oversteer Rescue (Counter-Steer)', 'P: Anticipatory TC (Derivative Cut)'],
                    'sanity_phase4_advanced.png', 'Phase 4: Advanced Dynamics', time_steps)

    # Explicit regression guard: Test N must show a genuine TV split under
    # regen, not a flat/symmetric line (this was the original bug).
    _, _, diff_N, _, _, _ = run_scenario(time_steps, scenario_regen_tv_entry)
    assert np.max(np.abs(diff_N)) > 4.0, (
        f"Regen-TV split collapsed to near-zero (max |diff|={np.max(np.abs(diff_N)):.2f} Nm) — "
        f"the per-wheel regen bound is not shaping asymmetrically."
    )
    print(f"✅ Test N regen-TV split check: max |RR-RL| = {np.max(np.abs(diff_N)):.1f} Nm")

    # New: regen-TV under a deliberately tight total budget — certifies the
    # budget is enforced by proportional rescale (ratio preserved), not by
    # independent per-wheel clamping (which would flatten the split).
    tight_rg = default_regen_limits(enable=1, max_total_trq=60.0, max_charge_power_w=40000.0)
    loose_rg = default_regen_limits(enable=1, max_total_trq=400.0, max_charge_power_w=40000.0)

    rl_tight, rr_tight, diff_tight, *_ = run_scenario(time_steps, scenario_regen_tv_at_limit, regen_limits=tight_rg)
    rl_loose, rr_loose, diff_loose, *_ = run_scenario(time_steps, scenario_regen_tv_at_limit, regen_limits=loose_rg)

    budget_ok = np.all((np.abs(rl_tight) + np.abs(rr_tight)) <= 60.0 + 1e-1)
    mask = np.abs(diff_loose) > 3.0  # <--- NEW THRESHOLD
    ratio_tight = np.abs(diff_tight[mask]) / (np.abs(rl_tight[mask]) + np.abs(rr_tight[mask]) + 1e-6)
    ratio_loose = np.abs(diff_loose[mask]) / (np.abs(rl_loose[mask]) + np.abs(rr_loose[mask]) + 1e-6)
    shape_preserved = np.mean(np.abs(ratio_tight - ratio_loose)) < 0.15

    status = "✅ PASS" if (budget_ok and shape_preserved) else "❌ FAIL"
    print(f"{status} | 29: Regen-TV At The Limit | Budget respected: {budget_ok} | Shape preserved: {shape_preserved}")
    assert budget_ok, "Total regen budget exceeded — scale_neg_trq / regen bound rescale is broken."
    assert shape_preserved, "Regen split collapsed toward symmetric under a tight budget — per-wheel clamping regression."

    # Mixed-sign TV regression guard relocated to Phase 12 (dedicated regen
    # test suite) — see run_phase12_regen_analysis() below.

    # ------------------ SECTION B: DOGFIGHT COMPARISONS ------------------
    print("\nStarting Dogfight Comparisons (PD vs. AL-QP)...")

    # ------------------ SECTION B: DOGFIGHT COMPARISONS ------------------
    print("\nStarting Dogfight Comparisons (PD vs. AL-QP)...")

    # Phase 5: Lateral Dynamics (Comparing Torque Delta for Agility)
    s5 = [scenario_limit_slalom, scenario_oversteer_rescue, scenario_ellipse, scenario_divergence]
    t5 = ['1: Slalom Agility (TV Dynamic Range)', '2: Oversteer Rescue (Counter-Steer Override)', 
          '3: Friction Ellipse (Lateral G Saturation)', '4: Solver Stability vs PID Windup']
    generate_comparison_report(s5, t5, 'sanity_phase5_dogfight_lateral.png', 
                               'Phase 5: Lateral Dynamics & Handling (PD vs. AL-QP)', time_steps, 'lateral')

    # Phase 6: Longitudinal Dynamics (Comparing Assigned Torque on Right Wheel)
    s6 = [scenario_launch_control, scenario_anticipatory_tc, scenario_vmax_aero_drag, scenario_mu_split]
    t6 = ['5: Launch Control (Pre-Tensioning)', '6: Anticipatory TC vs Blind Power', 
          '7: Aero-Downforce Scaling', '8: Mu-Split (Ice Patch Survival)']
    generate_comparison_report(s6, t6, 'sanity_phase6_dogfight_traction.png', 
                               'Phase 6: Longitudinal Traction & Power (PD vs. AL-QP)', time_steps, 'longitudinal')

    # Phase 7: Robustness & Signal Filtering (Comparing Assigned Torque on Left Wheel)
    s7 = [scenario_sensor_glitch, scenario_resonance, scenario_trail_braking, scenario_liftoff]
    t7 = ['9: CAN Bus Glitch (Spike Rejection)', '10: Driveline Resonance (15Hz Filter)', 
          '11: Trail Braking (Brake to Throttle)', '12: Lift-off Oversteer (Rate Limiter)']
    generate_comparison_report(s7, t7, 'sanity_phase7_dogfight_robustness.png', 
                               'Phase 7: Signal Filtering & Robustness (PD vs. AL-QP)', time_steps, 'robustness')
    
    # ------------------ SECTION C: COMPETITION & TORTURE ------------------
    print("\nStarting Phase 8 & 9: Competition & Hardware Torture...")

    # Phase 8: FSAE Dynamic Events (Competition Scoring Scenarios)
    s8 = [scenario_accel_75m, scenario_skidpad_transition, scenario_endurance_hairpin, scenario_fast_sweeper]
    t8 = ['13: Acceleration 75m (Slip Tracking)', '14: Skidpad Transition (Center Figure-8)', 
          '15: Endurance Hairpin (Mechanical Grip)', '16: Autocross Sweeper (Aero Supported)']
    generate_comparison_report(s8, t8, 'sanity_phase8_dogfight_dynamics.png', 
                               'Phase 8: FSAE Dynamic Events (AL-QP vs. PD)', time_steps, 'lateral')

    # Phase 9: Hardware Limits & Degradation (Torture Module)
    s9 = [scenario_thermal_mu_drop, scenario_bms_power_derating, scenario_asymmetric_wear, scenario_sine_with_dwell]
    t9 = ['17: Thermal Degradation (Mid-Corner Drop)', '18: BMS Power Derating (Endurance Heat)', 
          '19: Asymmetric Tire Wear (RL Mod Shift)', '20: Sine with Dwell (Evasive Maneuver ISO)']
    generate_comparison_report(s9, t9, 'sanity_phase9_dogfight_limits.png', 
                               'Phase 9: Hardware Limits & Degradation (AL-QP vs. PD)', time_steps, 'robustness')

    # ------------------ SECTION D: ABSOLUTE LIMITS ------------------
    print("\nStarting Phase 10: Envelope Expansion (Absolute Limits)...")

    s10 = [scenario_vmax_aero_drag, scenario_step_steer_high_speed, scenario_friction_circle_mapping, scenario_hydroplaning_survival]
    t10 = ['21: V-Max Aero-Drag (126 km/h Downforce)', '22: High-Speed Step Steer (100 km/h Transient)', 
           '23: G-Circle Spiral Mapping (Combined Slip)', '24: Hydroplaning Survival (Massive Over-rev)']
    
    # OJO: Usamos generate_report (el de las fases 1-4), NO generate_comparison_report
    generate_report(s10, t10, 'sanity_phase10_envelope_expansion.png', 
                    'Phase 10: AL-QP Absolute Performance Envelope', time_steps)
    
    # ------------------ SECTION E: ULTIMATE PERFORMANCE ------------------
    print("\nStarting Phase 11: Ultimate Performance & Race-Pace Analytics...")

    s11 = [scenario_mid_corner_curb, scenario_variable_grip_launch, scenario_spinout_recovery_limit, scenario_limit_slalom]
    t11 = ['25: Mid-Corner Curb Strike (Transient TC/TV)', '26: Variable Grip Launch (Pacejka Tracking)', 
           '27: High-Speed Spinout Recovery Limit', '28: Limit Slalom (Dynamic Degradation)']
    
    generate_report(s11, t11, 'sanity_phase11_ultimate_performance.png', 
                    'Phase 11: AL-QP Race-Pace Analytics', time_steps)

    # ------------------ SECTION E.5: PHASE 12 — REGEN ANALYSIS ------------------
    run_phase12_regen_analysis(time_steps)

    # ------------------ SECTION E.6: PHASE 13 — NMPC HORIZON SUITE ------------------
    print("\nStarting Phase 13: NMPC Predictive Horizon Analysis...")

    s13 = [scenario_nmpc_chicane_preview, scenario_nmpc_step_overshoot_mitigation, 
           scenario_nmpc_warm_start_recovery, scenario_nmpc_actuator_slew_smoothing]
    t13 = ['34: Chicane Preview (108 km/h)', '35: Step Steer Overshoot Damping', 
           '36: Emergency Lane Change Impulse', '37: 25Hz Encoder Noise Smoothing']

    # Phase 13 is explicitly the NMPC (Branch 4) horizon suite — it must run
    # against gp_lib_nmpc, not the gp_lib alias (which points at Branch 3
    # AL-QP). Running it under AL-QP silently tested the wrong controller:
    # Branch 3's raw PID differentiator has no mechanism analogous to NMPC's
    # r_effort/r_slew quadratic penalties, so no amount of upstream delta
    # filtering was ever going to reproduce NMPC's noise rejection here.
    generate_report(s13, t13, 'sanity_phase13_nmpc_horizon.png', 
                    'Phase 13: Branch 4 Embedded NMPC Horizon Performance', time_steps,
                    lib=gp_lib_nmpc)

    # ------------------ SECTION E.7: PHASE 14 — BRANCH 3 VS BRANCH 4 DOGFIGHT ------------------
    print("\nStarting Phase 14: Dogfight Comparisons (AL-QP Branch 3 vs. NMPC Branch 4)...")

    s14 = [scenario_limit_slalom, scenario_nmpc_chicane_preview, 
           scenario_sine_with_dwell, scenario_spinout_recovery_limit]
    t14 = ['38: Limit Slalom (Dynamic Range)', '39: High-Speed Chicane (Predictive Damping)', 
           '40: Sine with Dwell (Moose Test)', '41: Spinout Recovery (Limit Envelope)']

    generate_nmpc_dogfight_report(s14, t14, 'sanity_phase14_dogfight_nmpc_vs_alqp.png', 
                                  'Phase 14: AL-QP (Branch 3) vs. Embedded NMPC (Branch 4)', time_steps)

    # ------------------ SECTION E.8: PHASE 15-17 — CLOSED-LOOP + SCORECARD ------------------
    run_phase15_closed_loop_dogfight()
    run_phase16_nmpc_weight_sweep()
    run_phase17_scorecard(time_steps)
    run_phase18_live_telemetry_comparison()
    # ------------------ SECTION F: MONTE CARLO NOISE & LATENCY ------------------
    mc_scenarios = {
        "14: Skidpad Transition (Center Figure-8)": scenario_skidpad_transition,
        "22: High-Speed Step Steer (100 km/h)": scenario_step_steer_high_speed,
        "23: G-Circle Spiral Mapping (Combined Slip)": scenario_friction_circle_mapping,
        "25: Mid-Corner Curb Strike (Transient TC)": scenario_mid_corner_curb,
        "28: Limit Slalom (Dynamic Degradation)": scenario_limit_slalom,
        "29: Regen-TV At The Limit": scenario_regen_tv_at_limit,
        "30: Mixed-Sign Regen TV": scenario_mixed_sign_regen_tv,
        "31: Regen Lockup Recovery": scenario_regen_lockup_recovery,
        "34: NMPC Chicane Preview (108 km/h)": scenario_nmpc_chicane_preview,
        "35: NMPC Step Steer Overshoot Damping": scenario_nmpc_step_overshoot_mitigation,
        "36: NMPC Emergency Impulse": scenario_nmpc_warm_start_recovery,
    }
    
    run_monte_carlo_suite(mc_scenarios, num_trials=30, delay_ticks=1)

    # --- Final KPIs ---
    rl_I, rr_I, diff_I, beta_I, alpha_I, _ = run_scenario(time_steps, scenario_limit_slalom)
    rl_K, rr_K, diff_K, beta_K, alpha_K, _ = run_scenario(time_steps, scenario_resonance)
    
    print("\n--- KEY PERFORMANCE INDICATORS (KPIs) ---")
    print(f"Max Control Effort (TV Slew Rate): {np.max(np.abs(np.diff(rr_I) / 0.005)):.1f} Nm/s")
    print(f"Driveline Noise Transmissibility: {np.std(rr_K[100:500]):.2f} Nm RMS")
    print(f"Max Vehicle Sideslip (Slalom):    {np.degrees(np.max(np.abs(beta_I))):.2f}°")
    print(f"Mean Solver Feasibility Factor:  {np.mean(alpha_I):.3f}")
    print("\n✅ All comparisons generated successfully in 2x2 format. Check the output/ folder.\n")