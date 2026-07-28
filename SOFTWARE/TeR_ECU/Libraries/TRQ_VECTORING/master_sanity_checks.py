import ctypes
import numpy as np
import matplotlib.pyplot as plt
import os

# =====================================================================
# 1. ESTRUCTURAS ACTUALIZADAS (Ctypes)
# =====================================================================
import ctypes

class TCState(ctypes.Structure):
    _fields_ = [
        ("pi_integral",     ctypes.c_float * 4),
        ("kappa_filt",      ctypes.c_float * 4),
        ("mu_surface",      ctypes.c_float * 2),
        ("omega_last_raw",  ctypes.c_float * 4),
        ("omega_prev_ema",  ctypes.c_float * 4),
        ("rls_P",           ctypes.c_float * 4),
        ("rls_theta",       ctypes.c_float * 4),
        ("theta_prev",      ctypes.c_float * 4),
        ("kappa_prev",      ctypes.c_float * 4),
        ("fx_prev",         ctypes.c_float * 4),
        ("kappa_opt",       ctypes.c_float * 4),
    ]

class GPEKFState(ctypes.Structure):
    _fields_ = [
        ("x",            ctypes.c_float * 4),
        ("P",            (ctypes.c_float * 4) * 4),
        ("Q",            ctypes.c_float * 4),
        ("R_gps_vy",     ctypes.c_float),
        ("R_pseudo_vy",  ctypes.c_float),
        ("R_mu",         ctypes.c_float),
        ("beta_est",     ctypes.c_float),
        ("vy_std",       ctypes.c_float),
        ("wz_corrected", ctypes.c_float),
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

class TVState(ctypes.Structure):
    _fields_ = [
        ("wz_int",         ctypes.c_float),
        ("delta_prev",     ctypes.c_float),
        ("t_qp_prev",      ctypes.c_float * 4),
        ("t_out_prev",     ctypes.c_float * 4),
        ("tc",             TCState),
        ("ekf",            EkfState),
        ("vy_est",         ctypes.c_float),
        ("alpha_qp",       ctypes.c_float),
        ("lam_prev",       ctypes.c_float),
        ("mz_sat_ratio",   ctypes.c_float),
        ("vy_gps_last",    ctypes.c_float),
        ("vy_gps_age_ms",  ctypes.c_float),
        ("ax_filt",        ctypes.c_float),   # new
        ("ay_filt",        ctypes.c_float),   # new
        ("t_ub_rl_filt",   ctypes.c_float),   # new
        ("t_ub_rr_filt",   ctypes.c_float),   # new
        ("t_lb_rl_filt",   ctypes.c_float),   # NEW: filtered regen (negative) bound RL
        ("t_lb_rr_filt",   ctypes.c_float),   # NEW: filtered regen (negative) bound RR
    ]

# Structural Safety Assertions
assert ctypes.sizeof(TCState) == 42 * 4, f"TCState size mismatch"

try:
    gp_lib = ctypes.CDLL('./gp_core.so')
except OSError:
    print("Error: No se encuentra gp_core.so. Compila primero con gcc -shared...")
    exit(1)

# Runtime size probe to guarantee layout synchronization
gp_lib.gp_tv_state_sizeof.restype = ctypes.c_size_t
assert ctypes.sizeof(TVState) == gp_lib.gp_tv_state_sizeof(), \
    f"TVState layout drift: Python size ({ctypes.sizeof(TVState)}) != C size ({gp_lib.gp_tv_state_sizeof()})"

# Firma actualizada para gp_tv_step (incluye vy_gps y gps_valid)
gp_lib.gp_tv_step.argtypes = [
    ctypes.c_float,                     # fx_driver
    ctypes.c_float,                     # delta
    ctypes.c_float,                     # vx
    ctypes.c_float,                     # vy
    ctypes.c_float,                     # wz
    ctypes.c_float,                     # ay
    ctypes.c_float,                     # ax
    ctypes.POINTER(ctypes.c_float * 4), # omega
    ctypes.c_float,                     # brake_norm
    ctypes.c_float,                     # temp_inv_rl
    ctypes.c_float,                     # temp_inv_rr
    ctypes.c_float,                     # vy_gps
    ctypes.c_uint8,                     # gps_valid
    ctypes.POINTER(GPRegenLimits),      # regen
    ctypes.c_float,                     # dt
    ctypes.POINTER(TVState),            # state
    ctypes.POINTER(ctypes.c_float * 4)  # t_out_c
]

# NUEVO: Firma para gp_tv_init
gp_lib.gp_tv_init.argtypes = [ctypes.POINTER(TVState)]

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

def run_scenario(time_array, input_generator, non_idealities=None, regen_limits=None):
    state = TVState()
    gp_lib.gp_tv_init(ctypes.byref(state))
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
        
        gp_lib.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, 
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
        "Slalom", "G-Circle", "Regen", "Glitch", "Launch", "Spinout"
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

def generate_report(scenarios, titles, filename, super_title, time_steps):
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(super_title, fontsize=16, fontweight='bold')
    
    for ax, (scenario, title) in zip(axs.flat, zip(scenarios, titles)):
        # Unpack all 6 returned telemetry arrays
        rl, rr, diff, beta, alpha_qp, mz_sat = run_scenario(time_steps, scenario)
        
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
                    "Step Steer",
                    "Hydroplaning",
                    "Curb Strike",
                    "Trail Braking",
                    "Slalom",
                    "G-Circle",
                    "Skidpad",
                    "Regen",
                ]
            )
            hf_limit = 25000.0 if is_transient else 3500.0
            
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
    assert np.max(np.abs(diff_N)) > 5.0, (
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
    mask = np.abs(diff_loose) > 5.0
    ratio_tight = np.abs(diff_tight[mask]) / (np.abs(rl_tight[mask]) + np.abs(rr_tight[mask]) + 1e-6)
    ratio_loose = np.abs(diff_loose[mask]) / (np.abs(rl_loose[mask]) + np.abs(rr_loose[mask]) + 1e-6)
    shape_preserved = np.mean(np.abs(ratio_tight - ratio_loose)) < 0.15

    status = "✅ PASS" if (budget_ok and shape_preserved) else "❌ FAIL"
    print(f"{status} | 29: Regen-TV At The Limit | Budget respected: {budget_ok} | Shape preserved: {shape_preserved}")
    assert budget_ok, "Total regen budget exceeded — scale_neg_trq / regen bound rescale is broken."
    assert shape_preserved, "Regen split collapsed toward symmetric under a tight budget — per-wheel clamping regression."

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

    # ------------------ SECTION F: MONTE CARLO NOISE & LATENCY ------------------
    mc_scenarios = {
        "14: Skidpad Transition (Center Figure-8)": scenario_skidpad_transition,
        "22: High-Speed Step Steer (100 km/h)": scenario_step_steer_high_speed,
        "23: G-Circle Spiral Mapping (Combined Slip)": scenario_friction_circle_mapping,
        "25: Mid-Corner Curb Strike (Transient TC)": scenario_mid_corner_curb,
        "28: Limit Slalom (Dynamic Degradation)": scenario_limit_slalom,
        "29: Regen-TV At The Limit": scenario_regen_tv_at_limit,
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