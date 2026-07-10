import ctypes
import numpy as np
import matplotlib.pyplot as plt
import os

# =====================================================================
# 1. ESTRUCTURAS ACTUALIZADAS (Ctypes)
# =====================================================================
class TCState(ctypes.Structure):
    _fields_ = [("pi_integral", ctypes.c_float * 4),
                ("kappa_filt", ctypes.c_float * 4),
                ("mu_surface", ctypes.c_float),
                ("omega_prev", ctypes.c_float * 4)] # NUEVO: Para el TC Anticipativo

class TVState(ctypes.Structure):
    _fields_ = [("wz_int", ctypes.c_float),
                ("delta_prev", ctypes.c_float),
                ("t_qp_prev", ctypes.c_float * 4),
                ("t_out_prev", ctypes.c_float * 4),
                ("tc", TCState)]

try:
    gp_lib = ctypes.CDLL('./gp_core.so')
except OSError:
    print("Error: No se encuentra gp_core.so. Compila primero con gcc -shared...")
    exit(1)

# Firma actualizada (Añadido el float del freno antes del dt)
gp_lib.gp_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float * 4), 
    ctypes.c_float, ctypes.c_float, # <--- brake_norm, dt
    ctypes.POINTER(TVState), ctypes.POINTER(ctypes.c_float * 4)
]

def run_scenario(time_array, input_generator):
    state = TVState()
    ctypes.memset(ctypes.byref(state), 0, ctypes.sizeof(TVState))
    state.tc.mu_surface = 1.5 
    
    t_rl_log, t_rr_log, tv_diff_log = [], [], []
    
    for t in time_array:
        # Ahora el generador debe devolver 9 valores (añadido brake_norm)
        fx, delta, vx, vy, wz, ay, ax, omega, brake = input_generator(t)
        
        omega_c = (ctypes.c_float * 4)(*omega)
        t_out_c = (ctypes.c_float * 4)()
        
        gp_lib.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, 
                          omega_c, brake, 0.005, ctypes.byref(state), t_out_c)
        
        t_rl_log.append(t_out_c[2])
        t_rr_log.append(t_out_c[3])
        tv_diff_log.append(t_out_c[3] - t_out_c[2])
        
    return np.array(t_rl_log), np.array(t_rr_log), np.array(tv_diff_log)


# =====================================================================
# 2. ESCENARIOS LEGACY (Fases 1 a 3) - [Actualizados con brake=0.0]
# =====================================================================
def scenario_launch(t):
    vx = max(t * 5.0, 0.0) 
    return 2000.0, 0.0, vx, 0.0, 0.0, 0.0, 8.0, [0, 0, (vx*1.05)/0.2032, (vx*1.05)/0.2032], 0.0

def scenario_ellipse(t):
    vx, ay = 20.0, t * 6.0 
    return 3000.0, 0.2, vx, 0.0, 0.5, ay, 0.0, [0, 0, vx/0.2032, vx/0.2032], 0.0

def scenario_curb_shock(t):
    vx = 15.0
    omega_rr = (vx*1.5)/0.2032 if 1.0 < t < 1.5 else (vx*1.05)/0.2032
    return 1500.0, 0.0, vx, 0.0, 0.0, 0.0, 5.0, [0, 0, (vx*1.05)/0.2032, omega_rr], 0.0

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

def scenario_slalom(t):
    vx, freq = 25.0, 0.5 * np.pi 
    return 2000.0, 0.4 * np.sin(freq * t), vx, 0.0, 0.2 * np.sin(freq * t - 0.2), 8.0 * np.sin(freq * t - 0.1), 0.0, [0, 0, vx/0.2032, vx/0.2032], 0.0

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

def scenario_aero_downforce(t):
    """ N: Aceleración prolongada para ver cómo el t_ub crece con v^2 por la aerodinámica """
    vx = t * 10.0 # De 0 a 30 m/s (108 km/h)
    omega = [0, 0, vx/0.2032, vx/0.2032]
    # Forzamos una demanda absurdamente alta para estar siempre saturando el t_ub
    return 5000.0, 0.0, vx, 0.0, 0.0, 0.0, 0.0, omega, 0.0

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


# =====================================================================
# 4. MOTOR DE RENDERIZADO UNIFICADO
# =====================================================================
plt.style.use('default')
plt.rcParams.update({
    'figure.facecolor': 'white', 'axes.facecolor': 'white', 'axes.grid': True,
    'grid.linestyle': '--', 'grid.alpha': 0.7, 'text.color': 'black',
    'axes.labelcolor': 'black', 'xtick.color': 'black', 'ytick.color': 'black'
})

def generate_report(scenarios, titles, filename, super_title, time_steps):
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(super_title, fontsize=16, fontweight='bold')
    
    for ax, (scenario, title) in zip(axs.flat, zip(scenarios, titles)):
        rl, rr, diff = run_scenario(time_steps, scenario)
        
        ax.plot(time_steps, rl, color='#0052cc', linewidth=2.5, label='RL Torque (Nm)')
        ax.plot(time_steps, rr, color='#e60000', linewidth=2.5, linestyle='--', label='RR Torque (Nm)')
        
        if np.max(np.abs(diff)) > 1.0:
            ax.plot(time_steps, diff, color='#2ca02c', linewidth=1.5, alpha=0.8, label='Delta (RR-RL)')
            
        ax.set_title(title, fontsize=11, fontweight='semibold')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.legend(loc='best')
    
    plt.tight_layout()
    output_path = os.path.join('output', filename)
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ Generado: {output_path}")
    plt.close()

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

def run_comparison(time_array, input_generator):
    """Ejecuta ambos sistemas en paralelo y extrae las métricas comparativas"""
    state_new = TVState()
    ctypes.memset(ctypes.byref(state_new), 0, ctypes.sizeof(TVState))
    state_new.tc.mu_surface = 1.5 
    legacy_tv = LegacyTV()
    
    log = {'new_rl': [], 'new_rr': [], 'new_diff': [], 
           'old_rl': [], 'old_rr': [], 'old_diff': []}
    
    for t in time_array:
        fx, delta, vx, vy, wz, ay, ax, omega, brake = input_generator(t)
        
        # Sistema Nuevo
        omega_c = (ctypes.c_float * 4)(*omega)
        t_out_c = (ctypes.c_float * 4)()
        gp_lib.gp_tv_step(fx, delta, vx, vy, wz, ay, ax, omega_c, brake, 0.005, ctypes.byref(state_new), t_out_c)
        
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
    output_path = os.path.join('output', filename)
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"✅ Generated: {output_path}")
    plt.close()


# =====================================================================
# 6. MAIN EXECUTION
# =====================================================================
if __name__ == "__main__":
    os.makedirs('output', exist_ok=True)
    time_steps = np.linspace(0, 3.0, 600)
    
    print("\nStarting Sanity Checks Battery (Master V5.2)...")
    
    # ------------------ SECTION A: STANDARD VALIDATION ------------------
    generate_report([scenario_launch, scenario_ellipse, scenario_curb_shock, scenario_divergence],
                    ['A: Dead-Stop Launch (Div/0 Protect)', 'B: Friction Ellipse Saturation', 'C: Traction Control (Curb Shock)', 'D: Solver Stability (Impossible Mz)'],
                    'sanity_phase1_core.png', 'Phase 1: Core Physics', time_steps)
    
    generate_report([scenario_mu_split, scenario_sensor_glitch, scenario_liftoff, scenario_rollback],
                    ['E: Mu-Split Asymmetric Loss', 'F: CAN Bus Glitch Resilience', 'G: Lift-off Oversteer Rate Limit', 'H: Rollback / Negative Velocity'],
                    'sanity_phase2_edge_cases.png', 'Phase 2: Edge Cases', time_steps)
    
    generate_report([scenario_slalom, scenario_trail_braking, scenario_resonance, scenario_porpoising],
                    ['I: Slalom TV Dynamic Tracking', 'J: Trail Braking Entry', 'K: Driveline Resonance (15Hz)', 'L: Suspension Porpoising (4Hz)'],
                    'sanity_phase3_performance.png', 'Phase 3: High Performance', time_steps)
    
    generate_report([scenario_launch_control, scenario_aero_downforce, scenario_oversteer_rescue, scenario_anticipatory_tc],
                    ['M: Launch Control (Pre-Tension)', 'N: Aero-Aware Downforce (Speed Sweep)', 'O: Oversteer Rescue (Counter-Steer)', 'P: Anticipatory TC (Derivative Cut)'],
                    'sanity_phase4_advanced.png', 'Phase 4: Advanced Dynamics', time_steps)

    # ------------------ SECTION B: DOGFIGHT COMPARISONS ------------------
    print("\nStarting Dogfight Comparisons (PD vs. AL-QP)...")

    # Phase 5: Lateral Dynamics (Comparing Torque Delta for Agility)
    s5 = [scenario_slalom, scenario_oversteer_rescue, scenario_ellipse, scenario_divergence]
    t5 = ['1: Slalom Agility (TV Dynamic Range)', '2: Oversteer Rescue (Counter-Steer Override)', 
          '3: Friction Ellipse (Lateral G Saturation)', '4: Solver Stability vs PID Windup']
    generate_comparison_report(s5, t5, 'sanity_phase5_dogfight_lateral.png', 
                               'Phase 5: Lateral Dynamics & Handling (PD vs. AL-QP)', time_steps, 'lateral')

    # Phase 6: Longitudinal Dynamics (Comparing Assigned Torque on Right Wheel)
    s6 = [scenario_launch_control, scenario_anticipatory_tc, scenario_aero_downforce, scenario_mu_split]
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

    # --- Final KPIs ---
    _, rr_I, _ = run_scenario(time_steps, scenario_slalom)
    _, rr_K, _ = run_scenario(time_steps, scenario_resonance)
    
    print("\n--- KEY PERFORMANCE INDICATORS (KPIs) ---")
    print(f"Max Control Effort (TV Slew Rate): {np.max(np.abs(np.diff(rr_I) / 0.005)):.1f} Nm/s")
    print(f"Driveline Noise Transmissibility: {np.std(rr_K[100:500]):.2f} Nm RMS")
    print("\n✅ All comparisons generated successfully in 2x2 format. Check the output/ folder.\n")