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
        ("pi_integral", ctypes.c_float * 4),
        ("kappa_filt", ctypes.c_float * 4),
        ("mu_surface", ctypes.c_float * 2),
        ("omega_last_raw", ctypes.c_float * 4),
        ("omega_prev_ema", ctypes.c_float * 4),
        ("rls_P", ctypes.c_float * 4),
        ("rls_theta", ctypes.c_float * 4),
        ("theta_prev", ctypes.c_float * 4),  # <-- Crítico para la secante
        ("kappa_prev", ctypes.c_float * 4),
        ("fx_prev", ctypes.c_float * 4),
        ("kappa_opt", ctypes.c_float * 4),
    ]

class TVState(ctypes.Structure):
    _fields_ = [
        ("wz_int", ctypes.c_float),
        ("delta_prev", ctypes.c_float),
        ("t_qp_prev", ctypes.c_float * 4),
        ("t_out_prev", ctypes.c_float * 4),
        ("tc", TCState),
        ("vy_est", ctypes.c_float),
        ("alpha_qp", ctypes.c_float),  # <-- Crítico para el AL-QP Iterativo
        ("lam_prev", ctypes.c_float),  # <-- Crítico para el AL-QP Iterativo
    ]
try:
    gp_lib = ctypes.CDLL('./gp_core.so')
except OSError:
    print("Error: No se encuentra gp_core.so. Compila primero con gcc -shared...")
    exit(1)

# Firma actualizada para gp_tv_step
gp_lib.gp_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float * 4), 
    ctypes.c_float, ctypes.c_float, 
    ctypes.POINTER(TVState), ctypes.POINTER(ctypes.c_float * 4)
]

# NUEVO: Firma para gp_tv_init
gp_lib.gp_tv_init.argtypes = [ctypes.POINTER(TVState)]

def run_scenario(time_array, input_generator):
    state = TVState()
    # ELIMINAR: ctypes.memset(ctypes.byref(state), 0, ctypes.sizeof(TVState))
    gp_lib.gp_tv_init(ctypes.byref(state)) # <--- AÑADIDO: Llama a C directamente
    state.tc.mu_surface[0] = 1.5
    state.tc.mu_surface[1] = 1.5
    
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

def evaluate_test_kpis(time_steps, t_rl, t_rr, t_diff, test_name):
    """ Evalúa la señal matemática y decide si el coche sobrevive o rompe. """
    # 1. Detección de Chattering (Ruido de alta frecuencia letal para el inversor)
    slew_rate_rl = np.diff(t_rl) / (time_steps[1] - time_steps[0])
    noise_rms = np.std(slew_rate_rl)
    
    # 2. Detección de Inestabilidad (Oscilación divergente o picos absurdos)
    max_torque = np.max(np.abs(t_rl))
    
    # 3. Criterios de Validación (Límites duros)
    is_chattering = noise_rms > 5000.0  # Límite de slew rate "saludable"
    is_exploding = max_torque > 600.0   # Más de 600 Nm por rueda es un error matemático
    
    if is_exploding:
        status = "❌ FAIL (Divergencia)"
        color = "\033[91m" # Rojo
    elif is_chattering:
        status = "⚠️ WARN (Chattering)"
        color = "\033[93m" # Amarillo
    else:
        status = "✅ PASS"
        color = "\033[92m" # Verde
        
    reset_color = "\033[0m"
    print(f"{color}{status:<18} | {test_name:<42} | Ruido RMS: {noise_rms:7.1f} | Par Max: {max_torque:5.1f} Nm{reset_color}")

def generate_report(scenarios, titles, filename, super_title, time_steps):
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(super_title, fontsize=16, fontweight='bold')
    
    for ax, (scenario, title) in zip(axs.flat, zip(scenarios, titles)):
        rl, rr, diff = run_scenario(time_steps, scenario)
        # NUEVA LÍNEA: Imprimir diagnóstico numérico por terminal
        evaluate_test_kpis(time_steps, rl, rr, diff, title)
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
    state_new = TVState()
    # ELIMINAR: ctypes.memset(ctypes.byref(state_new), 0, ctypes.sizeof(TVState))
    gp_lib.gp_tv_init(ctypes.byref(state_new)) # <--- AÑADIDO: Llama a C directamente
    state_new.tc.mu_surface[0] = 1.5
    state_new.tc.mu_surface[1] = 1.5
    # ...
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

def scenario_trail_braking_apex(t):
    """27: Trail Braking. Transición de saturación longitudinal a lateral."""
    # Frena fuerte, luego va dando gas
    fx = 0.0 if t < 1.5 else min((t - 1.5) * 2500.0, 2000.0)
    brake = max(1.0 - t, 0.0) # Suelta el freno progresivamente de t=0 a t=1
    
    # Empieza a girar el volante justo mientras suelta el freno
    delta = min(t * 0.8, 1.2) if t < 2.0 else 1.2 
    
    vx = max(25.0 - (t * 10.0), 12.0) if t < 1.5 else min(12.0 + (t-1.5)*5.0, 20.0)
    vy = 0.0
    wz = delta * 1.1
    ay = vx * wz
    ax = -12.0 if brake > 0 else (fx / 250.0)
    
    w_rear = vx / 0.23
    omega = [0.0, 0.0, w_rear, w_rear]
    return fx, delta, vx, vy, wz, ay, ax, omega, brake

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

    s11 = [scenario_mid_corner_curb, scenario_variable_grip_launch, scenario_trail_braking_apex, scenario_limit_slalom]
    t11 = ['25: Mid-Corner Curb Strike (Transient TC/TV)', '26: Variable Grip Launch (Pacejka Tracking)', 
           '27: Aggressive Trail Braking (Combined Saturation)', '28: Limit Slalom (Dynamic Degration)']
    
    generate_report(s11, t11, 'sanity_phase11_ultimate_performance.png', 
                    'Phase 11: AL-QP Race-Pace Analytics', time_steps)

    # --- Final KPIs ---
    _, rr_I, _ = run_scenario(time_steps, scenario_slalom)
    _, rr_K, _ = run_scenario(time_steps, scenario_resonance)
    
    print("\n--- KEY PERFORMANCE INDICATORS (KPIs) ---")
    print(f"Max Control Effort (TV Slew Rate): {np.max(np.abs(np.diff(rr_I) / 0.005)):.1f} Nm/s")
    print(f"Driveline Noise Transmissibility: {np.std(rr_K[100:500]):.2f} Nm RMS")
    print("\n✅ All comparisons generated successfully in 2x2 format. Check the output/ folder.\n")