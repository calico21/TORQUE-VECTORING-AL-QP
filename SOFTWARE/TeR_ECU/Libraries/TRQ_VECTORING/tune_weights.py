import subprocess
import re
import numpy as np
import os
import sys
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern

def update_header_file(smooth_val, reg_val, kp_val):
    """ Programmatically updates tuning macros in inc/gp_params.h """
    params_path = "inc/gp_params.h"
    
    c_smooth, c_reg, c_kp = 0, 0, 0
    
    if os.path.exists(params_path):
        with open(params_path, "r") as f:
            lines = f.readlines()
            
        new_lines = []
        for line in lines:
            if "#define" in line:
                if "GP_W_SMOOTH" in line:
                    line = f"#define GP_W_SMOOTH   {smooth_val:.3f}f   // Actuator rate penalty weight\n"
                    c_smooth += 1
                elif "GP_W_REG" in line:
                    line = f"#define GP_W_REG    {reg_val:.3f}f\n"
                    c_reg += 1
                elif "GP_TC_KP" in line:
                    line = f"#define GP_TC_KP      {kp_val:.3f}f   // Traction control proportional gain\n"
                    c_kp += 1
            new_lines.append(line)
            
        with open(params_path, "w") as f:
            f.writelines(new_lines)
            
    print(f"[DEBUG] Updates applied -> Smooth: {c_smooth}, Reg: {c_reg}, KP: {c_kp}")

def compile_c_core():
    """ Re-compiles gp_core.so with the new weights """
    cmd = [
        "gcc", "-shared", "-fPIC", "-O3", "-lm",
        "src/gp_math.c",
        "src/gp_vehicle_model.c",
        "src/gp_solver.c",
        "src/gp_traction_control.c",
        "src/gp_torque_vectoring.c",
        "src/gp_ekf.c",
        "-I.", "-Isrc", "-Iinc", "-I../../TeR/Inc",
        "-o", "gp_core.so"
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Compilation Error:", result.stderr)
    return result.returncode == 0

def evaluate_in_subprocess():
    """ Evaluates performance in a fresh subprocess to bypass ctypes DLL caching """
    eval_script = """
import numpy as np
import ctypes
from master_sanity_checks import TVState, run_scenario, scenario_limit_slalom

try:
    gp_lib = ctypes.CDLL('./gp_core.so')
except OSError as e:
    print("ERR:", e)
    exit(1)

gp_lib.gp_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float * 4), 
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, 
    ctypes.c_uint8, ctypes.c_float, ctypes.POINTER(TVState), ctypes.POINTER(ctypes.c_float * 4)
]
gp_lib.gp_tv_init.argtypes = [ctypes.POINTER(TVState)]

time_steps = np.linspace(0, 3.0, 600)
dt = time_steps[1] - time_steps[0]

scenario_res = run_scenario(time_steps, scenario_limit_slalom)
rl, rr, diff = scenario_res[0], scenario_res[1], scenario_res[2]
slew_rate = np.diff(rl) / dt
rms_noise = np.std(slew_rate)

detrended = rl - np.linspace(rl[0], rl[-1], len(rl))
window = np.hanning(len(detrended))
fft_vals = np.abs(np.fft.rfft(detrended * window))
freqs = np.fft.rfftfreq(len(detrended), d=dt)
hf_energy = np.sum(fft_vals[freqs > 20.0])

cost = hf_energy + (rms_noise * 0.5)
print(f"{cost:.2f}")
"""
    result = subprocess.run([sys.executable, "-c", eval_script], capture_output=True, text=True)
    if result.returncode != 0 or "ERR" in result.stdout:
        print("--- SUBPROCESS ERROR TRACE ---")
        print(result.stderr)
        print(result.stdout)
        print("------------------------------")
        return float('inf')
    try:
        return float(result.stdout.strip())
    except ValueError:
        return float('inf')

def expected_improvement(X, X_sample, Y_sample, gpr, xi=0.01):
    """ Computes Expected Improvement acquisition function """
    mu, sigma = gpr.predict(X, return_std=True)
    sigma = np.maximum(sigma, 1e-9)
    
    current_best = np.min(Y_sample)
    improvement = current_best - mu - xi
    
    from scipy.stats import norm
    z = improvement / sigma
    ei = improvement * norm.cdf(z) + sigma * norm.pdf(z)
    return ei

def run_bayesian_optimization(total_iterations=30, init_random=5):
    """ Runs Bayesian Optimization with a Gaussian Process Surrogate Model """
    print(f"\nStarting Bayesian Optimization Suite ({total_iterations} total trials)...\n")
    
    # Parameter bounds: [GP_W_SMOOTH, GP_W_REG, GP_TC_KP]
    bounds = np.array([[2.0, 6.0], [0.4, 1.5], [20.0, 60.0]])
    
    X_sample = []
    Y_sample = []
    
    best_cost = float('inf')
    best_params = None
    
    # 1. Initial Random Sampling Phase
    for i in range(init_random):
        smooth = np.random.uniform(bounds[0,0], bounds[0,1])
        reg = np.random.uniform(bounds[1,0], bounds[1,1])
        kp = np.random.uniform(bounds[2,0], bounds[2,1])
        
        update_header_file(smooth, reg, kp)
        if not compile_c_core():
            continue
            
        cost = evaluate_in_subprocess()
        print(f"Init Trial {i+1:2d} | Smooth: {smooth:.2f} | Reg: {reg:.2f} | KP: {kp:.1f} | Cost: {cost:8.2f}")
        
        X_sample.append([smooth, reg, kp])
        Y_sample.append(cost)
        
        if cost < best_cost:
            best_cost = cost
            best_params = (smooth, reg, kp)

    X_sample = np.array(X_sample)
    Y_sample = np.array(Y_sample)
    
    # Gaussian Process Surrogate with Matérn Kernel
    kernel = Matern(nu=2.5)
    gpr = GaussianProcessRegressor(kernel=kernel, alpha=1e-6, normalize_y=True, n_restarts_optimizer=5, random_state=42)
    
    # 2. Bayesian Optimization Loop
    for i in range(init_random, total_iterations):
        gpr.fit(X_sample, Y_sample)
        
        # Sample candidate points randomly to maximize acquisition function (Expected Improvement)
        candidate_count = 2000
        candidates = np.zeros((candidate_count, 3))
        candidates[:, 0] = np.random.uniform(bounds[0,0], bounds[0,1], candidate_count)
        candidates[:, 1] = np.random.uniform(bounds[1,0], bounds[1,1], candidate_count)
        candidates[:, 2] = np.random.uniform(bounds[2,0], bounds[2,1], candidate_count)
        
        ei = expected_improvement(candidates, X_sample, Y_sample, gpr)
        next_x = candidates[np.argmax(ei)]
        
        smooth, reg, kp = next_x[0], next_x[1], next_x[2]
        
        update_header_file(smooth, reg, kp)
        if not compile_c_core():
            continue
            
        cost = evaluate_in_subprocess()
        print(f"BO Trial   {i+1:2d} | Smooth: {smooth:.2f} | Reg: {reg:.2f} | KP: {kp:.1f} | Cost: {cost:8.2f} (EI selected)")
        
        X_sample = np.vstack((X_sample, next_x))
        Y_sample = np.append(Y_sample, cost)
        
        if cost < best_cost:
            best_cost = cost
            best_params = (smooth, reg, kp)

    print("\n==============================================")
    print(f"🏆 TRUE OPTIMAL CONFIGURATION FOUND:")
    print(f"   GP_W_SMOOTH = {best_params[0]:.3f}f")
    print(f"   GP_W_REG    = {best_params[1]:.3f}f")
    print(f"   GP_TC_KP    = {best_params[2]:.3f}f")
    print(f"   Min Cost    = {best_cost:.2f}")
    print("==============================================\n")
    
    update_header_file(best_params[0], best_params[1], best_params[2])
    compile_c_core()

if __name__ == "__main__":
    run_bayesian_optimization(total_iterations=500, init_random=20)