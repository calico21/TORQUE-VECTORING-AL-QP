import subprocess
import re
import numpy as np
import os
import sys
import glob
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern

def update_header_files(smooth_val, reg_val, kp_val, rate_limit_val):
    """ Programmatically updates macros in canonical header files """
    # Update canonical params file (gp_params.h)
    with open("inc/gp_params.h", "r") as f:
        content = f.read()
    
    content = re.sub(r'(#define GP_W_SMOOTH\s+)[0-9\.]+(f?)', rf'\g<1>{smooth_val:.3f}f', content)
    content = re.sub(r'(#define GP_W_REG\s+)[0-9\.]+(f?)', rf'\g<1>{reg_val:.3f}f', content)
    content = re.sub(r'(#define GP_TC_KP\s+)[0-9\.]+(f?)', rf'\g<1>{kp_val:.3f}f', content)
    
    with open("inc/gp_params.h", "w") as f:
        f.write(content)

    # Update TV rate limiter (gp_torque_vectoring.h)
    with open("inc/gp_torque_vectoring.h", "r") as f:
        tv_content = f.read()
        
    tv_content = re.sub(r'(#define GP_TV_RATE_LIMIT\s+)[0-9\.]+(f?)', rf'\g<1>{rate_limit_val:.1f}f', tv_content)
    
    with open("inc/gp_torque_vectoring.h", "w") as f:
        f.write(tv_content)

def compile_c_core():
    """ Re-compiles gp_core.so including all source files in src/ """
    c_files = [
        f for f in glob.glob("src/*.c")
        if not any(ex in f for ex in ["gp_interface.c", "tv_mds.c", "pid.c"])
    ]
    cmd = ["gcc", "-shared", "-fPIC", "-O3", "-lm"] + c_files + ["-I.", "-Isrc", "-Iinc", "-o", "gp_core.so"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"\n[Compilation Error]:\n{result.stderr.strip()}\n")
        return False
    return True

def evaluate_in_subprocess():
    """ Evaluates chatter & slew cost across transient step scenarios in an isolated process """
    eval_script = """
import numpy as np
from master_sanity_checks import (
    run_scenario,
    scenario_regen_reversal, scenario_sensor_glitch, 
    scenario_liftoff, scenario_launch_control, scenario_spinout_recovery_limit
)

time_steps = np.linspace(0, 3.0, 600)
dt = time_steps[1] - time_steps[0]

transient_scenarios = [
    scenario_regen_reversal,
    scenario_sensor_glitch,
    scenario_liftoff,
    scenario_launch_control,
    scenario_spinout_recovery_limit
]

total_cost = 0.0
for sc in transient_scenarios:
    rl, rr, diff, beta, alpha_qp, mz_sat = run_scenario(time_steps, sc)
    slew_rate = np.diff(rl) / dt
    rms_noise = np.std(slew_rate)

    detrended = rl - np.linspace(rl[0], rl[-1], len(rl))
    window = np.hanning(len(detrended))
    fft_vals = np.abs(np.fft.rfft(detrended * window))
    freqs = np.fft.rfftfreq(len(detrended), d=dt)
    hf_energy = np.sum(fft_vals[freqs > 20.0])

    total_cost += hf_energy + (rms_noise * 0.5)

print(f"{total_cost:.2f}")
"""
    result = subprocess.run([sys.executable, "-c", eval_script], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"\n[Subprocess Error Output]:\n{result.stderr.strip()}\n")
        return 1e8
    try:
        val = float(result.stdout.strip())
        return val if np.isfinite(val) else 1e8
    except ValueError:
        return 1e8

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

def run_bayesian_optimization(total_iterations=40, init_random=10):
    """ Optimizes GP_W_SMOOTH, GP_W_REG, GP_TC_KP, and GP_TV_RATE_LIMIT """
    print(f"\nStarting Bayesian Optimization Suite ({total_iterations} total trials)...\n")
    
    # Parameter bounds: [GP_W_SMOOTH, GP_W_REG, GP_TC_KP, GP_TV_RATE_LIMIT]
    bounds = np.array([
        [8.0, 20.0],     # GP_W_SMOOTH (Shifted higher)
        [0.05, 0.50],    # GP_W_REG (Shifted lower)
        [15.0, 35.0],    # GP_TC_KP
        [2500.0, 4000.0] # GP_TV_RATE_LIMIT (Centered around 3200)
    ])
    
    X_sample = []
    Y_sample = []
    
    best_cost = float('inf')
    best_params = None
    
    # 1. Initial Random Sampling Phase
    for i in range(init_random):
        smooth = np.random.uniform(bounds[0,0], bounds[0,1])
        reg = np.random.uniform(bounds[1,0], bounds[1,1])
        kp = np.random.uniform(bounds[2,0], bounds[2,1])
        rate_lim = np.random.uniform(bounds[3,0], bounds[3,1])
        
        update_header_files(smooth, reg, kp, rate_lim)
        if not compile_c_core():
            continue
            
        cost = evaluate_in_subprocess()
        print(f"Init Trial {i+1:2d} | Smooth: {smooth:5.2f} | Reg: {reg:4.2f} | KP: {kp:4.1f} | RateLim: {rate_lim:6.1f} | Cost: {cost:8.2f}")
        
        X_sample.append([smooth, reg, kp, rate_lim])
        Y_sample.append(cost)
        
        if cost < best_cost:
            best_cost = cost
            best_params = (smooth, reg, kp, rate_lim)

    X_sample = np.array(X_sample)
    Y_sample = np.array(Y_sample)
    
    kernel = Matern(nu=2.5)
    gpr = GaussianProcessRegressor(kernel=kernel, alpha=1e-6, normalize_y=True, n_restarts_optimizer=5, random_state=42)
    
    # 2. Bayesian Optimization Loop
    for i in range(init_random, total_iterations):
        gpr.fit(X_sample, Y_sample)
        
        candidate_count = 2000
        candidates = np.zeros((candidate_count, 4))
        candidates[:, 0] = np.random.uniform(bounds[0,0], bounds[0,1], candidate_count)
        candidates[:, 1] = np.random.uniform(bounds[1,0], bounds[1,1], candidate_count)
        candidates[:, 2] = np.random.uniform(bounds[2,0], bounds[2,1], candidate_count)
        candidates[:, 3] = np.random.uniform(bounds[3,0], bounds[3,1], candidate_count)
        
        ei = expected_improvement(candidates, X_sample, Y_sample, gpr)
        next_x = candidates[np.argmax(ei)]
        
        smooth, reg, kp, rate_lim = next_x[0], next_x[1], next_x[2], next_x[3]
        
        update_header_files(smooth, reg, kp, rate_lim)
        if not compile_c_core():
            continue
            
        cost = evaluate_in_subprocess()
        print(f"BO Trial   {i+1:2d} | Smooth: {smooth:5.2f} | Reg: {reg:4.2f} | KP: {kp:4.1f} | RateLim: {rate_lim:6.1f} | Cost: {cost:8.2f}")
        
        X_sample = np.vstack((X_sample, next_x))
        Y_sample = np.append(Y_sample, cost)
        
        if cost < best_cost:
            best_cost = cost
            best_params = (smooth, reg, kp, rate_lim)

    print("\n==============================================")
    print(f"🏆 OPTIMAL CONFIGURATION FOUND:")
    print(f"   GP_W_SMOOTH      = {best_params[0]:.3f}f")
    print(f"   GP_W_REG         = {best_params[1]:.3f}f")
    print(f"   GP_TC_KP         = {best_params[2]:.3f}f")
    print(f"   GP_TV_RATE_LIMIT = {best_params[3]:.1f}f")
    print(f"   Min Cost         = {best_cost:.2f}")
    print("==============================================\n")
    
    update_header_files(best_params[0], best_params[1], best_params[2], best_params[3])
    compile_c_core()

if __name__ == "__main__":
    run_bayesian_optimization(total_iterations=40, init_random=10)