import subprocess
import re
import sys
import os
import ctypes
import argparse
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern
from scipy.stats import norm

# =====================================================================
# TARGET SELECTION - tune either the AL-QP/TC gains (legacy) or the
# Branch 4 condensed SQP-RTI NMPC gains. Kept as one file since both
# share the compile/BO scaffolding; only the param set, header macros,
# and cost function differ.
# =====================================================================
PARAM_SPECS = {
    "alqp": {
        # (macro_name, low, high, header_fmt)
        "params": [
            ("GP_W_SMOOTH", 2.0, 6.0, lambda v: f"#define GP_W_SMOOTH   {v:.3f}f   // Actuator rate penalty weight\n"),
            ("GP_W_REG",    0.4, 1.5, lambda v: f"#define GP_W_REG    {v:.3f}f\n"),
            ("GP_TC_KP",    20.0, 60.0, lambda v: f"#define GP_TC_KP      {v:.3f}f   // Traction control proportional gain\n"),
        ],
        "use_nmpc_flag": False,
    },
    "nmpc": {
        # Bounds rationale:
        #  - Q_YAW: old code normalized by c_sum_sq (O(mz_rate^2)-ish
        #    dynamic scale); new code divides by N=8 flat. Old default
        #    2000 is very likely 3-8x too aggressive under the new
        #    normalization -> search wide, biased low.
        #  - R_EFFORT: direct control weight, same units/meaning as
        #    before, narrower band around old default.
        #  - R_SLEW: rate penalty on the CONDENSED sequence's tridiagonal
        #    term now, not a scalar u_warm anchor only -> also re-search.
        #  - Q_BETA: barrier weight, frozen-gate GN approx is sensitive
        #    to this being too soft (barrier never bites) or too hard
        #    (fights the Hessian conditioning).
        "params": [
            ("GP_NMPC_Q_YAW",    500.0,  10000.0, lambda v: f"#define GP_NMPC_Q_YAW       {v:.1f}f     // Yaw rate error penalty\n"),
            ("GP_NMPC_R_EFFORT", 0.001,  0.20,    lambda v: f"#define GP_NMPC_R_EFFORT       {v:.4f}f    // Control effort penalty\n"),
            ("GP_NMPC_R_SLEW",   0.1,    10.0,    lambda v: f"#define GP_NMPC_R_SLEW        {v:.3f}f     // Rate-of-change penalty\n"),
            ("GP_NMPC_Q_BETA",   200.0,  6000.0,  lambda v: f"#define GP_NMPC_Q_BETA          {v:.1f}f     // Soft barrier penalty weight\n"),
        ],
        "use_nmpc_flag": True,
    },
}

PARAMS_HEADER = "inc/gp_params.h"


def update_header_file(target, values):
    spec = PARAM_SPECS[target]
    names = [p[0] for p in spec["params"]]
    fmts = [p[3] for p in spec["params"]]

    with open(PARAMS_HEADER, "r") as f:
        lines = f.readlines()

    counts = {n: 0 for n in names}
    new_lines = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("#define"):
            token = stripped.split()[1] if len(stripped.split()) > 1 else ""
            if token in names:
                idx = names.index(token)
                line = fmts[idx](values[idx])
                counts[token] += 1
        new_lines.append(line)

    with open(PARAMS_HEADER, "w") as f:
        f.writelines(new_lines)

    missing = [n for n, c in counts.items() if c == 0]
    if missing:
        raise RuntimeError(f"Macro(s) not found in {PARAMS_HEADER}: {missing} - check gp_params.h hasn't drifted.")
    print(f"[DEBUG] Updated {dict(counts)}")


def compile_c_core(target, out_path="gp_core_tune.so"):
    spec = PARAM_SPECS[target]
    cmd = [
        "gcc", "-shared", "-fPIC", "-O3", "-lm",
        "src/gp_math.c",
        "src/gp_vehicle_model.c",
        "src/gp_solver.c",
        "src/gp_traction_control.c",
        "src/gp_torque_vectoring.c",
        "src/gp_ekf.c",
        "src/gp_nmpc.c",          # was missing entirely - gp_torque_vectoring.c
                                    # calls gp_nmpc_step()/gp_nmpc_init(), so the
                                    # old script was silently link-erroring or
                                    # (worse, if a stale gp_core.so existed)
                                    # evaluating whatever binary was already there.
    ]
    if spec["use_nmpc_flag"]:
        cmd.append("-DGP_TV_USE_NMPC=1")   # was never set - every prior "NMPC"
                                              # BO run actually compiled and
                                              # scored the AL-QP branch.
    cmd += ["-I.", "-Isrc", "-Iinc", "-o", out_path]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Compilation Error:", result.stderr)
    return result.returncode == 0


# =====================================================================
# Self-contained ctypes layer (does NOT import master_sanity_checks -
# that module eagerly CDLL-loads gp_core_alqp.so/gp_core_nmpc.so at
# import time with their own layout asserts; keeping the tuner
# independent means it can't be broken by, or accidentally clobber,
# those reference builds). Struct layout must track gp_torque_vectoring.h
# / gp_nmpc.h / gp_traction_control.h / gp_ekf.h exactly.
# =====================================================================
_EVAL_SCRIPT = r"""
import sys, ctypes
import numpy as np

class TCState(ctypes.Structure):
    _fields_ = [
        ("pi_integral", ctypes.c_float * 4), ("kappa_filt", ctypes.c_float * 4),
        ("mu_surface", ctypes.c_float * 2), ("omega_last_raw", ctypes.c_float * 4),
        ("omega_prev_ema", ctypes.c_float * 4), ("rls_P", ctypes.c_float * 4),
        ("rls_theta", ctypes.c_float * 4), ("theta_prev", ctypes.c_float * 4),
        ("kappa_prev", ctypes.c_float * 4), ("fx_prev", ctypes.c_float * 4),
        ("kappa_opt", ctypes.c_float * 4), ("omega_dot_kick_filt", ctypes.c_float * 4),
    ]

class EkfState(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float * 2), ("P", (ctypes.c_float * 2) * 2),
        ("Q", ctypes.c_float * 2), ("delta_ref", ctypes.c_float),
        ("R_gps_vy", ctypes.c_float), ("R_pseudo_vy", ctypes.c_float),
        ("R_mu", ctypes.c_float), ("beta_est", ctypes.c_float),
        ("vy_std", ctypes.c_float), ("wz_corrected", ctypes.c_float),
    ]

class NMPCState(ctypes.Structure):
    _fields_ = [
        ("x_pred", (ctypes.c_float * 2) * 9),   # N=8 -> N+1=9
        ("A_d", (ctypes.c_float * 2) * 2),
        ("B_d", (ctypes.c_float * 1) * 2),
        ("u_seq", ctypes.c_float * 8),
        ("u_warm", ctypes.c_float),
        ("q_yaw", ctypes.c_float), ("r_effort", ctypes.c_float), ("r_slew", ctypes.c_float),
    ]

class GPRegenLimits(ctypes.Structure):
    _fields_ = [
        ("enable", ctypes.c_uint8),
        ("max_total_trq", ctypes.c_float), ("max_charge_power_w", ctypes.c_float),
    ]

class TVState(ctypes.Structure):
    _fields_ = [
        ("wz_int", ctypes.c_float), ("delta_prev", ctypes.c_float),
        ("t_qp_prev", ctypes.c_float * 4), ("t_out_prev", ctypes.c_float * 4),
        ("tc", TCState), ("ekf", EkfState), ("nmpc", NMPCState),
        ("vy_est", ctypes.c_float), ("alpha_qp", ctypes.c_float),
        ("lam_prev", ctypes.c_float), ("mz_sat_ratio", ctypes.c_float),
        ("vy_gps_last", ctypes.c_float), ("vy_gps_age_ms", ctypes.c_float),
        ("ax_filt", ctypes.c_float), ("ay_filt", ctypes.c_float),
        ("t_ub_rl_filt", ctypes.c_float), ("t_ub_rr_filt", ctypes.c_float),
        ("t_lb_rl_filt", ctypes.c_float), ("t_lb_rr_filt", ctypes.c_float),
        ("delta_notch_x1", ctypes.c_float), ("delta_notch_x2", ctypes.c_float),
        ("delta_notch_y1", ctypes.c_float), ("delta_notch_y2", ctypes.c_float),
    ]

try:
    lib = ctypes.CDLL('./gp_core_tune.so')
except OSError as e:
    print("ERR:", e); sys.exit(1)

lib.gp_tv_state_sizeof.restype = ctypes.c_size_t
if ctypes.sizeof(TVState) != lib.gp_tv_state_sizeof():
    print(f"ERR: layout mismatch py={ctypes.sizeof(TVState)} c={lib.gp_tv_state_sizeof()}")
    sys.exit(1)

lib.gp_tv_init.argtypes = [ctypes.POINTER(TVState)]
lib.gp_tv_step.argtypes = [
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float * 4),
    ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8,
    ctypes.POINTER(GPRegenLimits), ctypes.c_float,
    ctypes.POINTER(TVState), ctypes.POINTER(ctypes.c_float * 4),
]

R_WHEEL = 0.2032
TRACK_R = 1.180
LF, LR = 0.8525, 0.6975

def make_state():
    s = TVState()
    lib.gp_tv_init(ctypes.byref(s))
    s.tc.mu_surface[0] = 1.5
    s.tc.mu_surface[1] = 1.5
    return s

def regen(enable=1, total=400.0, power=40000.0):
    r = GPRegenLimits(); r.enable = enable; r.max_total_trq = total; r.max_charge_power_w = power
    return r

class Plant:
    # 2-state linear bicycle plant, closed-loop reference. Mirrors
    # ClosedLoopBicyclePlant in master_sanity_checks.py exactly, kept local
    # to avoid a hard import dependency in the tuning subprocess.
    def __init__(self, mass=300.0, iz=150.0, lf=LF, lr=LR, cf=35000.0, cr=32000.0):
        self.mass, self.iz, self.lf, self.lr, self.cf, self.cr = mass, iz, lf, lr, cf, cr
        self.vy, self.wz = 0.0, 0.0
    def step(self, vx, delta, mz_ext, dt):
        vxs = max(abs(vx), 1.0)
        vy_dot = (-(self.cf+self.cr)/(self.mass*vxs))*self.vy \
                 + (((self.lr*self.cr - self.lf*self.cf)/(self.mass*vxs)) - vxs)*self.wz \
                 + (self.cf/self.mass)*delta
        wz_dot = ((self.lr*self.cr - self.lf*self.cf)/(self.iz*vxs))*self.vy \
                 - ((self.lf**2*self.cf + self.lr**2*self.cr)/(self.iz*vxs))*self.wz \
                 + (self.lf*self.cf/self.iz)*delta + mz_ext/self.iz
        self.vy += vy_dot*dt; self.wz += wz_dot*dt
        return self.vy, self.wz

def run_closed_loop(vx, delta_base, freq=0.0, noise_amp=0.0, t_total=2.0, dt=0.005):
    state = make_state()
    plant = Plant()
    rg = regen()
    n = int(t_total/dt)
    wz_log, ref_log, mz_log = [], [], []
    vy_t, wz_t = 0.0, 0.0
    for k in range(n):
        t = k*dt
        if freq > 0:
            delta = delta_base*np.sin(2*np.pi*freq*t) if t > 0.05 else 0.0
        else:
            delta = delta_base if t > 0.05 else 0.0
        if noise_amp > 0 and t > 0.05:
            delta += noise_amp*np.sin(2*np.pi*18.0*t)
        wz_ref = (vx*delta)/(LF+LR)
        omega_c = (ctypes.c_float*4)(0.0, 0.0, vx/R_WHEEL, vx/R_WHEEL)
        out = (ctypes.c_float*4)()
        ay_meas = wz_t*vx
        lib.gp_tv_step(800.0, delta, vx, vy_t, wz_t, ay_meas, 0.0, omega_c, 0.0,
                       60.0, 60.0, 0.0, 0, ctypes.byref(rg), dt, ctypes.byref(state), out)
        mz_cmd = (out[3]-out[2])*TRACK_R/(2.0*R_WHEEL)
        vy_t, wz_t = plant.step(vx, delta, mz_cmd, dt)
        wz_log.append(wz_t); ref_log.append(wz_ref); mz_log.append(mz_cmd)
    return np.array(wz_log), np.array(ref_log), np.array(mz_log)

def step_metrics(wz, ref):
    yf = ref[-1]
    if abs(yf) < 1e-6:
        return dict(rise=np.nan, overshoot=np.nan, settle=np.nan)
    sign = np.sign(yf); yn = wz*sign; yfn = abs(yf)
    idx10 = np.argmax(yn >= 0.1*yfn); idx90 = np.argmax(yn >= 0.9*yfn)
    rise = (idx90-idx10)*0.005 if idx90 > idx10 else 2.0
    yss = yn[-1]
    overshoot = max(0.0, (np.max(yn)-yss)/yss*100.0) if yss > 1e-6 else 0.0
    band = 0.02*yss
    outside = np.where(np.abs(yn-yss) > band)[0]
    settle = outside[-1]*0.005 if len(outside) else 0.0
    return dict(rise=rise, overshoot=overshoot, settle=settle)

# --- 1. Precision step (matches Phase 17's headline metric) ---
wz1, ref1, mz1 = run_closed_loop(20.0, 0.015)
if not np.all(np.isfinite(wz1)) or np.max(np.abs(wz1)) > 50.0:
    print("999999.0"); sys.exit(0)
m1 = step_metrics(wz1, ref1)

# --- 2. 18Hz steering jitter rejection (what R_effort/R_slew actually buy) ---
_, _, mz2 = run_closed_loop(20.0, 0.015, noise_amp=0.008)
slew_noise = np.mean(np.abs(np.diff(mz2)/0.005))

# --- 3. Chicane-rate slalom (1.5Hz) - the scenario this upgrade targets.
#    Score BOTH tracking (settle) and how cleanly Mz is planned (slew),
#    since a horizon that "sees" the chicane should need less abrupt
#    correction than a control-hold approximation. ---
wz3, ref3, mz3 = run_closed_loop(25.0, 0.020, freq=1.8, t_total=2.0)
if not np.all(np.isfinite(wz3)) or np.max(np.abs(wz3)) > 50.0:
    print("999999.0"); sys.exit(0)
tracking_err3 = np.mean(np.abs(wz3 - ref3))
slew3 = np.mean(np.abs(np.diff(mz3)/0.005))

# Composite cost (lower = better). Weights chosen so no single term can
# be driven to zero at the expense of the others going unstable:
#  - settle/rise/overshoot: standard step-response quality (Phase 17 style)
#  - slew_noise: penalizes gains that amplify sensor jitter into chatter
#  - tracking_err3 + slew3: the chicane-preview metric this whole upgrade
#    exists to improve - weighted comparably to the step metrics so BO
#    can't "win" by overfitting the single-step case.
cost = (m1['settle']*35.0 + m1['overshoot']*0.8 + m1['rise']*25.0
        + slew_noise*0.02 + tracking_err3*400.0 + slew3*0.02)

if not np.isfinite(cost):
    cost = 999999.0
print(f"{cost:.3f}")
"""


def evaluate_in_subprocess():
    result = subprocess.run([sys.executable, "-c", _EVAL_SCRIPT], capture_output=True, text=True)
    if result.returncode != 0 or "ERR" in result.stdout:
        print("--- SUBPROCESS ERROR TRACE ---")
        print(result.stderr); print(result.stdout)
        print("------------------------------")
        return float("inf")
    try:
        return float(result.stdout.strip())
    except ValueError:
        return float("inf")


def expected_improvement(X, X_sample, Y_sample, gpr, xi=0.01):
    mu, sigma = gpr.predict(X, return_std=True)
    sigma = np.maximum(sigma, 1e-9)
    improvement = np.min(Y_sample) - mu - xi
    z = improvement / sigma
    return improvement * norm.cdf(z) + sigma * norm.pdf(z)


def run_bayesian_optimization(target, total_iterations=60, init_random=10):
    spec = PARAM_SPECS[target]
    names = [p[0] for p in spec["params"]]
    bounds = np.array([[p[1], p[2]] for p in spec["params"]])
    dim = len(names)

    print(f"\nTuning target: {target.upper()} | params: {names}\n")

    X_sample, Y_sample = [], []
    best_cost, best_params = float("inf"), None

    for i in range(init_random):
        x = np.array([np.random.uniform(lo, hi) for lo, hi in bounds])
        update_header_file(target, x)
        if not compile_c_core(target):
            continue
        cost = evaluate_in_subprocess()
        print(f"Init {i+1:2d} | " + " | ".join(f"{n}={v:.3f}" for n, v in zip(names, x)) + f" | Cost: {cost:9.3f}")
        X_sample.append(x); Y_sample.append(cost)
        if cost < best_cost:
            best_cost, best_params = cost, x

    X_sample = np.array(X_sample); Y_sample = np.array(Y_sample)
    kernel = Matern(nu=2.5)
    gpr = GaussianProcessRegressor(kernel=kernel, alpha=1e-6, normalize_y=True,
                                    n_restarts_optimizer=5, random_state=42)

    for i in range(init_random, total_iterations):
        gpr.fit(X_sample, Y_sample)
        cand = np.random.uniform(bounds[:, 0], bounds[:, 1], size=(3000, dim))
        ei = expected_improvement(cand, X_sample, Y_sample, gpr)
        x = cand[np.argmax(ei)]

        update_header_file(target, x)
        if not compile_c_core(target):
            continue
        cost = evaluate_in_subprocess()
        print(f"BO   {i+1:2d} | " + " | ".join(f"{n}={v:.3f}" for n, v in zip(names, x)) + f" | Cost: {cost:9.3f} (EI)")

        X_sample = np.vstack((X_sample, x))
        Y_sample = np.append(Y_sample, cost)
        if cost < best_cost:
            best_cost, best_params = cost, x

    print("\n" + "=" * 60)
    print(f"OPTIMAL {target.upper()} CONFIGURATION:")
    for n, v in zip(names, best_params):
        print(f"   {n} = {v:.3f}")
    print(f"   Min Cost = {best_cost:.3f}")
    print("=" * 60 + "\n")

    update_header_file(target, best_params)
    compile_c_core(target)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", choices=["nmpc", "alqp"], default="nmpc")
    ap.add_argument("--iterations", type=int, default=60)
    ap.add_argument("--init-random", type=int, default=10)
    args = ap.parse_args()
    run_bayesian_optimization(args.target, args.iterations, args.init_random)