import ctypes
import os
import pytest
from hypothesis import given, strategies as st, settings

# Load the compiled shared C library
LIB_PATH = os.path.abspath("./gp_core.so")
gp = ctypes.CDLL(LIB_PATH)

F4 = ctypes.c_float * 4

gp.gp_qp_solve_rwd.argtypes = [
    F4, F4, ctypes.c_float, F4, F4, 
    ctypes.c_float, ctypes.POINTER(ctypes.c_float), F4, ctypes.POINTER(ctypes.c_float)
]

gp.gp_qp_solve_rwd_closedform.argtypes = [
    F4, F4, ctypes.c_float, F4, F4, F4, ctypes.POINTER(ctypes.c_float)
]

TOL_ABS_NM = 3.0   # Active-set softness width threshold (GP_SAT_SOFTNESS = 3.0 Nm)
TOL_REL    = 0.02  # 2% relative tolerance

@settings(max_examples=5000, deadline=None)
@given(
    fx=st.floats(min_value=-3000.0, max_value=3000.0),
    t_prev_rl_raw=st.floats(min_value=-300.0, max_value=300.0),
    t_prev_rr_raw=st.floats(min_value=-300.0, max_value=300.0),
    ub_rl=st.floats(min_value=0.0, max_value=400.0),
    ub_rr=st.floats(min_value=0.0, max_value=400.0),
)
def test_closedform_matches_iterative_reference(fx, t_prev_rl_raw, t_prev_rr_raw, ub_rl, ub_rr):
    """
    Property-based test: Certifies that the fast smooth active-set closed-form 
    solver matches the 16-iteration AL-QP reference solver across physical states.
    """
    t_ub = F4(0.0, 0.0, ub_rl, ub_rr)
    t_lb = F4(0.0, 0.0, 0.0, 0.0)

    # Physical State Invariant: t_prev is bounded by [t_lb, t_ub] (+/- softness margin)
    t_prev_rl = max(0.0, min(t_prev_rl_raw, ub_rl + 3.0))
    t_prev_rr = max(0.0, min(t_prev_rr_raw, ub_rr + 3.0))

    t_prev = F4(0.0, 0.0, t_prev_rl, t_prev_rr)
    warm = t_prev

    # 1. Run Reference Iterative AL-QP Solver (16 Iterations)
    ref_out = F4()
    alpha = 1.0 / (0.405 + 5.787 + 5.0 * (2.0 / (0.2032**2)))
    lam = ctypes.c_float(0.0)
    ref_res = ctypes.c_float(0.0)
    
    gp.gp_qp_solve_rwd(
        warm, t_prev, ctypes.c_float(fx), t_lb, t_ub,
        ctypes.c_float(alpha), ctypes.byref(lam), ref_out, ctypes.byref(ref_res)
    )

    # 2. Run Fast Smooth Active-Set Closed-Form Solver
    cf_out = F4()
    cf_res = ctypes.c_float(0.0)
    
    gp.gp_qp_solve_rwd_closedform(
        warm, t_prev, ctypes.c_float(fx), t_lb, t_ub, cf_out, ctypes.byref(cf_res)
    )

    # Calculate step jump delta between initial warmstart and target demand
    target_total_trq = fx * 0.2032
    step_delta = abs((t_prev_rl + t_prev_rr) - target_total_trq)

    # 3. Assert precision across rear wheels (RL = index 2, RR = index 3)
    for i in (2, 3):
        err = abs(cf_out[i] - ref_out[i])
        # Account for 16-iteration AL-QP gradient convergence lag during massive open-loop step jumps
        max_allowed = max(TOL_ABS_NM, TOL_REL * abs(ref_out[i]), 0.05 * step_delta)
        assert err <= max_allowed, (
            f"Wheel {i} Mismatch: Reference={ref_out[i]:.2f} Nm, "
            f"ClosedForm={cf_out[i]:.2f} Nm, Error={err:.2f} Nm (Limit: {max_allowed:.2f} Nm)"
        )