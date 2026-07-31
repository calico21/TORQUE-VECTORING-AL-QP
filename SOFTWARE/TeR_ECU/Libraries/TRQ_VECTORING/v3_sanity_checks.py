import os
import ctypes
import numpy as np

# Rutas de compilación
LIB_NAME = "v3_core.so"
SRC_FILES = ["src/v3_vehicle_dynamics.c", "src/v3_qp_solver.c"]
INC_DIR = "inc"

def compile_shared_library():
    print("[SIL] Compilando libreria compartida para la Rama 3...")
    cmd = f"gcc -shared -fPIC -O2 -o {LIB_NAME} {' '.join(SRC_FILES)} -I{INC_DIR} -lm"
    ret = os.system(cmd)
    if ret != 0:
        raise RuntimeError("Error de compilacion en GCC para la Rama 3.")
    print("[SIL] Compilacion exitosa ->", LIB_NAME)

class V3WheelLoads(ctypes.Structure):
    _fields_ = [("fz_n", ctypes.c_float * 2)]

class V3VehicleParams(ctypes.Structure):
    _fields_ = [
        ("mass_kg", ctypes.c_float),
        ("h_cdg_m", ctypes.c_float),
        ("track_width_rear_m", ctypes.c_float),
        ("wheelbase_m", ctypes.c_float),
        ("rear_weight_frac", ctypes.c_float),
        ("rear_roll_stiffness_frac", ctypes.c_float),
        ("mu", ctypes.c_float),
        ("wheel_radius_m", ctypes.c_float),
    ]

class V3QPConfig(ctypes.Structure):
    _fields_ = [
        ("wt", ctypes.c_float * 2),
        ("ws", ctypes.c_float * 2),
        ("rho", ctypes.c_float),
        ("recip_w", ctypes.c_float * 2),
        ("valid", ctypes.c_uint8),
    ]

class V3QPState(ctypes.Structure):
    _fields_ = [("lambda", ctypes.c_float)]

class V3QPInput(ctypes.Structure):
    _fields_ = [
        ("x_nom", ctypes.c_float * 2),
        ("x_prev", ctypes.c_float * 2),
        ("lb", ctypes.c_float * 2),
        ("ub", ctypes.c_float * 2),
        ("t_demand", ctypes.c_float),
    ]

class V3QPOutput(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float * 2),
        ("lambda", ctypes.c_float),
        ("residual", ctypes.c_float),
        ("feasible", ctypes.c_uint8),
    ]

def run_tests():
    compile_shared_library()
    lib = ctypes.CDLL(os.path.abspath(LIB_NAME))

    # Configurar prototipos de funciones
    lib.v3_qp_config_init.argtypes = [ctypes.POINTER(V3QPConfig), ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_float]
    lib.v3_qp_config_init.restype = ctypes.c_uint8

    lib.v3_qp_state_reset.argtypes = [ctypes.POINTER(V3QPState)]
    lib.v3_qp_solve.argtypes = [ctypes.POINTER(V3QPConfig), ctypes.POINTER(V3QPState), ctypes.POINTER(V3QPInput), ctypes.POINTER(V3QPOutput)]

    # Inicializar Configuración del QP
    cfg = V3QPConfig()
    wt = (ctypes.c_float * 2)(1.0, 1.0)
    ws = (ctypes.c_float * 2)(5.0, 5.0)
    res = lib.v3_qp_config_init(ctypes.byref(cfg), wt, ws, ctypes.c_float(2.0))
    assert res == 1, "Fallo al inicializar la config del QP"

    state = V3QPState()
    lib.v3_qp_state_reset(ctypes.byref(state))

    # Definir Entrada de Prueba
    inp = V3QPInput()
    inp.x_nom = (ctypes.c_float * 2)(50.0, 120.0)    # Demanda nominal asimétrica (Yaw moment)
    inp.x_prev = (ctypes.c_float * 2)(60.0, 100.0)   # Ciclo anterior
    inp.lb = (ctypes.c_float * 2)(-100.0, -100.0)    # Límites inferiores (regen)
    inp.ub = (ctypes.c_float * 2)(300.0, 300.0)      # Límites superiores (motor)
    inp.t_demand = ctypes.c_float(200.0)             # Suma total requerida (RL + RR = 200 Nm)

    out = V3QPOutput()
    lib.v3_qp_solve(ctypes.byref(cfg), ctypes.byref(state), ctypes.byref(inp), ctypes.byref(out))

    print(f"[TEST] Solucion optima RL: {out.x[0]:.2f} Nm, RR: {out.x[1]:.2f} Nm")
    print(f"[TEST] Suma resultante: {out.x[0] + out.x[1]:.2f} Nm (Demandado: {inp.t_demand} Nm)")
    print(f"[TEST] Residuo de igualdad: {out.residual:.6f}")
    print(f"[TEST] Viable: {out.feasible}")

    assert out.feasible == 1, "El problema debería ser estrictamente viable"
    assert abs((out.x[0] + out.x[1]) - inp.t_demand) < 1e-3, "El solver no cumplió la restricción de igualdad"
    print("\n✅ ¡Todas las pruebas unitarias de la Rama 3 superadas con éxito!")

if __name__ == "__main__":
    run_tests()