"""
Tecnun eRacing | TeR Trackside Telemetry Dashboard v3.0
=========================================================

Sustituye/mejora a hil_telemetry_gui.py. Cambios clave:

  1. Selector de canales (panel izquierdo, agrupado por subsistema + presets)
     -> puedes elegir cualquier subconjunto de ~35 señales sin saturar la UI.
  2. Cobertura completa de sensores: AL-QP/TV, TC/RLS-Pacejka, velocidad y
     par por rueda, inversores, IMU, pedales/volante, AMS/HV, estado AS/DV.
  3. Decodificación real de CAN vía `cantools` + tus .dbc (ter/inverter/
     hvbms/ams) para las señales "normales", y decodificación manual para
     las 3 tramas de debug propietarias del AL-QP (0x100/0x101/0x102),
     replicando *exactamente* el packing de gp_pack_telemetry().
  4. Tema oscuro estilo MoTeC, banda de alertas con umbrales warn/crit por
     canal, y logging opcional a CSV para post-análisis.

BUGS ENCONTRADOS AL IMPLEMENTAR ESTO (léelo, es importante):
--------------------------------------------------------------
  A) gp_pack_telemetry() [gp_interface.c] empaqueta en la trama 0x100:
         bytes[0:2] = vy_est*100
         bytes[2:4] = wz_int*100      <-- NO es qp_residual
         bytes[4:6] = kappa_opt[RL]*10000
         bytes[6:8] = kappa_opt[RR]*10000
     Tanto hil_telemetry_gui.py como trackside_terminal.py decodificaban
     bytes[2:4] como "qp_residual / 1000" (unsigned) y descartaban por
     completo kappa_opt[RR] (bytes[6:8]). Es decir: el residuo del solver
     KKT que se mostraba en pantalla NUNCA fue el residuo real, era el
     entero de wz_int mal escalado. He corregido el decode aquí (ver
     CanWorker._decode) y sustituido el canal fantasma "qp_residual" por
     "wz_int" (que sí se transmite). Si quieres recuperar qp_residual de
     verdad, hace falta añadir un 4º campo/trama en firmware — parche
     mínimo sugerido al final de este fichero como comentario.

  B) TeR_INERTIAL.c, función inertial(): las tres líneas de encode del
     acelerómetro usan la MISMA función de encode para los 3 ejes:
         TeR.accel.a_x = ter_accel_a_x_encode(IMU.a_x);
         TeR.accel.a_y = ter_accel_a_x_encode(IMU.a_y);   // <- debería ser _a_y_encode
         TeR.accel.a_z = ter_accel_a_x_encode(IMU.a_z);   // <- debería ser _a_z_encode
     Si el .dbc define factor/offset distintos por señal (habitual en
     cantools cuando cada eje tiene rango físico distinto), accel_y y
     accel_z que verás en este dashboard estarán mal escalados. Si
     comparten factor/offset es inofensivo, pero mejor confirmarlo contra
     el .dbc antes de fiarte del canal en pista.

  C) TeR_DV_STATEMACHINE.c define localmente:
         #define DEG2RAD (180.0/PI)
     que matemáticamente es RAD2DEG, no DEG2RAD (compara con el
     DEG2RAD correcto en tv_mds.h: (PI/180.0f)). Usado en set_steer_angle()
     justo después de un encode "que ya pasa a grados" según el comentario
     — el nombre está invertido como mínimo, y merece una revisión rápida
     para confirmar que la conversión de ángulo de steering es la
     correcta y no solo "funciona por casualidad" con las unidades del
     encode.

Requisitos:
    pip install PyQt5 pyqtgraph numpy
    pip install python-can cantools        # solo si USE_REAL_CAN = True
"""

from __future__ import annotations

import csv
import math
import sys
import threading
import os
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt, QTimer
from pyqtgraph.dockarea import DockArea, Dock
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygonF, QFont
from PyQt5.QtCore import QPointF

try:
    import can  # python-can
    HAVE_PYTHON_CAN = True
except ImportError:
    HAVE_PYTHON_CAN = False

try:
    import cantools
    HAVE_CANTOOLS = True
except ImportError:
    HAVE_CANTOOLS = False


# =====================================================================
# CONFIGURACIÓN
# =====================================================================
USE_REAL_CAN = False          # True en el garaje/pista con el USB-CAN puesto
CAN_BUSTYPE = "pcan"
CAN_CHANNEL = "PCAN_USBBUS1"
CAN_BITRATE = 500000

DBC_FILES = [                 # ficheros cantools ya usados para generar el firmware
    "dbc/ter.dbc",
    "dbc/inverter.dbc",
    "dbc/hvbms.dbc",
    "dbc/ams.dbc",
]

HISTORY_LEN = 600              # profundidad del ring buffer (30 s @ 20 Hz)
UI_REFRESH_HZ = 20


# =====================================================================
# MODELO DE CANAL
# =====================================================================
@dataclass
class ChannelSpec:
    key: str
    label: str
    group: str
    units: str
    color: str
    y_range: Optional[tuple] = None
    decimals: int = 2
    warn: Optional[Callable[[float], bool]] = None
    crit: Optional[Callable[[float], bool]] = None


GROUP_ORDER = [
    "AL-QP / Torque Vectoring",
    "Traction Control / RLS Pacejka",
    "Velocidad y Par de Rueda",
    "Powertrain / Inversores",
    "Chasis / IMU",
    "Inputs del Piloto",
    "Batería HV (AMS)",
    "Estado AS / DV",
]

CHANNELS: List[ChannelSpec] = [
    # ---- AL-QP / Torque Vectoring --------------------------------------
    ChannelSpec("vy_est", "Deriva Lateral (v_y)", GROUP_ORDER[0], "m/s", "#FF3333",
                y_range=(-3, 3), warn=lambda v: abs(v) > 1.5, crit=lambda v: abs(v) > 2.5),
    ChannelSpec("wz_int", "Integral Error Yaw", GROUP_ORDER[0], "-", "#FFAA00",
                y_range=(-210, 210), warn=lambda v: abs(v) > 150),
    ChannelSpec("delta_trq", "TV Delta Torque (RR-RL)", GROUP_ORDER[0], "Nm", "#00E5FF",
                y_range=(-60, 60)),
    ChannelSpec("yaw_ref", "Yaw Rate Referencia", GROUP_ORDER[0], "deg/s", "#FF00FF",
                y_range=(-150, 150)),

    # ---- Traction Control / RLS Pacejka --------------------------------
    ChannelSpec("kappa_opt_rl", "Target Slip RL", GROUP_ORDER[1], "%", "#FFEA00", y_range=(0, 25)),
    ChannelSpec("kappa_opt_rr", "Target Slip RR", GROUP_ORDER[1], "%", "#FFD700", y_range=(0, 25)),
    ChannelSpec("kappa_filt_rl", "Slip Filtrado RL", GROUP_ORDER[1], "%", "#FFFFFF", y_range=(0, 25)),
    ChannelSpec("kappa_filt_rr", "Slip Filtrado RR", GROUP_ORDER[1], "%", "#CFCFCF", y_range=(0, 25)),
    ChannelSpec("theta_rl", "Pacejka Slope RL (dFx/dk)", GROUP_ORDER[1], "N", "#00E5FF",
                y_range=(-10000, 45000), warn=lambda v: v < 2000),
    ChannelSpec("theta_rr", "Pacejka Slope RR (dFx/dk)", GROUP_ORDER[1], "N", "#FF00FF",
                y_range=(-10000, 45000), warn=lambda v: v < 2000),
    ChannelSpec("mu_rl", "Grip Estimado RL (mu)", GROUP_ORDER[1], "-", "#00FF00",
                y_range=(0, 2), warn=lambda v: v < 0.8),
    ChannelSpec("mu_rr", "Grip Estimado RR (mu)", GROUP_ORDER[1], "-", "#33FF66",
                y_range=(0, 2), warn=lambda v: v < 0.8),

    # ---- Velocidad y Par de Rueda --------------------------------------
    ChannelSpec("rl_rpm", "RPM Rueda RL", GROUP_ORDER[2], "rpm", "#0052CC", y_range=(0, 6000)),
    ChannelSpec("rr_rpm", "RPM Rueda RR", GROUP_ORDER[2], "rpm", "#E60000", y_range=(0, 6000)),
    ChannelSpec("rl_trq", "Par Estimado RL", GROUP_ORDER[2], "Nm", "#0052CC", y_range=(-60, 250)),
    ChannelSpec("rr_trq", "Par Estimado RR", GROUP_ORDER[2], "Nm", "#E60000", y_range=(-60, 250)),
    ChannelSpec("t_out_rl", "Par Comandado RL (AL-QP)", GROUP_ORDER[2], "Nm", "#00E5FF", y_range=(-60, 300)),
    ChannelSpec("t_out_rr", "Par Comandado RR (AL-QP)", GROUP_ORDER[2], "Nm", "#FF00FF", y_range=(-60, 300)),
    ChannelSpec("vehicle_speed", "Velocidad Vehículo", GROUP_ORDER[2], "km/h", "#FFFFFF", y_range=(0, 140)),

    # ---- Powertrain / Inversores ---------------------------------------
    ChannelSpec("left_dem", "Inv. Izq. Dem", GROUP_ORDER[3], "%", "#FFAA00", y_range=(0, 100)),
    ChannelSpec("right_dem", "Inv. Der. Dem", GROUP_ORDER[3], "%", "#FF8800", y_range=(0, 100)),
    ChannelSpec("left_motor_temp", "Temp. Motor Izq.", GROUP_ORDER[3], "C", "#FF3333",
                y_range=(0, 120), warn=lambda v: v > 90, crit=lambda v: v > 110),
    ChannelSpec("right_motor_temp", "Temp. Motor Der.", GROUP_ORDER[3], "C", "#FF6666",
                y_range=(0, 120), warn=lambda v: v > 90, crit=lambda v: v > 110),
    ChannelSpec("left_power_stage_temp", "Temp. Etapa Potencia Izq.", GROUP_ORDER[3], "C", "#FFAA88",
                y_range=(0, 100), warn=lambda v: v > 80, crit=lambda v: v > 95),
    ChannelSpec("right_power_stage_temp", "Temp. Etapa Potencia Der.", GROUP_ORDER[3], "C", "#FFCCAA",
                y_range=(0, 100), warn=lambda v: v > 80, crit=lambda v: v > 95),

    # ---- Chasis / IMU ----------------------------------------------------
    ChannelSpec("accel_x", "Accel Longitudinal", GROUP_ORDER[4], "m/s2", "#00E5FF", y_range=(-20, 20)),
    ChannelSpec("accel_y", "Accel Lateral", GROUP_ORDER[4], "m/s2", "#FF00FF", y_range=(-20, 20)),
    ChannelSpec("accel_z", "Accel Vertical", GROUP_ORDER[4], "m/s2", "#00FF88", y_range=(0, 20)),
    ChannelSpec("yaw_rate", "Yaw Rate (IMU)", GROUP_ORDER[4], "deg/s", "#FF3333", y_range=(-200, 200)),
    ChannelSpec("pitch_rate", "Pitch Rate", GROUP_ORDER[4], "deg/s", "#FFAA00", y_range=(-100, 100)),
    ChannelSpec("roll_rate", "Roll Rate", GROUP_ORDER[4], "deg/s", "#FFEA00", y_range=(-100, 100)),
    ChannelSpec("yaw", "Yaw", GROUP_ORDER[4], "deg", "#00E5FF", y_range=(-180, 180)),
    ChannelSpec("pitch", "Pitch", GROUP_ORDER[4], "deg", "#00FFAA", y_range=(-45, 45)),
    ChannelSpec("roll", "Roll", GROUP_ORDER[4], "deg", "#88FFEE", y_range=(-45, 45)),

    # ---- Inputs del Piloto ------------------------------------------------
    ChannelSpec("apps", "APPS", GROUP_ORDER[5], "%", "#00FF00", y_range=(0, 100)),
    ChannelSpec("bpps", "BPPS", GROUP_ORDER[5], "%", "#FF3333", y_range=(0, 100)),
    ChannelSpec("steer_angle", "Ángulo Volante", GROUP_ORDER[5], "deg", "#FFFFFF", y_range=(-90, 90)),

    # ---- Batería HV (AMS) --------------------------------------------------
    ChannelSpec("cell_max_volt", "Celda Max Volt", GROUP_ORDER[6], "mV", "#FF6666",
                y_range=(3000, 4300), warn=lambda v: v > 4150, crit=lambda v: v > 4200),
    ChannelSpec("cell_min_volt", "Celda Min Volt", GROUP_ORDER[6], "mV", "#66AAFF",
                y_range=(3000, 4300), warn=lambda v: v < 3200, crit=lambda v: v < 3000),
    ChannelSpec("cell_max_temp", "Celda Max Temp", GROUP_ORDER[6], "C", "#FF8800",
                y_range=(0, 70), warn=lambda v: v > 55, crit=lambda v: v > 65),
    ChannelSpec("hv_current", "Corriente HV", GROUP_ORDER[6], "A", "#00E5FF",
                y_range=(-100, 300), crit=lambda v: abs(v) > 150),

    # ---- Estado AS / DV ------------------------------------------------------
    ChannelSpec("as_status", "AS Status", GROUP_ORDER[7], "enum", "#FFEA00", y_range=(0, 6), decimals=0),
    ChannelSpec("r2d", "Ready2Drive", GROUP_ORDER[7], "bool", "#00FF00", y_range=(-0.2, 1.2), decimals=0),
    ChannelSpec("sl_relay", "SL Relay Cerrado", GROUP_ORDER[7], "bool", "#00FFAA", y_range=(-0.2, 1.2), decimals=0),
]

PRESETS: Dict[str, set] = {
    "AL-QP Core": {"vy_est", "wz_int", "delta_trq", "yaw_ref"},
    "Tracción y Grip": {"kappa_opt_rl", "kappa_opt_rr", "kappa_filt_rl", "kappa_filt_rr",
                         "theta_rl", "theta_rr", "mu_rl", "mu_rr"},
    "Salud Powertrain": {"left_dem", "right_dem", "left_motor_temp", "right_motor_temp",
                          "left_power_stage_temp", "right_power_stage_temp", "rl_trq", "rr_trq"},
    "Dinámica Chasis": {"accel_x", "accel_y", "accel_z", "yaw_rate", "pitch_rate",
                         "roll_rate", "yaw", "pitch", "roll"},
    "Piloto y Ruedas": {"apps", "bpps", "steer_angle", "rl_rpm", "rr_rpm", "vehicle_speed"},
    "Batería HV": {"cell_max_volt", "cell_min_volt", "cell_max_temp", "hv_current"},
    "Todo": {c.key for c in CHANNELS},
    "Vaciar": set(),
}

# NOTE: verifica estos pares (mensaje, señal) contra tu .dbc real generado
# con cantools — los nombres están inferidos de los campos de struct en
# TeR_CAN.h / TeR_STATEMACHINE.c y NO están garantizados 1:1.
SIGNAL_MAP: Dict[tuple, str] = {
    ("TER_WHEEL_INFO", "rl_rpm"): "rl_rpm",
    ("TER_WHEEL_INFO", "rr_rpm"): "rr_rpm",
    ("TER_WHEEL_INFO", "rl_trq"): "rl_trq",
    ("TER_WHEEL_INFO", "rr_trq"): "rr_trq",
    ("TER_WHEEL_INFO", "speed"): "vehicle_speed",
    ("TER_INVERTER_INFO", "left_dem"): "left_dem",
    ("TER_INVERTER_INFO", "right_dem"): "right_dem",
    ("TER_INVERTER_INFO", "left_motor_temp"): "left_motor_temp",
    ("TER_INVERTER_INFO", "right_motor_temp"): "right_motor_temp",
    ("TER_INVERTER_INFO", "left_power_stage_temp"): "left_power_stage_temp",
    ("TER_INVERTER_INFO", "right_power_stage_temp"): "right_power_stage_temp",
    ("TER_TV_DEBUG", "delta_trq"): "delta_trq",
    ("TER_TV_DEBUG", "yaw_ref"): "yaw_ref",
    ("TER_ANG_RATE", "yaw_rate_z"): "yaw_rate",
    ("TER_ANG_RATE", "pitch_rate_y"): "pitch_rate",
    ("TER_ANG_RATE", "roll_rate_x"): "roll_rate",
    ("TER_ACCEL", "a_x"): "accel_x",
    ("TER_ACCEL", "a_y"): "accel_y",
    ("TER_ACCEL", "a_z"): "accel_z",
    ("TER_YPR", "yaw"): "yaw",
    ("TER_YPR", "pitch"): "pitch",
    ("TER_YPR", "roll"): "roll",
    ("TER_APPS", "apps_av"): "apps",
    ("TER_BPPS", "bpps"): "bpps",
    ("TER_STEER", "angle"): "steer_angle",
    ("AMS_CELL_VOLTAGE_STATUS", "cell_max_volt"): "cell_max_volt",
    ("AMS_CELL_VOLTAGE_STATUS", "cell_min_volt"): "cell_min_volt",
    ("AMS_CELL_TEMPERATURES_STATUS", "cell_max_temp"): "cell_max_temp",
    ("AMS_HV_MEASUREMENTS_STATUS", "current_a"): "hv_current",
    ("TER_DV_SYSTEM_STATUS", "as_status"): "as_status",
}


# =====================================================================
# MODELO PACEJKA MF6.2-LITE — funciones puras para la "Pacejka Lab"
# =====================================================================
# OJO: estas curvas son analíticas/paramétricas, NO telemetría real. El
# TeR no tiene sensor de presión ni de camber (ver TeR_CAN.h). Están
# ancladas a las constantes reales usadas en gp_vehicle_model.h y
# gp_traction_control.h para que al menos el orden de magnitud sea
# consistente con lo que corre en la ECU. Los coeficientes de camber y
# presión son estimaciones de banco típicas para slicks F-SAE — hay que
# reemplazarlas por datos de tire test cuando/si los tengáis.

GP_MASS_PY, GP_GRAVITY_PY = 300.0, 9.81          # gp_vehicle_model.h
GP_H_CG_PY, GP_WB_PY = 0.330, 0.8525 + 0.6975    # gp_vehicle_model.h
GP_AIR_DENSITY_PY, GP_AERO_CL_REAR_PY, GP_AERO_AREA_PY = 1.225, 2.27, 1.10
GP_R_WHEEL_PY = 0.2032                            # gp_vehicle_model.h

PAC_MU_NOM = 1.5           # GP_MU_NOM
PAC_C_ALPHA_F = 35000.0    # GP_C_ALPHA_F [N/rad]
PAC_C_ALPHA_R = 32000.0    # GP_C_ALPHA_R [N/rad]
PAC_FZ_NOM = 0.25 * GP_MASS_PY * GP_GRAVITY_PY   # ~735 N carga estática/rueda
PAC_CAMBER_STIFF = 900.0     # N/rad, orden de magnitud típico F-SAE (AJUSTAR con banco)
PAC_ROLL_RES_COEF = 0.012    # coef. resistencia a la rodadura típico slick


def magic_formula(x, B, C, D, E=0.0):
    Bx = B * x
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


def pac_fx_kappa(kappa_pct, fz=PAC_FZ_NOM):
    D = PAC_MU_NOM * fz
    B = 10.0 / (D + 1e-3)
    return magic_formula(np.asarray(kappa_pct) / 100.0, B=B, C=1.65, D=D, E=-2.0)


def pac_fy_alpha(alpha_deg, fz=PAC_FZ_NOM, c_alpha=PAC_C_ALPHA_F):
    D = PAC_MU_NOM * fz
    B = c_alpha / (1.4 * D + 1e-3)
    return magic_formula(np.radians(alpha_deg), B=B, C=1.4, D=D, E=-1.0)


def pac_pneumatic_trail(alpha_deg, fz=PAC_FZ_NOM):
    return 0.04 * (fz / PAC_FZ_NOM) * np.exp(-3.0 * np.abs(np.radians(alpha_deg)))


def pac_mz_alpha(alpha_deg, fz=PAC_FZ_NOM):
    fy = pac_fy_alpha(alpha_deg, fz)
    return -fy * pac_pneumatic_trail(alpha_deg, fz)


def pac_fy_camber(gamma_deg, fz=PAC_FZ_NOM):
    return PAC_CAMBER_STIFF * np.radians(gamma_deg) * (fz / PAC_FZ_NOM)


def pac_mx_camber(gamma_deg, fz=PAC_FZ_NOM):
    return 0.6 * pac_fy_camber(gamma_deg, fz) * 0.05  # brazo ~5cm


def pac_my_rolling(fz_axis):
    return PAC_ROLL_RES_COEF * np.asarray(fz_axis) * GP_R_WHEEL_PY


def pac_fz_vs_speed(vx_ms, ax=0.0):
    """Replica simplificada de gp_estimate_fz() [gp_vehicle_model.c], eje trasero."""
    fz_static = PAC_FZ_NOM
    dfz_lon = GP_MASS_PY * ax * GP_H_CG_PY / (GP_WB_PY + 1e-3)
    downforce_rear = 0.5 * GP_AIR_DENSITY_PY * (np.asarray(vx_ms) ** 2) * GP_AERO_CL_REAR_PY * GP_AERO_AREA_PY
    return fz_static + dfz_lon + downforce_rear * 0.5


def pac_fy_vs_kappa(kappa_pct, alpha_deg=5.0, fz=PAC_FZ_NOM):
    """Elipse de Kamm: cuánta Fy queda disponible mientras se usa Fx."""
    fy_pure = pac_fy_alpha(alpha_deg, fz)
    fx_pure = pac_fx_kappa(kappa_pct, fz)
    fy_max = PAC_MU_NOM * fz
    remaining = np.sqrt(np.clip(1.0 - (fx_pure / (fy_max + 1e-3)) ** 2, 0.0, 1.0))
    return fy_pure * remaining


def pac_fx_vs_alpha(alpha_deg, kappa_pct=10.0, fz=PAC_FZ_NOM):
    fx_pure = pac_fx_kappa(kappa_pct, fz)
    fy_pure = pac_fy_alpha(alpha_deg, fz)
    fx_max = PAC_MU_NOM * fz
    remaining = np.sqrt(np.clip(1.0 - (fy_pure / (fx_max + 1e-3)) ** 2, 0.0, 1.0))
    return fx_pure * remaining


def pac_mz_vs_kappa(kappa_pct, alpha_deg=5.0, fz=PAC_FZ_NOM):
    fy_combined = pac_fy_vs_kappa(kappa_pct, alpha_deg, fz)
    return -fy_combined * pac_pneumatic_trail(alpha_deg, fz)


def pac_friction_ellipse(fx_axis, fz=PAC_FZ_NOM):
    fmax = PAC_MU_NOM * fz
    return fmax * np.sqrt(np.clip(1.0 - (np.asarray(fx_axis) / fmax) ** 2, 0.0, 1.0))


def pac_peak_vs_fz(fz_axis):
    return PAC_MU_NOM * np.asarray(fz_axis)


def pac_fx_vs_pressure(p_bar, fz=PAC_FZ_NOM):
    mu_p = PAC_MU_NOM * (1.0 - 0.06 * (np.asarray(p_bar) - 0.9))
    return mu_p * fz


def pac_fy_vs_pressure(p_bar, fz=PAC_FZ_NOM):
    return pac_fx_vs_pressure(p_bar, fz)  # mismo mu(p) aproximado


# =====================================================================
# BUS DE DATOS (thread-safe, ring buffers con timestamp)
# =====================================================================
class DataHub:
    def __init__(self, channels: List[ChannelSpec]):
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.buffers: Dict[str, deque] = {c.key: deque(maxlen=HISTORY_LEN) for c in channels}
        self.latest: Dict[str, float] = {c.key: 0.0 for c in channels}

    def push(self, key: str, t: float, value: float) -> None:
        with self.lock:
            buf = self.buffers.get(key)
            if buf is None:
                return
            buf.append((t, value))
            self.latest[key] = value

    def snapshot(self, key: str):
        with self.lock:
            buf = self.buffers[key]
            if not buf:
                return np.empty(0), np.empty(0)
            t, y = zip(*buf)
            return np.asarray(t), np.asarray(y)


def i16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "big", signed=True)


def u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "big", signed=False)


# =====================================================================
# WORKER: SIMULACIÓN (sin hardware)
# =====================================================================
class SimWorker(threading.Thread):
    """Genera formas de onda plausibles para desarrollar/probar la UI sin CAN."""

    def __init__(self, hub: DataHub):
        super().__init__(daemon=True)
        self.hub = hub
        self._stop = threading.Event()

    def run(self) -> None:
        t = 0.0
        while not self._stop.is_set():
            t += 0.05
            now = time.monotonic() - self.hub.t0
            for key, value in _generate_sim_frame(t).items():
                self.hub.push(key, now, value)
            time.sleep(0.05)

    def stop(self) -> None:
        self._stop.set()


def _generate_sim_frame(t: float) -> Dict[str, float]:
    noise = lambda s=1.0: float(np.random.normal(0, s))

    vx = max(0.0, 20.0 + 10.0 * math.sin(t * 0.1))
    k_base = 12.0 + 2.0 * math.sin(t * 0.5)

    return {
        "vy_est": 0.5 * math.sin(t),
        "wz_int": 120.0 * math.sin(t * 0.15),
        "delta_trq": 20.0 * math.sin(t * 2.0),
        "yaw_ref": 15.0 * math.sin(t * 0.7),

        "kappa_opt_rl": k_base,
        "kappa_opt_rr": k_base * 0.95,
        "kappa_filt_rl": k_base + noise(0.8),
        "kappa_filt_rr": k_base * 0.95 + noise(0.8),
        "theta_rl": 20000.0 + 10000.0 * math.sin(t * 0.8),
        "theta_rr": 20000.0 + 8000.0 * math.cos(t * 0.8),
        "mu_rl": 1.2 + 0.1 * math.sin(t * 0.1),
        "mu_rr": 1.2 + 0.1 * math.cos(t * 0.1),

        "rl_rpm": vx / 0.2032 * 60.0 / (2.0 * math.pi) * 5.0,
        "rr_rpm": vx / 0.2032 * 60.0 / (2.0 * math.pi) * 5.0 * (1.0 + 0.01 * math.sin(t)),
        "rl_trq": 150.0 + 50.0 * math.sin(t * 2.0),
        "rr_trq": 150.0 + 50.0 * math.sin(t * 2.0 + 0.5),
        "t_out_rl": 150.0 + 50.0 * math.sin(t * 2.0),
        "t_out_rr": 150.0 + 50.0 * math.sin(t * 2.0 + 0.5),
        "vehicle_speed": vx * 3.6,

        "left_dem": 60.0 + 20.0 * math.sin(t * 0.3),
        "right_dem": 60.0 + 20.0 * math.sin(t * 0.3 + 0.2),
        "left_motor_temp": 55.0 + 10.0 * math.sin(t * 0.05) + t * 0.01,
        "right_motor_temp": 56.0 + 10.0 * math.sin(t * 0.05 + 0.1) + t * 0.01,
        "left_power_stage_temp": 45.0 + 8.0 * math.sin(t * 0.05),
        "right_power_stage_temp": 46.0 + 8.0 * math.sin(t * 0.05 + 0.1),

        "accel_x": 2.0 * math.sin(t * 0.4),
        "accel_y": 6.0 * math.sin(t * 0.3 + 1.0),
        "accel_z": 9.81 + 0.3 * math.sin(t * 2.0),
        "yaw_rate": 20.0 * math.sin(t * 0.3),
        "pitch_rate": 5.0 * math.sin(t * 0.5),
        "roll_rate": 8.0 * math.sin(t * 0.6),
        "yaw": 10.0 * math.sin(t * 0.05),
        "pitch": 2.0 * math.sin(t * 0.1),
        "roll": 3.0 * math.sin(t * 0.15),

        "apps": max(0.0, min(100.0, 50.0 + 45.0 * math.sin(t * 0.2))),
        "bpps": max(0.0, -40.0 * math.sin(t * 0.2)),
        "steer_angle": 30.0 * math.sin(t * 0.25),

        "cell_max_volt": 4100.0 + 30.0 * math.sin(t * 0.05),
        "cell_min_volt": 4050.0 + 30.0 * math.sin(t * 0.05),
        "cell_max_temp": 35.0 + 5.0 * math.sin(t * 0.02) + t * 0.005,
        "hv_current": 80.0 * math.sin(t * 0.3),

        "as_status": 3.0,
        "r2d": 1.0,
        "sl_relay": 1.0,
    }


# =====================================================================
# WORKER: CAN REAL (python-can + cantools)
# =====================================================================
class CanWorker(threading.Thread):
    def __init__(self, hub: DataHub, channel: str = CAN_CHANNEL,
                 bitrate: int = CAN_BITRATE, dbc_paths: Optional[List[str]] = None):
        super().__init__(daemon=True)
        self.hub = hub
        self.channel = channel
        self.bitrate = bitrate
        self._stop = threading.Event()
        self.dbs = []
        if HAVE_CANTOOLS:
            for path in (dbc_paths or DBC_FILES):
                try:
                    self.dbs.append(cantools.database.load_file(path))
                except (FileNotFoundError, OSError):
                    print(f"[CanWorker] DBC no encontrado, se omite decode genérico: {path}")
        if not self.dbs:
            print("[CanWorker] Sin DBC cargado — solo se decodificarán las tramas "
                  "propietarias AL-QP/TC (0x100-0x102).")

    def run(self) -> None:
        if not HAVE_PYTHON_CAN:
            print("[CanWorker] python-can no está instalado — no se puede usar CAN real.")
            return
        bus = can.interface.Bus(bustype=CAN_BUSTYPE, channel=self.channel, bitrate=self.bitrate)
        try:
            while not self._stop.is_set():
                msg = bus.recv(timeout=1.0)
                if msg is None:
                    continue
                now = time.monotonic() - self.hub.t0
                self._decode(msg.arbitration_id, msg.data, now)
        finally:
            bus.shutdown()

    def stop(self) -> None:
        self._stop.set()

    def _decode(self, mid: int, d: bytes, now: float) -> None:
        # --- Tramas propietarias del AL-QP / TC, replican gp_pack_telemetry() ---
        if mid == 0x100 and len(d) >= 8:
            self.hub.push("vy_est", now, i16(d, 0) / 100.0)
            self.hub.push("wz_int", now, i16(d, 2) / 100.0)          # antes mal leído como qp_residual
            self.hub.push("kappa_opt_rl", now, u16(d, 4) / 100.0)    # raw/10000 -> fracción; *100 -> %
            self.hub.push("kappa_opt_rr", now, u16(d, 6) / 100.0)
            return
        if mid == 0x101 and len(d) >= 8:
            self.hub.push("theta_rl", now, i16(d, 0) * 10.0)
            self.hub.push("theta_rr", now, i16(d, 2) * 10.0)
            self.hub.push("mu_rl", now, u16(d, 4) / 1000.0)
            self.hub.push("mu_rr", now, u16(d, 6) / 1000.0)
            return
        if mid == 0x102 and len(d) >= 8:
            self.hub.push("t_out_rl", now, i16(d, 0) / 10.0)
            self.hub.push("t_out_rr", now, i16(d, 2) / 10.0)
            self.hub.push("kappa_filt_rl", now, u16(d, 4) / 100.0)
            self.hub.push("kappa_filt_rr", now, u16(d, 6) / 100.0)
            return

        # --- Resto de señales vía DBC / cantools -------------------------------
        for db in self.dbs:
            try:
                message = db.get_message_by_frame_id(mid)
                decoded = db.decode_message(mid, d)
            except Exception:
                continue
            for sig_name, value in decoded.items():
                key = SIGNAL_MAP.get((message.name, sig_name))
                if key is None:
                    continue
                try:
                    self.hub.push(key, now, float(value))
                except (TypeError, ValueError):
                    pass
            break


# =====================================================================
# UI: SELECTOR DE CANALES
# =====================================================================
class ChannelSelector(QtWidgets.QWidget):
    def __init__(self, channels: List[ChannelSpec], on_change: Callable[[set], None], parent=None):
        super().__init__(parent)
        self.channels = {c.key: c for c in channels}
        self._active: set = set()
        self._on_change = on_change

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        title = QtWidgets.QLabel("SELECTOR DE CANALES")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        preset_grid = QtWidgets.QGridLayout()
        for i, name in enumerate(PRESETS):
            btn = QtWidgets.QPushButton(name)
            btn.setObjectName("presetBtn")
            btn.clicked.connect(lambda _checked, n=name: self._apply_preset(n))
            preset_grid.addWidget(btn, i // 2, i % 2)
        layout.addLayout(preset_grid)

        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        layout.addWidget(line)

        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.itemChanged.connect(self._on_item_changed)
        layout.addWidget(self.tree, 1)

        self._chan_items: Dict[str, QtWidgets.QTreeWidgetItem] = {}
        self._build_tree()

    def _build_tree(self) -> None:
        by_group: Dict[str, List[ChannelSpec]] = {}
        for c in self.channels.values():
            by_group.setdefault(c.group, []).append(c)

        self.tree.blockSignals(True)
        for group in GROUP_ORDER:
            gi = QtWidgets.QTreeWidgetItem(self.tree, [group])
            gi.setFlags(gi.flags() | Qt.ItemIsUserCheckable)
            gi.setCheckState(0, Qt.Unchecked)
            gi.setData(0, Qt.UserRole, None)
            for c in by_group.get(group, []):
                ci = QtWidgets.QTreeWidgetItem(gi, [f"{c.label}  [{c.units}]"])
                ci.setFlags(ci.flags() | Qt.ItemIsUserCheckable)
                ci.setCheckState(0, Qt.Unchecked)
                ci.setData(0, Qt.UserRole, c.key)
                self._chan_items[c.key] = ci
        self.tree.expandAll()
        self.tree.blockSignals(False)

    def _on_item_changed(self, item: QtWidgets.QTreeWidgetItem, _col: int) -> None:
        key = item.data(0, Qt.UserRole)
        if key is None:
            # cabecera de grupo -> cascada a los hijos
            state = item.checkState(0)
            self.tree.blockSignals(True)
            for i in range(item.childCount()):
                child = item.child(i)
                child.setCheckState(0, state)
                ck = child.data(0, Qt.UserRole)
                if state == Qt.Checked:
                    self._active.add(ck)
                else:
                    self._active.discard(ck)
            self.tree.blockSignals(False)
        else:
            if item.checkState(0) == Qt.Checked:
                self._active.add(key)
            else:
                self._active.discard(key)
        self._on_change(set(self._active))

    def _apply_preset(self, name: str) -> None:
        keys = PRESETS[name]
        self.tree.blockSignals(True)
        for key, item in self._chan_items.items():
            item.setCheckState(0, Qt.Checked if key in keys else Qt.Unchecked)
        self.tree.blockSignals(False)
        self._active = set(keys)
        self._on_change(set(self._active))


# =====================================================================
# UI: TILE DE GRÁFICA INDIVIDUAL
# =====================================================================
class PlotTile(QtWidgets.QFrame):
    def __init__(self, spec: ChannelSpec, parent=None):
        super().__init__(parent)
        self.spec = spec
        self.setObjectName("plotTile")

        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(2)

        header = QtWidgets.QHBoxLayout()
        dot = QtWidgets.QLabel("\u25CF")
        dot.setStyleSheet(f"color:{spec.color}; font-size:14px;")
        label = QtWidgets.QLabel(spec.label)
        label.setObjectName("tileLabel")
        self.value_lbl = QtWidgets.QLabel("--")
        self.value_lbl.setObjectName("tileValue")
        self.value_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        header.addWidget(dot)
        header.addWidget(label)
        header.addStretch(1)
        header.addWidget(self.value_lbl)
        v.addLayout(header)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.getAxis("bottom").setStyle(showValues=False)
        self.plot.setMinimumHeight(150)
        if spec.y_range:
            self.plot.setYRange(*spec.y_range)
        else:
            self.plot.enableAutoRange(axis="y")
        self.curve = self.plot.plot(pen=pg.mkPen(spec.color, width=2))
        v.addWidget(self.plot)

    def update_data(self, t: np.ndarray, y: np.ndarray) -> Optional[str]:
        if len(t) == 0:
            return None
        self.curve.setData(t, y)
        val = float(y[-1])
        self.value_lbl.setText(f"{val:.{self.spec.decimals}f} {self.spec.units}")

        status = "normal"
        if self.spec.crit and self.spec.crit(val):
            status = "crit"
        elif self.spec.warn and self.spec.warn(val):
            status = "warn"
        color = {"normal": "#d1d1d1", "warn": "#FFEA00", "crit": "#FF3333"}[status]
        self.value_lbl.setStyleSheet(f"color:{color}; font-weight:bold;")
        return status


# =====================================================================
# UI: ÁREA DE DASHBOARD (grid dinámico de tiles)
# =====================================================================
class DashboardArea(QtWidgets.QScrollArea):
    def __init__(self, channels: Dict[str, ChannelSpec], columns: int = 3, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.channels = channels
        self.columns = columns

        self.container = QtWidgets.QWidget()
        self.grid = QtWidgets.QGridLayout(self.container)
        self.grid.setSpacing(8)
        self.setWidget(self.container)

        self.tiles: Dict[str, PlotTile] = {}
        self.active_order: List[str] = []

    def set_active(self, active_keys: set) -> None:
        ordered = [k for k in self.channels if k in active_keys]
        if ordered == self.active_order:
            return
        self.active_order = ordered

        while self.grid.count():
            item = self.grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)

        for idx, key in enumerate(ordered):
            tile = self.tiles.get(key)
            if tile is None:
                tile = PlotTile(self.channels[key])
                self.tiles[key] = tile
            r, c = divmod(idx, self.columns)
            self.grid.addWidget(tile, r, c)

    def refresh(self, hub: DataHub) -> Dict[str, List[str]]:
        alerts: Dict[str, List[str]] = {"warn": [], "crit": []}
        for key in self.active_order:
            t, y = hub.snapshot(key)
            tile = self.tiles[key]
            status = tile.update_data(t, y)
            if status == "warn":
                alerts["warn"].append(tile.spec.label)
            elif status == "crit":
                alerts["crit"].append(tile.spec.label)
        return alerts


# =====================================================================
# ESTILO — Dark theme estilo MoTeC
# =====================================================================
DARK_QSS = """
QMainWindow, QWidget { background-color: #0a0a0a; color: #d1d1d1; font-family: 'Consolas', monospace; }
#topBar { background-color: #1a1a1a; border-bottom: 1px solid #333333; }
#modeLabel, #clockLabel { font-size: 13px; font-weight: bold; color: #d1d1d1; }
#alertLabel { font-size: 14px; }
#panelTitle { font-size: 13px; font-weight: bold; color: #00E5FF; padding: 4px 0; letter-spacing: 1px; }
#presetBtn { background-color: #1a1a1a; border: 1px solid #333333; border-radius: 3px; padding: 4px; color: #d1d1d1; }
#presetBtn:hover { background-color: #262626; border-color: #00E5FF; }
QTreeWidget { background-color: #111111; border: 1px solid #262626; }
QTreeWidget::item { padding: 3px; }
#plotTile { background-color: #131313; border: 1px solid #262626; border-radius: 4px; }
#tileLabel { font-size: 12px; color: #b0b0b0; }
#tileValue { font-size: 14px; font-family: 'Consolas', monospace; }
QPushButton#recordBtn { background-color: #1a1a1a; border: 1px solid #333333; border-radius: 3px; padding: 6px 12px; }
QScrollBar:vertical { background: #111111; width: 10px; }
QScrollBar::handle:vertical { background: #333333; border-radius: 4px; }
"""

# =====================================================================
# COMPONENTES VISUALES AVANZADOS (Añadir antes de MainWindow)
# =====================================================================

class MultiPlot(pg.PlotWidget):
    """ Gráfica que superpone múltiples canales en un solo eje """
    def __init__(self, title, channels, y_range=None):
        super().__init__(title=title)
        self.showGrid(x=True, y=True, alpha=0.3)
        self.addLegend(offset=(10, 10))
        if y_range:
            self.setYRange(*y_range)
        
        self.channels = channels
        self.curves = {}
        for key, color, name in channels:
            self.curves[key] = self.plot(pen=pg.mkPen(color, width=2), name=name)
            
    def update_data(self, hub):
        for key, _, _ in self.channels:
            t, y = hub.snapshot(key)
            if len(t) > 0:
                self.curves[key].setData(t, y)

class GGDiagram(pg.PlotWidget):
    """ Diagrama G-G (Friction Circle) con estela de 200 puntos """
    def __init__(self):
        super().__init__(title="DIAGRAMA G-G (Friction Circle)")
        self.setXRange(-20, 20)
        self.setYRange(-20, 20)
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setAspectLocked(True) # Círculo perfecto
        
        # Dibujar círculo teórico de 1.5G (~15 m/s2)
        circle = QtWidgets.QGraphicsEllipseItem(-15, -15, 30, 30)
        circle.setPen(pg.mkPen('#555555', width=2, style=Qt.DashLine))
        self.addItem(circle)
        
        self.scatter = pg.ScatterPlotItem(size=6, pen=pg.mkPen(None), brush=pg.mkBrush('#00E5FF'))
        self.addItem(self.scatter)
        self.current_dot = pg.ScatterPlotItem(size=12, pen=pg.mkPen('w'), brush=pg.mkBrush('#FF3333'))
        self.addItem(self.current_dot)

    def update_data(self, hub):
        _, ax = hub.snapshot("accel_x")
        _, ay = hub.snapshot("accel_y")
        if len(ax) > 0 and len(ay) > 0:
            # Mostrar los últimos 200 puntos como estela
            self.scatter.setData(x=ay[-200:], y=ax[-200:])
            self.current_dot.setData(x=[ay[-1]], y=[ax[-1]])

class PacejkaTracer(pg.PlotWidget):
    """ Recrea la campana de Pacejka en vivo y ubica la rueda en ella """
    def __init__(self, wheel_name, c_theta, c_mu, c_slip, c_trq):
        super().__init__(title=f"PACEJKA ANALYTICS - {wheel_name}")
        self.keys = (c_theta, c_mu, c_slip, c_trq)
        self.setXRange(0, 30)  # Slip %
        self.setYRange(0, 350) # Torque Nm
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setLabel('bottom', "Slip Ratio (%)")
        self.setLabel('left', "Fuerza / Torque (Nm)")
        
        self.theory_curve = self.plot(pen=pg.mkPen('#44FF44', width=2, style=Qt.DashLine), name="Límite Teórico RLS")
        self.current_dot = pg.ScatterPlotItem(size=14, brush=pg.mkBrush('#FF00FF'))
        self.addItem(self.current_dot)

    def update_data(self, hub):
        theta = hub.latest.get(self.keys[0], 25000)
        mu = hub.latest.get(self.keys[1], 1.0)
        slip = hub.latest.get(self.keys[2], 0.0)
        trq = hub.latest.get(self.keys[3], 0.0)
        
        # Calcular campana de Pacejka aproximada (D*sin(C*arctan(B*kappa)))
        D = mu * 180.0 # Aproximación de Torque Max
        C = 1.65
        B = theta / (C * D * 100.0) if D > 0 else 0
        
        k_vals = np.linspace(0, 30, 60)
        f_vals = D * np.sin(C * np.arctan(B * k_vals))
        
        self.theory_curve.setData(k_vals, f_vals)
        self.current_dot.setData(x=[slip], y=[trq])

class ChassisTopDown(QtWidgets.QWidget):
    """ Renderizado del chasis visto desde arriba con vectores de fuerza """
    def __init__(self):
        super().__init__()
        self.setMinimumSize(250, 400)

    def update_data(self, hub):
        self.vy = hub.latest.get("vy_est", 0)
        self.wz = hub.latest.get("yaw_rate", 0)
        self.trl = hub.latest.get("t_out_rl", 0)
        self.trr = hub.latest.get("t_out_rr", 0)
        self.steer = hub.latest.get("steer_angle", 0)
        self.ay = hub.latest.get("accel_y", 0)  # proxy de Fy en eje delantero/trasero
        self.update() # Fuerza el repintado

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        
        p.fillRect(0, 0, w, h, QColor("#111111"))
        
        # Centro
        cx, cy = w // 2, h // 2
        car_w, car_h = 60, 160
        
        # Dibujar Chasis
        p.setPen(QPen(QColor("#00E5FF"), 2))
        p.setBrush(QBrush(QColor("#222222")))
        p.drawRoundedRect(cx - car_w//2, cy - car_h//2, car_w, car_h, 10, 10)
        
        # Dibujar Ruedas Delanteras (con dirección)
        p.setBrush(QBrush(QColor("#555555")))
        p.translate(cx - car_w//2 - 10, cy - car_h//2 + 20)
        p.rotate(self.steer)
        p.drawRect(-5, -15, 10, 30)
        p.rotate(-self.steer)
        p.translate(car_w + 20, 0)
        p.rotate(self.steer)
        p.drawRect(-5, -15, 10, 30)
        p.resetTransform()
        
        # Vectores de Torque Traseros
        scale = 0.3
        p.setPen(QPen(QColor("#FF00FF"), 4))
        p.drawLine(cx - car_w//2 - 10, cy + car_h//2 - 20, 
                   cx - car_w//2 - 10, cy + car_h//2 - 20 - int(self.trl * scale))
        
        p.setPen(QPen(QColor("#FFEA00"), 4))
        p.drawLine(cx + car_w//2 + 10, cy + car_h//2 - 20, 
                   cx + car_w//2 + 10, cy + car_h//2 - 20 - int(self.trr * scale))
                   
        # Vector Deriva (vy)
        p.setPen(QPen(QColor("#FF3333"), 3))
        p.drawLine(cx, cy, cx + int(self.vy * 40), cy)

        # Vectores de Fuerza Lateral por eje (proxy: ay repartido delante/detrás,
        # no hay Fy por rueda medida directamente, solo estimada en gp_vehicle_model.c)
        fy_len = int(getattr(self, "ay", 0.0) * 3.0)
        p.setPen(QPen(QColor("#00FFAA"), 3))
        p.drawLine(cx - car_w // 2, cy - car_h // 2 + 20, cx - car_w // 2 - fy_len, cy - car_h // 2 + 20)
        p.drawLine(cx + car_w // 2, cy - car_h // 2 + 20, cx + car_w // 2 - fy_len, cy - car_h // 2 + 20)
        p.drawLine(cx - car_w // 2, cy + car_h // 2 - 20, cx - car_w // 2 - fy_len, cy + car_h // 2 - 20)
        p.drawLine(cx + car_w // 2, cy + car_h // 2 - 20, cx + car_w // 2 - fy_len, cy + car_h // 2 - 20)

class ActiveSetLEDs(QtWidgets.QWidget):
    """ Matriz de estado del Solver KKT """
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        self.leds = {}
        
        # Definir los "LEDs"
        checks = ["AL-QP Convergencia", "Límite Térmico Inv.", "Límite Fricción (Kamm)", "Asimetría / Mu-Split"]
        for name in checks:
            row = QtWidgets.QHBoxLayout()
            led = QtWidgets.QLabel("⚫")
            led.setFont(QFont("Arial", 20))
            lbl = QtWidgets.QLabel(name)
            lbl.setStyleSheet("color: white; font-weight: bold;")
            row.addWidget(led)
            row.addWidget(lbl)
            row.addStretch()
            layout.addLayout(row)
            self.leds[name] = led

    def update_data(self, hub):
        res = hub.latest.get("wz_int", 0) # Usamos wz_int como proxy visual del residuo temporalmente
        temp = max(hub.latest.get("left_motor_temp", 0), hub.latest.get("right_motor_temp", 0))
        ay = hub.latest.get("accel_y", 0)
        mu_diff = abs(hub.latest.get("mu_rl", 1) - hub.latest.get("mu_rr", 1))

        self._set_led("AL-QP Convergencia", abs(res) > 150, "red")
        self._set_led("Límite Térmico Inv.", temp > 85, "orange")
        self._set_led("Límite Fricción (Kamm)", abs(ay) > 12, "cyan")
        self._set_led("Asimetría / Mu-Split", mu_diff > 0.4, "yellow")

    def _set_led(self, name, condition, color_name):
        colors = {"red": "🔴", "orange": "🟠", "cyan": "🔵", "yellow": "🟡"}
        self.leds[name].setText(colors[color_name] if condition else "🟢")


# =====================================================================
# PACEJKA LAB — gráfica genérica reutilizable
# =====================================================================
class PacejkaCurvePlot(pg.PlotWidget):
    """Familia de curvas Pacejka (barrido de un parámetro) + punto de
    operación en vivo superpuesto cuando hay canales de telemetría reales
    que representarlo (kappa/alpha/Fz medidos/estimados)."""

    def __init__(self, title, xlabel, ylabel, curve_fn, x_range,
                 sweep_values=(None,), sweep_labels=(None,),
                 live_x_key=None, live_y_key=None, colors=None):
        super().__init__(title=title)
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setLabel('bottom', xlabel)
        self.setLabel('left', ylabel)
        self.setMinimumHeight(230)

        colors = colors or ['#00E5FF', '#FF00FF', '#FFEA00', '#44FF44', '#FF8800']
        x_vals = np.linspace(*x_range, 200)

        if len(sweep_values) > 1:
            self.addLegend(offset=(10, 10))

        for i, (val, lbl) in enumerate(zip(sweep_values, sweep_labels)):
            y_vals = curve_fn(x_vals, val) if val is not None else curve_fn(x_vals)
            self.plot(x_vals, y_vals, pen=pg.mkPen(colors[i % len(colors)], width=2), name=lbl)

        self.live_x_key = live_x_key
        self.live_y_key = live_y_key
        self.live_dot = None
        if live_x_key and live_y_key:
            self.live_dot = pg.ScatterPlotItem(size=12, pen=pg.mkPen('w'), brush=pg.mkBrush('#FF3333'))
            self.addItem(self.live_dot)

    def update_data(self, hub) -> None:
        if self.live_dot is None:
            return
        x = hub.latest.get(self.live_x_key)
        y = hub.latest.get(self.live_y_key)
        if x is not None and y is not None:
            self.live_dot.setData(x=[x], y=[y])


# Configuración de las 14 gráficas pedidas (título, ejes, función, barrido, live overlay)
_PACEJKA_SPECS = [
    dict(title="1. Fx vs Slip Ratio (κ)", xlabel="κ (%)", ylabel="Fx (N)",
         x_range=(-30, 30), curve_fn=pac_fx_kappa,
         sweep_values=(1200.0, 2200.0, 3200.0), sweep_labels=("Fz=1200N", "Fz=2200N", "Fz=3200N"),
         live_x_key="kappa_filt_rr", live_y_key="t_out_rr"),
    dict(title="2. Fy vs Slip Angle (α)", xlabel="α (deg)", ylabel="Fy (N)",
         x_range=(-15, 15), curve_fn=pac_fy_alpha,
         sweep_values=(1200.0, 2200.0, 3200.0), sweep_labels=("Fz=1200N", "Fz=2200N", "Fz=3200N")),
    dict(title="3. Fz vs Velocidad (aero + transf. carga)", xlabel="Vx (m/s)", ylabel="Fz (N)",
         x_range=(0, 30), curve_fn=pac_fz_vs_speed,
         sweep_values=(-8.0, 0.0, 8.0), sweep_labels=("ax=-8 (frenada)", "ax=0", "ax=+8 (tracción)"),
         live_x_key="vehicle_speed", live_y_key="rr_trq"),
    dict(title="4. Mz vs Slip Angle (α)", xlabel="α (deg)", ylabel="Mz (N·m)",
         x_range=(-15, 15), curve_fn=pac_mz_alpha,
         sweep_values=(1200.0, 2200.0, 3200.0), sweep_labels=("Fz=1200N", "Fz=2200N", "Fz=3200N")),
    dict(title="5. Mx vs Camber (γ)", xlabel="γ (deg)", ylabel="Mx (N·m)",
         x_range=(-4, 4), curve_fn=pac_mx_camber,
         sweep_values=(PAC_FZ_NOM,), sweep_labels=(None,)),
    dict(title="6. My (rodadura) vs Carga Fz", xlabel="Fz (N)", ylabel="My (N·m)",
         x_range=(0, 3500), curve_fn=lambda x: pac_my_rolling(x)),
    dict(title="7. Elipse de Adherencia (Fy vs Fx)", xlabel="Fx (N)", ylabel="Fy (N)",
         x_range=(-PAC_FZ_NOM * PAC_MU_NOM, PAC_FZ_NOM * PAC_MU_NOM), curve_fn=pac_friction_ellipse),
    dict(title="8. Fy combinada vs Slip Ratio (κ)", xlabel="κ (%)", ylabel="Fy (N)",
         x_range=(-30, 30), curve_fn=pac_fy_vs_kappa,
         sweep_values=(2.0, 5.0, 9.0), sweep_labels=("α=2°", "α=5°", "α=9°"),
         live_x_key="kappa_filt_rr", live_y_key="delta_trq"),
    dict(title="9. Fx combinada vs Slip Angle (α)", xlabel="α (deg)", ylabel="Fx (N)",
         x_range=(-15, 15), curve_fn=pac_fx_vs_alpha,
         sweep_values=(5.0, 12.0, 20.0), sweep_labels=("κ=5%", "κ=12%", "κ=20%"),
         live_x_key="steer_angle", live_y_key="t_out_rr"),
    dict(title="10. Mz combinado vs Slip Ratio (κ)", xlabel="κ (%)", ylabel="Mz (N·m)",
         x_range=(-30, 30), curve_fn=pac_mz_vs_kappa,
         sweep_values=(2.0, 5.0, 9.0), sweep_labels=("α=2°", "α=5°", "α=9°")),
    dict(title="11. Sensibilidad a Fz (pico Fx/Fy ≈ μ·Fz)", xlabel="Fz (N)", ylabel="Fuerza pico (N)",
         x_range=(0, 3500), curve_fn=lambda x: pac_peak_vs_fz(x)),
    dict(title="12. Sensibilidad a Camber (Fy vs γ)", xlabel="γ (deg)", ylabel="Fy (N)",
         x_range=(-4, 4), curve_fn=pac_fy_camber,
         sweep_values=(1200.0, 2200.0, 3200.0), sweep_labels=("Fz=1200N", "Fz=2200N", "Fz=3200N")),
    dict(title="13. Sensibilidad a Presión (Fx/Fy vs Pi)", xlabel="Presión (bar)", ylabel="Fuerza pico (N)",
         x_range=(0.5, 1.4), curve_fn=pac_fx_vs_pressure),
    dict(title="14. Pneumatic Trail (t) vs Slip Angle (α)", xlabel="α (deg)", ylabel="t (m)",
         x_range=(-15, 15), curve_fn=pac_pneumatic_trail,
         sweep_values=(1200.0, 2200.0, 3200.0), sweep_labels=("Fz=1200N", "Fz=2200N", "Fz=3200N")),
]


class PacejkaLabPanel(QtWidgets.QScrollArea):
    """Grid con las 14 gráficas de caracterización Pacejka (MF6.2-lite,
    paramétrico + overlay de operación en vivo donde aplica)."""

    def __init__(self, columns: int = 2):
        super().__init__()
        self.setWidgetResizable(True)
        container = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(container)
        grid.setSpacing(8)
        self.setWidget(container)

        self.plots: List[PacejkaCurvePlot] = []
        for idx, spec in enumerate(_PACEJKA_SPECS):
            p = PacejkaCurvePlot(**spec)
            r, c = divmod(idx, columns)
            grid.addWidget(p, r, c)
            self.plots.append(p)

    def update_data(self, hub) -> None:
        for p in self.plots:
            p.update_data(hub)


# =====================================================================
# VERIFICADOR DE SENSORES
# =====================================================================
STALE_THRESHOLD_S = 1.0

class SensorHealthPanel(QtWidgets.QWidget):
    """Comprueba que cada canal siga actualizándose (no muerto en el bus)
    y que su último valor esté dentro de rango físico plausible."""

    def __init__(self, channels: List[ChannelSpec]):
        super().__init__()
        self.channels = channels
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("VERIFICADOR DE SENSORES")
        title.setObjectName("panelTitle")
        layout.addWidget(title)

        self.table = QtWidgets.QTableWidget(len(channels), 3)
        self.table.setHorizontalHeaderLabels(["Canal", "Último Valor", "Estado"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        for row, c in enumerate(channels):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(c.label))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem("--"))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem("SIN DATOS"))
        layout.addWidget(self.table)

    def update_data(self, hub) -> None:
        now = time.monotonic() - hub.t0
        for row, c in enumerate(self.channels):
            t, y = hub.snapshot(c.key)
            if len(t) == 0:
                status, color, val_txt = "SIN DATOS", "#777777", "--"
            else:
                age = now - t[-1]
                val = y[-1]
                val_txt = f"{val:.{c.decimals}f} {c.units}"
                if age > STALE_THRESHOLD_S:
                    status, color = f"MUERTO ({age:.1f}s)", "#FF3333"
                elif c.crit and c.crit(val):
                    status, color = "CRÍTICO", "#FF3333"
                elif c.warn and c.warn(val):
                    status, color = "AVISO", "#FFEA00"
                else:
                    status, color = "OK", "#00FF00"
            self.table.item(row, 1).setText(val_txt)
            item = self.table.item(row, 2)
            item.setText(status)
            item.setForeground(QColor(color))


# =====================================================================
# PANEL IZQUIERDO: G-G + Modelo Vehicular + Salud de Sensores
# =====================================================================
class LeftInfoPanel(QtWidgets.QWidget):
    def __init__(self, channels: List[ChannelSpec]):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.gg = GGDiagram()
        self.gg.setMinimumHeight(260)
        layout.addWidget(self.gg)

        self.chassis = ChassisTopDown()
        layout.addWidget(self.chassis)

        self.health = SensorHealthPanel(channels)
        layout.addWidget(self.health, 1)

    def update_data(self, hub) -> None:
        self.gg.update_data(hub)
        self.chassis.update_data(hub)
        self.health.update_data(hub)


# =====================================================================
# VENTANA PRINCIPAL
# =====================================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, channels: List[ChannelSpec], hub: DataHub):
        super().__init__()
        self.hub = hub
        self.channels_by_key = {c.key: c for c in channels}
        self.setWindowTitle("Tecnun eRacing | TeR Trackside Dashboard")
        self.resize(1700, 1000)

        top = QtWidgets.QWidget()
        top.setObjectName("topBar")
        top_l = QtWidgets.QHBoxLayout(top)
        top_l.setContentsMargins(12, 6, 12, 6)

        self.mode_lbl = QtWidgets.QLabel("\u25CF SIMULADO" if not USE_REAL_CAN else "\u25CF CAN EN VIVO")
        self.mode_lbl.setObjectName("modeLabel")
        self.mode_lbl.setStyleSheet(f"color:{'#FFEA00' if not USE_REAL_CAN else '#00FF00'};")

        self.clock_lbl = QtWidgets.QLabel("t+0.0s")
        self.clock_lbl.setObjectName("clockLabel")

        self.alert_lbl = QtWidgets.QLabel("SISTEMAS NOMINALES")
        self.alert_lbl.setObjectName("alertLabel")
        self.alert_lbl.setStyleSheet("color:#00FF00; font-weight:bold;")

        self.record_btn = QtWidgets.QPushButton("\u25CF INICIAR LOG")
        self.record_btn.setObjectName("recordBtn")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._toggle_logging)

        top_l.addWidget(self.mode_lbl)
        top_l.addSpacing(20)
        top_l.addWidget(self.clock_lbl)
        top_l.addStretch(1)
        top_l.addWidget(self.alert_lbl)
        top_l.addStretch(1)
        top_l.addWidget(self.record_btn)

        self.dashboard = DashboardArea(self.channels_by_key, columns=3)
        self.selector = ChannelSelector(channels, self.dashboard.set_active)

        # ---- Columna izquierda: selector de canales | G-G/Modelo/Salud ----
        self.left_info = LeftInfoPanel(channels)
        left_tabs = QtWidgets.QTabWidget()
        left_tabs.addTab(self.selector, "Canales")
        left_tabs.addTab(self.left_info, "G-G / Modelo / Salud")
        left_tabs.setFixedWidth(360)

        # ---- Zona central: Dashboard principal | Pacejka Lab ----
        self.pacejka_lab = PacejkaLabPanel(columns=2)
        center_tabs = QtWidgets.QTabWidget()
        center_tabs.addTab(self.dashboard, "Dashboard Principal")
        center_tabs.addTab(self.pacejka_lab, "Pacejka Lab")

        splitter = QtWidgets.QSplitter()
        splitter.addWidget(left_tabs)
        splitter.addWidget(center_tabs)
        splitter.setStretchFactor(1, 1)

        central = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(central)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        cl.addWidget(top)
        cl.addWidget(splitter, 1)
        self.setCentralWidget(central)

        self.selector._apply_preset("AL-QP Core")  # preset inicial sensato

        self._logging = False
        self._log_file = None
        self._log_writer = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / UI_REFRESH_HZ))

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.hub.t0
        self.clock_lbl.setText(f"t+{elapsed:6.1f}s")

        alerts = self.dashboard.refresh(self.hub)
        self.left_info.update_data(self.hub)
        self.pacejka_lab.update_data(self.hub)
        if alerts["crit"]:
            self.alert_lbl.setText("\u26A0 CRÍTICO: " + ", ".join(alerts["crit"]))
            self.alert_lbl.setStyleSheet("color:#FF3333; font-weight:bold;")
        elif alerts["warn"]:
            self.alert_lbl.setText("\u26A0 AVISO: " + ", ".join(alerts["warn"]))
            self.alert_lbl.setStyleSheet("color:#FFEA00; font-weight:bold;")
        else:
            self.alert_lbl.setText("SISTEMAS NOMINALES")
            self.alert_lbl.setStyleSheet("color:#00FF00; font-weight:bold;")

        if self._logging:
            self._write_log_row(elapsed)

    def _toggle_logging(self, checked: bool) -> None:
        if checked:
            log_dir = os.path.join("output", "logs")
            os.makedirs(log_dir, exist_ok=True) # Crea la carpeta mágicamente si no existe
            fname = os.path.join(log_dir, time.strftime("telemetry_%Y%m%d_%H%M%S.csv"))            
            self._log_file = open(fname, "w", newline="")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(["t"] + list(self.channels_by_key.keys()))
            self._logging = True
            self.record_btn.setText("\u25A0 PARAR LOG")
            self.record_btn.setStyleSheet("background-color:#FF3333;")
        else:
            self._logging = False
            self.record_btn.setText("\u25CF INICIAR LOG")
            self.record_btn.setStyleSheet("")
            if self._log_file:
                self._log_file.close()
                self._log_file = None

    def _write_log_row(self, t: float) -> None:
        row = [f"{t:.3f}"] + [self.hub.latest.get(k, "") for k in self.channels_by_key.keys()]
        self._log_writer.writerow(row)

    def closeEvent(self, event) -> None:
        if self._log_file:
            self._log_file.close()
        event.accept()


# =====================================================================
# MAIN
# =====================================================================
def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_QSS)

    pg.setConfigOption("background", "#0a0a0a")
    pg.setConfigOption("foreground", "#d1d1d1")
    pg.setConfigOptions(antialias=False)

    hub = DataHub(CHANNELS)
    worker = CanWorker(hub) if USE_REAL_CAN else SimWorker(hub)
    worker.start()

    win = MainWindow(CHANNELS, hub)
    win.show()

    ret = app.exec_()
    worker.stop()
    sys.exit(ret)


if __name__ == "__main__":
    main()


# =====================================================================
# BONUS — parche mínimo de firmware para recuperar qp_residual "de verdad"
# =====================================================================
# Si quieres qp_residual real en pista (no solo en el simulador), añade una
# 4ª trama 0x103 en gp_interface.c dentro de gp_pack_telemetry():
#
#   void gp_pack_telemetry(..., uint8_t can_diag[8]) {
#       ...
#       float qp_res = 0.0f;
#       gp_qp_solve_rwd(..., &qp_res);           // ya te devuelve el residuo
#       uint16_t qp_pack = (uint16_t)(qp_res * 1000.0f);
#       can_diag[0] = (qp_pack >> 8) & 0xFF; can_diag[1] = qp_pack & 0xFF;
#       int16_t alpha_pack = (int16_t)(state->alpha_qp * 1e6f);
#       can_diag[2] = (alpha_pack >> 8) & 0xFF; can_diag[3] = alpha_pack & 0xFF;
#   }
#
# y envíala con ID 0x103 desde gp_mode_intermediate(). En este dashboard
# solo haría falta añadir un `elif mid == 0x103:` en CanWorker._decode y
# un ChannelSpec "qp_residual" de vuelta al registro.