import time
import random
import threading
from rich.live import Live
from rich.table import Table
from rich.layout import Layout
from rich.panel import Panel
from rich.console import Console

# Si tienes un adaptador real, cambia esto a True (necesitas python-can)
USE_REAL_CAN = False 

# Variables globales para compartir entre el hilo de lectura y la interfaz
telemetry = {
    "vy_est": 0.0,
    "qp_residual": 0.0,
    "k_opt_rl": 0.0, "k_opt_rr": 0.0,
    "theta_rl": 0.0, "theta_rr": 0.0,
    "mu_rl": 0.0, "mu_rr": 0.0
}

def can_listener_thread():
    """ Hilo en background que lee el USB-CAN y actualiza el diccionario """
    if USE_REAL_CAN:
        import can
        # Ajusta "pcan", "kvaser" o "slcan" según el hardware que uséis
        bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        while True:
            msg = bus.recv()
            if msg.arbitration_id == 0x100:
                telemetry["vy_est"] = int.from_bytes(msg.data[0:2], byteorder='big', signed=True) / 100.0
                telemetry["qp_residual"] = int.from_bytes(msg.data[2:4], byteorder='big', signed=False) / 1000.0
                telemetry["k_opt_rl"] = int.from_bytes(msg.data[4:6], byteorder='big', signed=False) / 10000.0
                telemetry["k_opt_rr"] = int.from_bytes(msg.data[6:8], byteorder='big', signed=False) / 10000.0
            elif msg.arbitration_id == 0x101:
                telemetry["theta_rl"] = int.from_bytes(msg.data[0:2], byteorder='big', signed=True) * 10.0
                telemetry["theta_rr"] = int.from_bytes(msg.data[2:4], byteorder='big', signed=True) * 10.0
                telemetry["mu_rl"] = int.from_bytes(msg.data[4:6], byteorder='big', signed=False) / 1000.0
                telemetry["mu_rr"] = int.from_bytes(msg.data[6:8], byteorder='big', signed=False) / 1000.0
    else:
        # MODO SIMULACIÓN PARA TESTEAR EN CASA
        while True:
            telemetry["vy_est"] = random.uniform(-0.5, 0.5)
            telemetry["qp_residual"] = random.uniform(0.0, 0.05)
            telemetry["k_opt_rl"] = random.uniform(0.10, 0.12)
            telemetry["k_opt_rr"] = random.uniform(0.10, 0.12)
            telemetry["theta_rl"] = random.uniform(12000, 25000)
            telemetry["theta_rr"] = random.uniform(12000, 25000)
            telemetry["mu_rl"] = 1.2
            telemetry["mu_rr"] = 1.2
            time.sleep(0.05) # 20 Hz refresh

def generate_dashboard() -> Layout:
    """ Genera la UI de la terminal basándose en los datos en vivo """
    
    # 1. Panel de Dinámica (KKT y Observador Lateral)
    table_dyn = Table(expand=True, show_header=False, border_style="cyan")
    table_dyn.add_column("Sensor", style="bold cyan")
    table_dyn.add_column("Value", justify="right")
    table_dyn.add_column("Status", justify="center")
    
    vy = telemetry["vy_est"]
    vy_status = "🟢 OK" if abs(vy) < 1.0 else "⚠️ DERRAPE"
    table_dyn.add_row("v_y Est (Deriva)", f"{vy:+.2f} m/s", vy_status)
    
    qp = telemetry["qp_residual"]
    qp_status = "🟢 OK" if qp < 0.1 else "🔴 SATURED"
    table_dyn.add_row("Solver KKT Residual", f"{qp:.4f}", qp_status)

    # 2. Panel de Tracción (RLS Pacejka)
    table_tc = Table(expand=True, show_header=False, border_style="magenta")
    table_tc.add_column("Param", style="bold magenta")
    table_tc.add_column("Rear Left", justify="right")
    table_tc.add_column("Rear Right", justify="right")
    
    th_rl, th_rr = telemetry["theta_rl"], telemetry["theta_rr"]
    k_rl, k_rr = telemetry["k_opt_rl"] * 100, telemetry["k_opt_rr"] * 100
    mu_rl, mu_rr = telemetry["mu_rl"], telemetry["mu_rr"]
    
    table_tc.add_row("Pacejka Slope (Theta)", f"{th_rl:,.0f}", f"{th_rr:,.0f}")
    table_tc.add_row("Target Slip (k_opt)", f"{k_rl:.1f} %", f"{k_rr:.1f} %")
    table_tc.add_row("Est. Grip (Mu)", f"{mu_rl:.2f}", f"{mu_rr:.2f}")

    # Layout Principal
    layout = Layout()
    layout.split_column(
        Layout(Panel("[bold white]Tecnun eRacing - VCU Live Diagnostics (HIL Mode)[/]", style="on blue"), size=3),
        Layout(name="body")
    )
    layout["body"].split_row(
        Layout(Panel(table_dyn, title="[CHASIS Y SOLVER]")),
        Layout(Panel(table_tc, title="[TRACCIÓN Y PACEJKA]"))
    )
    
    return layout

# Iniciar el hilo de lectura CAN
thread = threading.Thread(target=can_listener_thread, daemon=True)
thread.start()

# Iniciar la interfaz Rich
console = Console()
console.clear()
try:
    with Live(generate_dashboard(), refresh_per_second=15, screen=True) as live:
        while True:
            time.sleep(0.1)
            live.update(generate_dashboard())
except KeyboardInterrupt:
    console.print("\n[bold red]Telemetría desconectada.[/]")