import sys
import numpy as np
import threading
import time
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QGridLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer

# Cambiar a True cuando estéis en el garaje con el USB-CAN enchufado
USE_REAL_CAN = False 

# --- CONFIGURACIÓN DE MEMORIA CIRCULAR (Últimos 500 puntos) ---
HISTORY_LEN = 500
data = {
    "time": np.linspace(-5, 0, HISTORY_LEN),
    "t_out_rl": np.zeros(HISTORY_LEN), "t_out_rr": np.zeros(HISTORY_LEN),
    "k_opt_rl": np.zeros(HISTORY_LEN), "k_filt_rl": np.zeros(HISTORY_LEN),
    "vy_est": np.zeros(HISTORY_LEN),   "qp_residual": np.zeros(HISTORY_LEN),
    "theta_rl": np.full(HISTORY_LEN, 25000.0), "theta_rr": np.full(HISTORY_LEN, 25000.0),
    "mu_rl": np.full(HISTORY_LEN, 1.0)
}

state_text = {"theta_rl": 0, "theta_rr": 0, "mu_rl": 1.0, "qp_res": 0.0}

def can_listener_thread():
    """ Hilo de adquisición de datos (aislado de la interfaz gráfica) """
    if USE_REAL_CAN:
        import can
        bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        while True:
            try:
                msg = bus.recv()
                if msg.arbitration_id == 0x100:
                    new_vy = int.from_bytes(msg.data[0:2], 'big', signed=True) / 100.0
                    new_qp = int.from_bytes(msg.data[2:4], 'big', signed=False) / 1000.0
                    new_kopt = int.from_bytes(msg.data[4:6], 'big', signed=False) / 10000.0
                    
                    data["vy_est"] = np.roll(data["vy_est"], -1); data["vy_est"][-1] = new_vy
                    data["qp_residual"] = np.roll(data["qp_residual"], -1); data["qp_residual"][-1] = new_qp
                    data["k_opt_rl"] = np.roll(data["k_opt_rl"], -1); data["k_opt_rl"][-1] = new_kopt * 100
                    state_text["qp_res"] = new_qp

                elif msg.arbitration_id == 0x101:
                    new_th_rl = int.from_bytes(msg.data[0:2], 'big', signed=True) * 10.0
                    new_th_rr = int.from_bytes(msg.data[2:4], 'big', signed=True) * 10.0
                    new_mu = int.from_bytes(msg.data[4:6], 'big', signed=False) / 1000.0
                    
                    data["theta_rl"] = np.roll(data["theta_rl"], -1); data["theta_rl"][-1] = new_th_rl
                    data["theta_rr"] = np.roll(data["theta_rr"], -1); data["theta_rr"][-1] = new_th_rr
                    data["mu_rl"] = np.roll(data["mu_rl"], -1); data["mu_rl"][-1] = new_mu
                    state_text["theta_rl"] = new_th_rl; state_text["mu_rl"] = new_mu

                elif msg.arbitration_id == 0x102:
                    new_trl = int.from_bytes(msg.data[0:2], 'big', signed=True) / 10.0
                    new_trr = int.from_bytes(msg.data[2:4], 'big', signed=True) / 10.0
                    new_kfilt = int.from_bytes(msg.data[4:6], 'big', signed=False) / 10000.0
                    
                    data["t_out_rl"] = np.roll(data["t_out_rl"], -1); data["t_out_rl"][-1] = new_trl
                    data["t_out_rr"] = np.roll(data["t_out_rr"], -1); data["t_out_rr"][-1] = new_trr
                    data["k_filt_rl"] = np.roll(data["k_filt_rl"], -1); data["k_filt_rl"][-1] = new_kfilt * 100
            except:
                pass
    else:
        # MODO SIMULADOR AVANZADO (Ondas dinámicas para visualizar la UI)
        t = 0
        while True:
            t += 0.05
            data["t_out_rl"] = np.roll(data["t_out_rl"], -1); data["t_out_rl"][-1] = 150 + 50 * np.sin(t*2)
            data["t_out_rr"] = np.roll(data["t_out_rr"], -1); data["t_out_rr"][-1] = 150 + 50 * np.sin(t*2 + 0.5)
            
            data["k_opt_rl"] = np.roll(data["k_opt_rl"], -1); data["k_opt_rl"][-1] = 12.0 + 2*np.sin(t*0.5)
            data["k_filt_rl"] = np.roll(data["k_filt_rl"], -1); data["k_filt_rl"][-1] = data["k_opt_rl"][-1] + np.random.normal(0, 0.8)
            
            data["vy_est"] = np.roll(data["vy_est"], -1); data["vy_est"][-1] = 0.5 * np.sin(t)
            
            data["qp_residual"] = np.roll(data["qp_residual"], -1)
            data["qp_residual"][-1] = np.abs(np.random.normal(0, 0.01)) if np.sin(t*0.2) > 0 else 0.0
            
            data["theta_rl"] = np.roll(data["theta_rl"], -1); data["theta_rl"][-1] = 20000 + 10000*np.sin(t*0.8)
            data["theta_rr"] = np.roll(data["theta_rr"], -1); data["theta_rr"][-1] = 20000 + 8000*np.cos(t*0.8)
            
            data["mu_rl"] = np.roll(data["mu_rl"], -1); data["mu_rl"][-1] = 1.2 + 0.1*np.sin(t*0.1)
            
            state_text["theta_rl"] = data["theta_rl"][-1]
            state_text["qp_res"] = data["qp_residual"][-1]
            state_text["mu_rl"] = data["mu_rl"][-1]
            time.sleep(0.05)


class TelemetryApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tecnun eRacing | VCU Advanced Telemetry (HIL)")
        self.resize(1400, 900)
        
        # Tema Oscuro Competición (Estilo MoTeC)
        pg.setConfigOption('background', '#0a0a0a')
        pg.setConfigOption('foreground', '#d1d1d1')
        
        main_widget = QWidget()
        layout = QVBoxLayout()
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)
        
        # --- PANEL SUPERIOR DE ESTADO (Diseño Premium) ---
        self.status_label = QLabel("INICIALIZANDO CONEXIÓN...")
        self.status_label.setStyleSheet("""
            background-color: #1a1a1a;
            color: #ffffff;
            font-size: 16px;
            font-family: monospace;
            padding: 12px;
            border: 1px solid #333333;
            border-radius: 4px;
        """)
        layout.addWidget(self.status_label)

        # --- GRID DE GRÁFICAS (2 Filas x 3 Columnas) ---
        grid = QGridLayout()
        layout.addLayout(grid)

        # Configuración común para las gráficas
        def create_plot(title, y_range):
            p = pg.PlotWidget(title=title)
            p.setYRange(*y_range)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.getAxis('bottom').setStyle(showValues=False) # Ocultar eje X
            return p

        # 1. Torque Comandado (Top Left)
        self.p1 = create_plot("1. TORQUE VECTORING (Nm) | RL(Cyan) RR(Mag)", (-50, 350))
        self.curve_trl = self.p1.plot(pen=pg.mkPen('#00E5FF', width=2))
        self.curve_trr = self.p1.plot(pen=pg.mkPen('#FF00FF', width=2))
        grid.addWidget(self.p1, 0, 0)

        # 2. Slip Ratio (Top Center)
        self.p2 = create_plot("2. CONTROL DE TRACCIÓN (%) | Tgt(Yel) Act(Wht)", (0, 25))
        self.curve_kopt = self.p2.plot(pen=pg.mkPen('#FFEA00', width=2, style=pg.QtCore.Qt.DashLine))
        self.curve_kfilt = self.p2.plot(pen=pg.mkPen('#FFFFFF', width=1.5))
        grid.addWidget(self.p2, 0, 1)

        # 3. Superficie / Fricción (Top Right)
        self.p3 = create_plot("3. FRICCIÓN ESTIMADA (Mu) | Grip Level", (0.0, 2.0))
        self.curve_mu = self.p3.plot(pen=pg.mkPen('#00FF00', width=2))
        grid.addWidget(self.p3, 0, 2)

        # 4. RLS Pacejka Theta (Bottom Left)
        self.p4 = create_plot("4. RLS PACEJKA THETA (Derivada Fx/Kappa)", (-10000, 45000))
        self.curve_th_rl = self.p4.plot(pen=pg.mkPen('#00E5FF', width=2))
        self.curve_th_rr = self.p4.plot(pen=pg.mkPen('#FF00FF', width=2))
        grid.addWidget(self.p4, 1, 0)

        # 5. Dinámica Lateral (Bottom Center)
        self.p5 = create_plot("5. OBSERVADOR LATERAL (v_y est) [m/s]", (-3, 3))
        self.curve_vy = self.p5.plot(pen=pg.mkPen('#FF3333', width=2))
        grid.addWidget(self.p5, 1, 1)

        # 6. Residuo Solver KKT (Bottom Right)
        self.p6 = create_plot("6. SOLVER KKT RESIDUAL (Salud Computacional)", (-0.01, 0.15))
        self.curve_qp = self.p6.plot(pen=pg.mkPen('#FFAA00', width=2))
        grid.addWidget(self.p6, 1, 2)

        # Bucle de actualización (60 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(16)

    def update_gui(self):
        # Actualizar Datos de las Gráficas
        t = data["time"]
        self.curve_trl.setData(t, data["t_out_rl"])
        self.curve_trr.setData(t, data["t_out_rr"])
        self.curve_kopt.setData(t, data["k_opt_rl"])
        self.curve_kfilt.setData(t, data["k_filt_rl"])
        self.curve_mu.setData(t, data["mu_rl"])
        self.curve_th_rl.setData(t, data["theta_rl"])
        self.curve_th_rr.setData(t, data["theta_rr"])
        self.curve_vy.setData(t, data["vy_est"])
        self.curve_qp.setData(t, data["qp_residual"])
        
        # Formatear el Panel Superior
        th_rl = state_text["theta_rl"]
        qp_res = state_text["qp_res"]
        mu = state_text["mu_rl"]
        
        # Lógica de Colores y Alertas para el Texto
        alert_qp = "<span style='color: #FF3333;'> ¡! SATURADO (Revisar Fricción)</span>" if qp_res > 0.05 else "<span style='color: #00FF00;'> ESTABLE</span>"
        alert_th = "<span style='color: #FFEA00;'>⚠️ LÍMITE ALCANZADO (Pico Pacejka)</span>" if th_rl < 2000 else "<span style='color: #00E5FF;'>⚙️ BUSCANDO AGARRE...</span>"
        
        html_text = f"""
        <table width='100%'>
            <tr>
                <td width='33%'><b>ESTADO SOLVER:</b> {alert_qp} <span style='color:#888;'>(Res: {qp_res:.4f})</span></td>
                <td width='33%'><b>ESTADO TRACCIÓN:</b> {alert_th} <span style='color:#888;'>(θ: {th_rl:,.0f})</span></td>
                <td width='33%'><b>AGARRE PISTA:</b> <span style='color: #00FF00;'>Mu {mu:.2f}</span></td>
            </tr>
        </table>
        """
        self.status_label.setText(html_text)


if __name__ == '__main__':
    # Arrancar hilo CAN
    threading.Thread(target=can_listener_thread, daemon=True).start()
    # Arrancar GUI
    app = QApplication(sys.argv)
    window = TelemetryApp()
    window.show()
    sys.exit(app.exec_())