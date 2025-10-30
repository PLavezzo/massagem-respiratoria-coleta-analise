import sys
import serial
import time
import os
import collections
import struct
import pandas as pd
from datetime import datetime
import json

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.dockarea import DockArea, Dock

from serial.tools import list_ports
import numpy as np
from scipy.signal import find_peaks

# ------------------- CONFIG --------------------
SERIAL_PORT = '/dev/cu.usbserial-130'  
BAUD_RATE = 250000
UPDATE_INTERVAL_MS = 10
MAX_HISTORY_SECONDS_VIEW = 13
# ------------------------------------------------

# ---------- Protocolo ----------
START_MARKER = b'\xFF'
NUM_DATA_BYTES = 8
PACKET_SIZE = 1 + NUM_DATA_BYTES
# --------------------------------

# --- Variáveis de estado ---
ser = None
connected = False
is_collecting = False
is_warming_up = False
countdown_seconds = 5

# Dicionário configuração da detecção de picos ###
peak_detection_config = {'source': 'X'} 

# --- Calibração (2 pontos) ---
CALIBRATION_CACHE_FILE = os.path.join(os.path.dirname(__file__), '.cache_calibracao.json')
calibration_data = {
    'X': {'offset': -7.0, 'gain': 1.10, 'calibrated': False},
    'Y': {'offset': -68.0, 'gain': 1.12, 'calibrated': False}
}

def load_calibration_cache():
    """Carrega calibração do arquivo cache."""
    global calibration_data
    if os.path.exists(CALIBRATION_CACHE_FILE):
        try:
            with open(CALIBRATION_CACHE_FILE, 'r') as f:
                loaded = json.load(f)
                calibration_data.update(loaded)
                print(f"Calibração carregada do cache: {CALIBRATION_CACHE_FILE}")
        except Exception as e:
            print(f"Erro ao carregar cache de calibração: {e}")

def save_calibration_cache():
    """Salva calibração no arquivo cache."""
    try:
        with open(CALIBRATION_CACHE_FILE, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        print(f"Calibração salva no cache: {CALIBRATION_CACHE_FILE}")
    except Exception as e:
        print(f"Erro ao salvar cache de calibração: {e}")

# Carrega calibração ao iniciar
load_calibration_cache() 


# --- Estruturas de dados ---
time_data = collections.deque()
x_data = collections.deque()
y_data = collections.deque()
px_data = collections.deque()
py_data = collections.deque()
recent_ts = collections.deque(maxlen=300)
current_frequency = 0.0
serial_buffer = b''
t0_us = None
WINDOW_SEC = 2.0
recent_t = collections.deque()
recent_px = collections.deque() # buffer preenchido com X ou Y
peak_freq_hz = 0.0
MIN_DIST_S = 0.05
WIDTH_MIN_S = 0.01
PROM_FACTOR = 1.50
WLEN_S = 2.0
DETECT_VALLEYS = False

# --- Buffers para calibração (sempre atualizados quando conectado) ---
calib_x_buffer = collections.deque(maxlen=50)  # últimas 50 leituras de X
calib_y_buffer = collections.deque(maxlen=50)  # últimas 50 leituras de Y

# ---------- UI ----------
app = QtWidgets.QApplication(sys.argv)

class CustomMainWindow(QtWidgets.QMainWindow):
    def closeEvent(self, event):
        global ser
        print('\nAplicação encerrada.')
        if ser is not None and ser.isOpen():
            try:
                ser.close()
                print('Porta serial desconectada.')
            except Exception as e:
                print(f"Erro ao fechar a porta serial: {e}")
        event.accept()

area = DockArea()
main_window = CustomMainWindow()
main_window.setWindowTitle('Coleta de Dados Fisioterápicos (desconectado)')
main_window.setCentralWidget(area)
main_window.resize(1200, 650)
main_window.show()

dock_plot = Dock("Gráfico", size=(800, 1))
area.addDock(dock_plot, 'left')
dock_ctrl = Dock("Controle e Métricas", size=(300, 1))
area.addDock(dock_ctrl, 'right')

plot_widget = pg.PlotWidget(title='Valores dos Sensores vs Tempo')
plot_widget.setLabel('left', 'Pressão (mmHg)')
plot_widget.setLabel('bottom', 'Tempo (s)')
plot_widget.showGrid(x=True, y=True)
plot_widget.addLegend()
curve_x = plot_widget.plot(pen='r', name='Direito (X)')
curve_y = plot_widget.plot(pen='b', name='Esquerdo (Y)')
peaks_scatter = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(255, 0, 255, 160), pen=pg.mkPen(20, 20, 20, 180, width=0.8))
plot_widget.addItem(peaks_scatter)
dock_plot.addWidget(plot_widget)

ctrl_container = QtWidgets.QWidget()
dock_ctrl.addWidget(ctrl_container)
ctrl_layout = QtWidgets.QVBoxLayout(ctrl_container)
ctrl_layout.setContentsMargins(8, 8, 8, 8)
ctrl_layout.setSpacing(8)

port_row = QtWidgets.QHBoxLayout()
port_label = QtWidgets.QLabel("<b>Porta serial:</b>")
combo_ports = QtWidgets.QComboBox()
btn_refresh = QtWidgets.QPushButton("Atualizar")
port_row.addWidget(port_label); port_row.addWidget(combo_ports, 1); port_row.addWidget(btn_refresh)
ctrl_layout.addLayout(port_row)

ctrl_row = QtWidgets.QHBoxLayout()
btn_iniciar = QtWidgets.QPushButton("Iniciar")
btn_pausar = QtWidgets.QPushButton("Pausar")
btn_pausar.setEnabled(False)
ctrl_row.addWidget(btn_iniciar); ctrl_row.addWidget(btn_pausar)
ctrl_layout.addLayout(ctrl_row)

action_row = QtWidgets.QHBoxLayout()
btn_salvar = QtWidgets.QPushButton("Salvar Coleta")
btn_resetar = QtWidgets.QPushButton("Resetar Coleta")
btn_salvar.setEnabled(False); btn_resetar.setEnabled(False)
action_row.addWidget(btn_salvar); action_row.addWidget(btn_resetar)
ctrl_layout.addLayout(action_row)

# Botão de Calibração
btn_calibracao = QtWidgets.QPushButton("Calibração")
btn_calibracao.setStyleSheet("background-color: #FFA500; font-weight: bold;")
ctrl_layout.addWidget(btn_calibracao)

lbl_tempo_coleta = QtWidgets.QLabel("Tempo de Coleta: 0.00 s")
font_tempo = lbl_tempo_coleta.font()
font_tempo.setBold(True)
lbl_tempo_coleta.setFont(font_tempo)
ctrl_layout.addWidget(lbl_tempo_coleta)


# --- PB(1ª ordem RC) ---
LPF_ON = True          # ativa/desativa filtro
LPF_FC = 60.0          # frequência de corte (Hz)
_lpf_state = {"x": None, "y": None}

def _lpf_step(val, dt, fc, key):
    """Filtro RC: y += alpha*(x - y), alpha = dt / (RC + dt), RC = 1/(2πfc)."""
    if not np.isfinite(dt) or dt <= 0:
        dt = 1.0 / max(current_frequency, 200.0)  # fallback (≈200 Hz)
    RC = 1.0 / (2.0 * np.pi * fc)
    alpha = dt / (RC + dt)
    y_prev = _lpf_state[key]
    y = val if y_prev is None else (y_prev + alpha * (val - y_prev))
    _lpf_state[key] = y
    return y





def populate_ports():
    combo_ports.clear()
    ports = list_ports.comports()
    for p in ports:
        text = f"{p.device} — {p.description}"
        combo_ports.addItem(text, userData=p.device)
    if combo_ports.count() == 0:
        combo_ports.addItem(f"{SERIAL_PORT} (manual)", userData=SERIAL_PORT)

def on_refresh_ports():
    populate_ports()

def end_warmup():
    global is_warming_up, t0_us
    countdown_timer.stop()
    if not is_collecting: return
    is_warming_up = False
    t0_us = None
    time_data.clear(); x_data.clear(); y_data.clear(); px_data.clear(); py_data.clear()
    print("Aquecimento concluído. Coleta real iniciada.")
    btn_iniciar.setText("Iniciar")
    btn_pausar.setEnabled(True)
    btn_salvar.setEnabled(False); btn_resetar.setEnabled(False)
    lbl_tempo_coleta.setText("Tempo de Coleta: 0.00 s")

def update_countdown():
    global countdown_seconds
    if not is_warming_up:
        countdown_timer.stop()
        return
        
    countdown_seconds -= 1
    if countdown_seconds > 0:
        btn_iniciar.setText(f"Aquecendo... {countdown_seconds}s")
    else:
        btn_iniciar.setText("Iniciando...")
        countdown_timer.stop()
        end_warmup()

def on_iniciar():
    global ser, connected, is_collecting, is_warming_up, countdown_seconds

    if not connected:
        device = combo_ports.currentData()
        if not device: device = SERIAL_PORT
        try:
            ser = serial.Serial(device, BAUD_RATE, timeout=0.05)
            ser.flushInput()
            connected = True
            print(f'Conectado à porta serial {device} a {BAUD_RATE} baud.')
            combo_ports.setEnabled(False)
            btn_refresh.setEnabled(False)
            if not timer.isActive(): timer.start(UPDATE_INTERVAL_MS)
            if not peaks_timer.isActive(): peaks_timer.start(2000)
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Erro de conexão", f"Não foi possível abrir {device}:\n{e}")
            return

    is_collecting = True
    is_warming_up = True
    print("Iniciando período de aquecimento de 5 segundos...")
    
    countdown_seconds = 5
    btn_iniciar.setEnabled(False)
    btn_iniciar.setText(f"Aquecendo... {countdown_seconds}s")
    btn_pausar.setEnabled(False)
    
    countdown_timer.start(1000)

def on_pausar():
    global is_collecting, is_warming_up
    is_collecting = False
    is_warming_up = False
    countdown_timer.stop()
    print("Coleta pausada.")
    btn_iniciar.setText("Iniciar")
    btn_iniciar.setEnabled(True)
    btn_pausar.setEnabled(False)

def on_salvar():
    if not time_data:
        QtWidgets.QMessageBox.information(None, "Salvar", "Nenhum dado coletado para salvar.")
        return
    try:
        df = pd.DataFrame({'Tempo (s)': list(time_data), 'Valor X': list(x_data), 'Valor Y': list(y_data),
                           'Pressão X (mmHg)': list(px_data), 'Pressão Y (mmHg)': list(py_data)})
        now = datetime.now()
        default_name = f"coleta_{now.strftime('%Y%m%d_%H%M%S')}.csv"
        save_path, _ = QtWidgets.QFileDialog.getSaveFileName(main_window, "Salvar dados", default_name, "CSV (*.csv)")
        if save_path:
            df.to_csv(save_path, index=False, sep=';', float_format='%.10f')
            print(f'Dados salvos em {save_path}')
            QtWidgets.QMessageBox.information(None, "Sucesso", f"Dados salvos com sucesso em:\n{save_path}")
    except Exception as e:
        QtWidgets.QMessageBox.critical(None, "Erro ao Salvar", f"Não foi possível salvar o arquivo CSV:\n{e}")

def on_resetar():
    global t0_us
    if not time_data and not is_warming_up:
        return
    reply = QtWidgets.QMessageBox.question(main_window, 'Resetar Coleta',
                                           "Tem certeza que deseja apagar os dados da coleta atual?",
                                           QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
    if reply == QtWidgets.QMessageBox.Yes:
        print("Resetando a coleta.")
        on_pausar() 
        time_data.clear(); x_data.clear(); y_data.clear(); px_data.clear(); py_data.clear()
        recent_t.clear(); recent_px.clear(); recent_ts.clear()
        t0_us = None
        curve_x.setData([], []); curve_y.setData([], []); peaks_scatter.setData([], [])
        plot_widget.setXRange(0, MAX_HISTORY_SECONDS_VIEW)
        btn_salvar.setEnabled(False); btn_resetar.setEnabled(False)
        lbl_tempo_coleta.setText("Tempo de Coleta: 0.00 s")

btn_refresh.clicked.connect(on_refresh_ports)
btn_iniciar.clicked.connect(on_iniciar)
btn_pausar.clicked.connect(on_pausar)
btn_salvar.clicked.connect(on_salvar)
btn_resetar.clicked.connect(on_resetar)

def open_calibration_dialog():
    """Abre janela de calibração interativa (método 2 pontos)."""
    global ser, connected
    
    # Se não estiver conectado, conecta automaticamente
    if not connected:
        device = combo_ports.currentData()
        if not device:
            device = SERIAL_PORT
        try:
            ser = serial.Serial(device, BAUD_RATE, timeout=0.05)
            ser.flushInput()
            connected = True
            print(f'Conectado à porta serial {device} a {BAUD_RATE} baud (via calibração).')
            combo_ports.setEnabled(False)
            btn_refresh.setEnabled(False)
            if not timer.isActive():
                timer.start(UPDATE_INTERVAL_MS)
        except Exception as e:
            QtWidgets.QMessageBox.critical(main_window, "Erro de conexão", 
                f"Não foi possível abrir {device}:\n{e}\n\nSelecione a porta correta e tente novamente.")
            return
    
    dialog = QtWidgets.QDialog(main_window)
    dialog.setWindowTitle("Calibração - Método 2 Pontos")
    dialog.resize(450, 500)
    
    layout = QtWidgets.QVBoxLayout(dialog)
    
    # Estado local da calibração
    calib_state = {
        'channel': 'X',
        'point1': {'adc_avg': None, 'manometer': None},
        'point2': {'adc_avg': None, 'manometer': None}
    }
    
    # Informações
    info = QtWidgets.QLabel(
        "<b>PROTOCOLO DE CALIBRAÇÃO</b><br>"
        "1. Escolha o canal (Direito ou Esquerdo)<br>"
        "2. Aplique pressão BAIXA (~30 mmHg)<br>"
        "3. Anote o manômetro e clique 'Capturar Ponto 1'<br>"
        "4. Aplique pressão ALTA (~60 mmHg)<br>"
        "5. Anote o manômetro e clique 'Capturar Ponto 2'<br>"
        "6. Os fatores serão calculados e salvos automaticamente"
    )
    info.setWordWrap(True)
    layout.addWidget(info)
    
    # Seleção de canal
    layout.addWidget(QtWidgets.QLabel("<b>Canal a calibrar:</b>"))
    combo_channel = QtWidgets.QComboBox()
    combo_channel.addItem("Direito (X - vermelho)", userData='X')
    combo_channel.addItem("Esquerdo (Y - azul)", userData='Y')
    layout.addWidget(combo_channel)
    
    def on_channel_change():
        calib_state['channel'] = combo_channel.currentData()
    combo_channel.currentIndexChanged.connect(on_channel_change)
    
    layout.addSpacing(10)
    
    # Leitura atual
    lbl_current = QtWidgets.QLabel("Leitura ADC atual: —")
    lbl_current.setStyleSheet("font-size: 11pt; font-weight: bold;")
    layout.addWidget(lbl_current)
    
    layout.addSpacing(10)
    
    # Ponto 1
    group_p1 = QtWidgets.QGroupBox("Ponto 1 (baixa ~30 mmHg)")
    layout_p1 = QtWidgets.QFormLayout()
    spin_p1 = QtWidgets.QDoubleSpinBox()
    spin_p1.setRange(0, 200)
    spin_p1.setValue(30.0)
    spin_p1.setDecimals(1)
    layout_p1.addRow("Valor manômetro (mmHg):", spin_p1)
    btn_capture_p1 = QtWidgets.QPushButton("Capturar Ponto 1")
    layout_p1.addRow(btn_capture_p1)
    lbl_p1_status = QtWidgets.QLabel("Status: Não capturado")
    lbl_p1_status.setStyleSheet("color: gray;")
    layout_p1.addRow(lbl_p1_status)
    group_p1.setLayout(layout_p1)
    layout.addWidget(group_p1)
    
    # Ponto 2
    group_p2 = QtWidgets.QGroupBox("Ponto 2 (alta ~60 mmHg)")
    layout_p2 = QtWidgets.QFormLayout()
    spin_p2 = QtWidgets.QDoubleSpinBox()
    spin_p2.setRange(0, 200)
    spin_p2.setValue(60.0)
    spin_p2.setDecimals(1)
    layout_p2.addRow("Valor manômetro (mmHg):", spin_p2)
    btn_capture_p2 = QtWidgets.QPushButton("Capturar Ponto 2")
    layout_p2.addRow(btn_capture_p2)
    lbl_p2_status = QtWidgets.QLabel("Status: Não capturado")
    lbl_p2_status.setStyleSheet("color: gray;")
    layout_p2.addRow(lbl_p2_status)
    group_p2.setLayout(layout_p2)
    layout.addWidget(group_p2)
    
    # Resultados
    lbl_results = QtWidgets.QLabel("Fatores calculados: —")
    lbl_results.setStyleSheet("font-size: 10pt; color: green; font-weight: bold;")
    layout.addWidget(lbl_results)
    
    # Timer para atualizar leitura atual
    def update_current_reading():
        if not connected or len(calib_x_buffer) == 0 or len(calib_y_buffer) == 0:
            return
        ch = calib_state['channel']
        # Pega apenas o ÚLTIMO valor (mais recente) dos buffers de calibração
        current_adc = calib_x_buffer[-1] if ch == 'X' else calib_y_buffer[-1]
        lbl_current.setText(f"Leitura ADC atual ({ch}): {current_adc:.0f} counts")
    
    timer_reading = QtCore.QTimer()
    timer_reading.timeout.connect(update_current_reading)
    timer_reading.start(200)
    
    def capture_point_1():
        ch = calib_state['channel']
        
        # Usa os buffers de calibração (sempre atualizados)
        if ch == 'X':
            if len(calib_x_buffer) < 10:
                QtWidgets.QMessageBox.warning(dialog, "Aviso", "Aguarde mais leituras! Conecte ao ESP32 primeiro.")
                return
            recent_adc = list(calib_x_buffer)
        else:
            if len(calib_y_buffer) < 10:
                QtWidgets.QMessageBox.warning(dialog, "Aviso", "Aguarde mais leituras! Conecte ao ESP32 primeiro.")
                return
            recent_adc = list(calib_y_buffer)
        
        # Média das últimas 30 leituras
        avg = np.mean(recent_adc[-30:])
        calib_state['point1']['adc_avg'] = avg
        calib_state['point1']['manometer'] = spin_p1.value()
        
        lbl_p1_status.setText(f"✓ ADC={avg:.1f} → {spin_p1.value():.1f} mmHg")
        lbl_p1_status.setStyleSheet("color: green;")
        
        print(f"[DEBUG] Ponto 1 capturado: ADC={avg:.1f}, Manômetro={spin_p1.value():.1f} mmHg")
        
        if calib_state['point2']['adc_avg'] is not None:
            calculate_and_save()
    
    def capture_point_2():
        ch = calib_state['channel']
        
        # Usa os buffers de calibração (sempre atualizados)
        if ch == 'X':
            if len(calib_x_buffer) < 10:
                QtWidgets.QMessageBox.warning(dialog, "Aviso", "Aguarde mais leituras! Conecte ao ESP32 primeiro.")
                return
            recent_adc = list(calib_x_buffer)
        else:
            if len(calib_y_buffer) < 10:
                QtWidgets.QMessageBox.warning(dialog, "Aviso", "Aguarde mais leituras! Conecte ao ESP32 primeiro.")
                return
            recent_adc = list(calib_y_buffer)
        
        # Média das últimas 30 leituras
        avg = np.mean(recent_adc[-30:])
        calib_state['point2']['adc_avg'] = avg
        calib_state['point2']['manometer'] = spin_p2.value()
        
        lbl_p2_status.setText(f"✓ ADC={avg:.1f} → {spin_p2.value():.1f} mmHg")
        lbl_p2_status.setStyleSheet("color: green;")
        
        print(f"[DEBUG] Ponto 2 capturado: ADC={avg:.1f}, Manômetro={spin_p2.value():.1f} mmHg")
        
        if calib_state['point1']['adc_avg'] is not None:
            calculate_and_save()
    
    def calculate_and_save():
        p1_adc = calib_state['point1']['adc_avg']
        p2_adc = calib_state['point2']['adc_avg']
        p1_real = calib_state['point1']['manometer']
        p2_real = calib_state['point2']['manometer']
        
        if p1_adc is None or p2_adc is None:
            return
        
        # Converte ADC para pressão teórica
        _beta = 18000.0 / (10000.0 + 18000.0)
        _ADC_FS = 3.3
        _VS = 5.0
        _ADCMAX = 4095.0
        _K = 416.7
        _scale = _ADC_FS / (_ADCMAX * _beta * _VS)
        
        p1_bruto = _K * ((p1_adc * _scale) - 0.04)
        p2_bruto = _K * ((p2_adc * _scale) - 0.04)
        
        if abs(p2_bruto - p1_bruto) < 0.1:
            QtWidgets.QMessageBox.critical(dialog, "Erro", "Os dois pontos são muito próximos!")
            return
        
        # Calcula ganho e offset
        # P_real = offset + gain × P_bruto
        gain = (p2_real - p1_real) / (p2_bruto - p1_bruto)
        offset = p1_real - gain * p1_bruto
        
        # Salva no dicionário global
        ch = calib_state['channel']
        calibration_data[ch]['offset'] = offset
        calibration_data[ch]['gain'] = gain
        calibration_data[ch]['calibrated'] = True
        
        save_calibration_cache()
        
        lbl_results.setText(f"✓ Offset: {offset:.2f} mmHg | Ganho: {gain:.4f}")
        
        QtWidgets.QMessageBox.information(
            dialog,
            "Calibração Concluída!",
            f"Canal {ch} ({'DIREITO' if ch == 'X' else 'ESQUERDO'}) calibrado com sucesso!\n\n"
            f"Offset: {offset:.2f} mmHg\n"
            f"Ganho: {gain:.4f}\n\n"
            f"Fórmula: P = {offset:.2f} + {gain:.4f} × P_bruta\n\n"
            f"Os valores foram salvos em cache."
        )
    
    btn_capture_p1.clicked.connect(capture_point_1)
    btn_capture_p2.clicked.connect(capture_point_2)
    
    # Botão fechar
    btn_close = QtWidgets.QPushButton("Fechar")
    btn_close.clicked.connect(dialog.close)
    layout.addWidget(btn_close)
    
    def cleanup():
        timer_reading.stop()
    
    dialog.finished.connect(cleanup)
    dialog.exec_()

btn_calibracao.clicked.connect(open_calibration_dialog)

populate_ports()

line_sep = QtWidgets.QFrame(); line_sep.setFrameShape(QtWidgets.QFrame.HLine)
ctrl_layout.addWidget(line_sep)
lab_title = QtWidgets.QLabel("<b>Detecção de Picos (janela ~2 s)</b>"); ctrl_layout.addWidget(lab_title)

# dropdown para selecionar o sinal
peak_source_layout = QtWidgets.QHBoxLayout()
peak_source_label = QtWidgets.QLabel("Analisar sinal:")
combo_peak_source = QtWidgets.QComboBox()
combo_peak_source.addItem("Direito (X)", userData='X')
combo_peak_source.addItem("Esquerdo (Y)", userData='Y')
peak_source_layout.addWidget(peak_source_label)
peak_source_layout.addStretch(1)
peak_source_layout.addWidget(combo_peak_source)
ctrl_layout.addLayout(peak_source_layout)


def make_spin(label, val, vmin, vmax, step, decimals=3):
    lab = QtWidgets.QLabel(label); spn = QtWidgets.QDoubleSpinBox(); spn.setDecimals(decimals); spn.setRange(vmin, vmax)
    spn.setSingleStep(step); spn.setValue(val); row = QtWidgets.QHBoxLayout(); row.addWidget(lab)
    row.addStretch(1); row.addWidget(spn); ctrl_layout.addLayout(row)
    return spn
spin_min_dist = make_spin("Distância mínima (s):", MIN_DIST_S, 0.005, 1.0, 0.005, 3)
spin_width    = make_spin("Largura mínima (s):",  WIDTH_MIN_S, 0.001, 0.3, 0.005, 3)
spin_prom     = make_spin("Prominência ×σ:",      PROM_FACTOR, 0.10,  2.0,  0.05, 2)
spin_wlen     = make_spin("Janela (s):",          WLEN_S,      0.50,  5.0,  0.10, 2)
chk_vales = QtWidgets.QCheckBox("Detectar vales (mínimos)"); chk_vales.setChecked(DETECT_VALLEYS); ctrl_layout.addWidget(chk_vales)
ctrl_layout.addSpacing(6); line_sep2 = QtWidgets.QFrame(); line_sep2.setFrameShape(QtWidgets.QFrame.HLine); ctrl_layout.addWidget(line_sep2)
freq_label = QtWidgets.QLabel("Freq(picos): — Hz (— BPM)"); font = freq_label.font(); font.setPointSize(25); font.setBold(True); freq_label.setFont(font); ctrl_layout.addWidget(freq_label)
info_label = QtWidgets.QLabel("Picos (janela 2 s): —\nΔt (1º→último pico): — s"); ctrl_layout.addWidget(info_label)
ctrl_layout.addStretch(1)

def update():
    global serial_buffer, t0_us, current_frequency
    if not connected or ser is None: return

    if ser.in_waiting:
        serial_buffer += ser.read(ser.in_waiting)

    new_points = 0
    while True:
        start_idx = serial_buffer.find(START_MARKER)
        if start_idx == -1: break
        if len(serial_buffer) < start_idx + PACKET_SIZE:
            serial_buffer = serial_buffer[start_idx:]; break
        
        payload = serial_buffer[start_idx + 1 : start_idx + PACKET_SIZE]
        serial_buffer = serial_buffer[start_idx + PACKET_SIZE :]

        try:
            valor_x, valor_y, t_us = struct.unpack('<HHI', payload)
        except struct.error as e:
            print(f'Erro no unpack: {e}'); continue
        
        # Descate de pacotes inválidos
        if not (0 <= valor_x <= 4095 and 0 <= valor_y <= 4095):
            print("Pacote inválido descartado:", valor_x, valor_y)
            continue
        
        # SEMPRE alimenta buffers de calibração quando conectado (mesmo sem coletar)
        calib_x_buffer.append(valor_x)
        calib_y_buffer.append(valor_y)
        
        # Só processa para coleta se estiver coletando
        if not is_collecting or is_warming_up:
            continue
        
        if not btn_salvar.isEnabled() and not time_data:
            btn_salvar.setEnabled(True); btn_resetar.setEnabled(True)

        if t0_us is None:
            t0_us = t_us

        delta_us = (t_us - t0_us) & 0xFFFFFFFF
        t_s = delta_us * 1e-6
        time_data.append(t_s)
        x_data.append(valor_x); y_data.append(valor_y)



        # === Calibração MPX5050DP (ESP32, 12 bits, divisor 10k/18k) ===
        # elétricos
        _beta   = 18000.0 / (10000.0 + 18000.0)          # fator do divisor ~ 0.642857
        _ADC_FS = 3.3                                     # faixa do ADC (V)
        _VS     = 5.0                                     # alimentação do sensor (V)
        _ADCMAX = 4095.0                                  # 12 bits
        _K      = 416.7                                   # 7.5006/0.018 -> mmHg por (Vout/Vs) unidade

        # Conversão comum: ADC -> (Vout/Vs)
        _scale  = _ADC_FS / (_ADCMAX * _beta * _VS)

        # Brutos por canal (sem offset/gain)
        px_raw = _K * ((valor_x * _scale) - 0.04)
        py_raw = _K * ((valor_y * _scale) - 0.04)

        # Aplica calibração se ativa, senão usa ajustes padrão
        if calibration_data['X']['calibrated']:
            px_mmHg = calibration_data['X']['offset'] + calibration_data['X']['gain'] * px_raw
        else:
            # Ajustes finos padrão: offset -7 mmHg, ganho +10%
            px_mmHg = (px_raw - 7.0) * 1.10
            
        if calibration_data['Y']['calibrated']:
            py_mmHg = calibration_data['Y']['offset'] + calibration_data['Y']['gain'] * py_raw
        else:
            # Ajustes finos padrão: offset -68 mmHg, ganho +12%
            py_mmHg = (py_raw - 68.0) * 1.12


        # --- Aplica passa baixas (1ª ordem) ---
        if LPF_ON:
            # estima dt da série de tempo (amostra atual vs anterior)
            if len(time_data) >= 2:
                dt_lpf = time_data[-1] - time_data[-2]
            else:
                dt_lpf = 1.0 / max(current_frequency, 200.0)  # fallback
            px_mmHg = _lpf_step(px_mmHg, dt_lpf, LPF_FC, "x")
            py_mmHg = _lpf_step(py_mmHg, dt_lpf, LPF_FC, "y")


        # Calibragem antiga    
        # px_mmHg = ((valor_x * 0.0048875855) - 0.2) * 83.340222
        # py_mmHg = ((valor_y * 0.0048875855) - 0.2) * 83.340222

        px_data.append(px_mmHg); py_data.append(py_mmHg)

        # Preenche o buffer de análise de picos com base na seleção do dropdown
        recent_t.append(t_s)
        if peak_detection_config['source'] == 'X':
            recent_px.append(px_mmHg)
        else: # 'Y'
            recent_px.append(py_mmHg)
        
        while recent_t and (recent_t[-1] - recent_t[0] > WINDOW_SEC):
            recent_t.popleft(); recent_px.popleft()

        recent_ts.append(t_s)
        new_points += 1

    if len(recent_ts) >= 2:
        dt = recent_ts[-1] - recent_ts[0]
        if dt > 0: current_frequency = (len(recent_ts) - 1) / dt

    if new_points > 0:
        view_points = int(MAX_HISTORY_SECONDS_VIEW * (current_frequency if current_frequency > 0 else 20))
        view_points = max(1, view_points)
        
        if not px_data or not py_data: return
        
        plot_widget.setYRange(min(list(px_data)[-view_points:] + list(py_data)[-view_points:]) -10 , max(list(px_data)[-view_points:] + list(py_data)[-view_points:]) + 10)
        curve_x.setData(list(time_data)[-view_points:], list(px_data)[-view_points:])
        curve_y.setData(list(time_data)[-view_points:], list(py_data)[-view_points:])
        
        if time_data:
            latest_t = time_data[-1]
            plot_widget.setXRange(max(0, latest_t - MAX_HISTORY_SECONDS_VIEW), latest_t, padding=0.01)

    if is_collecting and not is_warming_up and time_data:
        lbl_tempo_coleta.setText(f"Tempo de Coleta: {time_data[-1]:.2f} s")

    if connected:
        state_str = "COLETANDO" if is_collecting and not is_warming_up else ("AQUECENDO..." if is_warming_up else "PAUSADO")
        title = (f'Coleta - Conectado ({ser.port}) - {state_str} | '
                 f'Dados: {current_frequency:.1f} Hz | Freq(picos): {peak_freq_hz:.2f} Hz ({60*peak_freq_hz:.1f} BPM)')
    else:
        title = 'Coleta de Dados Fisioterápicos (desconectado)'
    main_window.setWindowTitle(title)

def _robust_sigma(x):
    x = np.asarray(x, float); med = np.median(x); mad = np.median(np.abs(x - med))
    return 1.4826 * mad if mad > 0 else np.std(x)
def _estimate_fs(t):
    t = np.asarray(t, float);
    if t.size < 3: return np.nan
    dt = np.diff(t); dt = dt[dt > 0]; return 1.0 / np.median(dt) if dt.size else np.nan
def compute_peaks_in_window():
    if len(recent_t) < 3: return np.array([]), np.array([]), np.array([], int), 0.0, 0.0, 0.0
    t = np.asarray(recent_t, float); x = np.asarray(recent_px, float); fs = _estimate_fs(t)
    prom = float(spin_prom.value()) * _robust_sigma(x)
    distance = max(1, int(float(spin_min_dist.value()) * fs)) if np.isfinite(fs) else 1
    width    = max(1, int(float(spin_width.value())    * fs)) if np.isfinite(fs) else 1
    wlen     = max(3, int(float(spin_wlen.value())     * fs)) if (spin_wlen.value() and np.isfinite(fs)) else None
    x_use    = -x if chk_vales.isChecked() else x
    peaks, props = find_peaks(x_use, distance=distance, prominence=prom, width=width, wlen=wlen)
    n = int(peaks.size)
    if n >= 2: dur = float(t[peaks[-1]] - t[peaks[0]]); freq_hz = (n - 1) / dur if dur > 0 else 0.0
    else: dur = 0.0; freq_hz = 0.0
    bpm = 60.0 * freq_hz
    return t, x, peaks, freq_hz, bpm, dur
def refresh_peaks_overlay_and_labels():
    global peak_freq_hz
    if not connected:
        peaks_scatter.setData([], []); freq_label.setText("Freq(picos): — Hz (— BPM)"); info_label.setText("Picos (janela 2 s): —\nΔt (1º→último pico): — s")
        return
    t, x, peaks, freq_hz, bpm, dur = compute_peaks_in_window()
    if t.size and peaks.size: peaks_scatter.setData(t[peaks], x[peaks])
    else: peaks_scatter.setData([], [])
    peak_freq_hz = freq_hz; freq_label.setText(f"Freq(picos): {freq_hz:.2f} Hz  ({bpm:.1f} BPM)")
    if peaks.size >= 2: info_label.setText(f"Picos (janela 2 s): {peaks.size}\nΔt (1º→último pico): {dur:.2f} s")
    else: info_label.setText(f"Picos (janela 2 s): {peaks.size}\nΔt (1º→último pico): — s")


def on_peak_source_change():
    selected_source = combo_peak_source.currentData()
    if peak_detection_config['source'] != selected_source:
        print(f"Análise de picos alterada para o sinal: {selected_source}")
        peak_detection_config['source'] = selected_source
        # Limpa os buffers recentes para forçar uma reanálise com os dados do novo sinal
        recent_t.clear()
        recent_px.clear()
        # Chama a atualização dos picos imediatamente para zerar o display
        refresh_peaks_overlay_and_labels()

peaks_timer = QtCore.QTimer()
peaks_timer.timeout.connect(refresh_peaks_overlay_and_labels)
_param_timer = QtCore.QTimer(); _param_timer.setSingleShot(True); _param_timer.setInterval(150); _param_timer.timeout.connect(refresh_peaks_overlay_and_labels)
def schedule_param_apply(): _param_timer.start()

# Conecta todos os widgets
combo_peak_source.currentIndexChanged.connect(on_peak_source_change) ### MUDANÇA ###
spin_min_dist.valueChanged.connect(schedule_param_apply)
spin_width.valueChanged.connect(schedule_param_apply)
spin_prom.valueChanged.connect(schedule_param_apply)
spin_wlen.valueChanged.connect(schedule_param_apply)
chk_vales.stateChanged.connect(schedule_param_apply)
timer = QtCore.QTimer()
timer.timeout.connect(update)

countdown_timer = QtCore.QTimer()
countdown_timer.timeout.connect(update_countdown)


print('Iniciando aplicação... escolha a porta e clique em "Iniciar".')
exit_code = app.exec_()
sys.exit(exit_code)