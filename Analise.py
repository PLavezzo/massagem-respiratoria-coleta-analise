# -*- coding: utf-8 -*-
import os
import numpy as np
import pandas as pd
import customtkinter as ctk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from tkinter import filedialog, messagebox

# ===================== PARÂMETROS GLOBAIS =====================
TIME_COL = "Tempo (s)"
df = pd.DataFrame(columns=[TIME_COL])
df_orig = pd.DataFrame(columns=[TIME_COL]) 

channel_state = {"name": ""}
params = {
    "MIN_DIST_S": 0.05, "WIDTH_MIN_S": 0.01, "PROM_FACTOR": 1.50,
    "WLEN_S": 2.00, "BUSCAR_VALES": False
}
fft_params = {
    "FREQ_ANALISE_MIN": 3.0, "FREQ_ANALISE_MAX": 25.0
}
# Guarda os últimos resultados para exportação
last_results = {"picos_df": None, "fft_pico": None}


# ===================== ARQUIVO E DADOS =====================
# Botão carregamento arquivo
def carregar_novo_arquivo():
    global df, df_orig, t_all, t_min, t_max, dt_min, available_channels, default_channel
    
    fn = filedialog.askopenfilename(
        title="Selecione o arquivo de dados",
        filetypes=[("Excel", "*.xlsx"), ("CSV", "*.csv")]
    )
    if not fn:
        return 

    try:
        if fn.lower().endswith(".csv"):
            new_df = pd.read_csv(fn, sep=';', decimal='.', encoding='utf-8')
        else:
            new_df = pd.read_excel(fn)

        if TIME_COL not in new_df.columns:
            messagebox.showerror("Erro", f"A planilha precisa ter a coluna '{TIME_COL}'.")
            return
            
        df = new_df
        df_orig = df.copy()

        # Reinicia todas as variáveis de estado e a UI com os novos dados
        t_all = df[TIME_COL].to_numpy(float)
        t_min, t_max = float(np.min(t_all)), float(np.max(t_all))
        dt_all = np.diff(t_all)
        dt_min = float(np.median(dt_all[dt_all > 0])) if np.any(dt_all > 0) else 0.001

        available_channels = [c for c in df.select_dtypes(include=[np.number]).columns if c != TIME_COL]
        if not available_channels:
            messagebox.showerror("Erro", "Nenhuma coluna numérica de dados encontrada.")
            return

        default_channel = "Pressão X (mmHg)" if "Pressão X (mmHg)" in available_channels else ("Pressão Y (mmHg)" if "Pressão Y (mmHg)" in available_channels else ("X_Filtrado" if "X_Filtrado" in available_channels else ("Y_Filtrado" if "Y_Filtrado" in available_channels else available_channels[0])))
        channel_state["name"] = default_channel

        # Atualiza o dropdown de canais
        opt_channel.configure(values=available_channels)
        opt_channel.set(default_channel)

        # Reseta a interface para o estado inicial
        resetar()
        
    except Exception as e:
        messagebox.showerror("Erro ao Carregar", f"Não foi possível carregar o arquivo:\n{e}")

# exportaa dados da análise atual
def exportar_analise():
    if df.empty or have_cut["v"] == False:
        messagebox.showwarning("Aviso", "Aplique um corte no sinal antes de exportar.")
        return

    default_name = f"analise_{pd.Timestamp.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
    fn = filedialog.asksaveasfilename(
        title="Salvar análise como...",
        defaultextension=".xlsx",
        filetypes=[("Excel Workbook", "*.xlsx")],
        initialfile=default_name
    )
    if not fn:
        return 

    try:
        with pd.ExcelWriter(fn, engine='openpyxl') as writer:
            
            df.to_excel(writer, sheet_name='Dados Cortados', index=False)

            
            if last_results["picos_df"] is not None and not last_results["picos_df"].empty:
                # Adiciona o pico da FFT ao DataFrame de resultados
                results_df = last_results["picos_df"].copy()
                if last_results["fft_pico"] is not None:
                    results_df['Pico FFT (Hz)'] = last_results["fft_pico"]
                
                results_df.to_excel(writer, sheet_name='Resultados dos Picos', index=False)
        
        messagebox.showinfo("Sucesso", f"Análise exportada com sucesso para:\n{fn}")
    except Exception as e:
        messagebox.showerror("Erro ao Exportar", f"Não foi possível salvar o arquivo:\n{e}")


# ===================== FUNÇÕES DE ANÁLISE =====================
def robust_sigma(x):
    x = np.asarray(x); med = np.median(x); mad = np.median(np.abs(x - med))
    return 1.4826 * mad if mad > 0 else np.std(x)

def detectar_picos(t, x, p):
    t = np.asarray(t, float); x = np.asarray(x, float)
    if len(t) < 3: return np.array([], dtype=int), {}, np.nan
    dt = np.diff(t); fs = 1.0 / np.median(dt) if np.any(dt > 0) else np.nan
    prom = p["PROM_FACTOR"] * robust_sigma(x)
    distance = max(1, int(p["MIN_DIST_S"] * fs)) if np.isfinite(fs) else 1
    width = max(1, int(p["WIDTH_MIN_S"] * fs)) if np.isfinite(fs) else 1
    wlen = max(3, int(p["WLEN_S"] * fs)) if (p["WLEN_S"] and np.isfinite(fs)) else None
    x_use = -x if p["BUSCAR_VALES"] else x
    peaks, props = find_peaks(x_use, distance=distance, prominence=prom, width=width, wlen=wlen)
    return peaks, props, fs

# ===================== FUNÇÕES DE PLOTAGEM =====================
def recomputa_e_desenha_picos(df_cur, atualizar_metricas=True):
    if df_cur.empty: return
    canal = channel_state["name"]; t = df_cur[TIME_COL].to_numpy(float); x = df_cur[canal].to_numpy(float)
    ax_sinal.clear(); ax_sinal.grid(True, alpha=0.3); ax_sinal.set_xlabel("tempo (s)"); ax_sinal.set_ylabel(canal)
    ax_sinal.plot(t, x, lw=1.2, label=canal)
    peaks, props, fs = detectar_picos(t, x, params)
    tp = t[peaks]; xp = x[peaks]
    ax_sinal.plot(tp, xp, "o", label="picos"); ax_sinal.legend(loc="upper right"); canvas_sinal.draw_idle()
    
    # Guarda os picos encontrados para a exportação
    if atualizar_metricas:
        picos_export_df = pd.DataFrame({
            'Tempo do Pico (s)': tp,
            f'Valor do Pico ({canal})': xp
        })
        last_results["picos_df"] = picos_export_df
        
        n_peaks = int(peaks.size)
        if n_peaks >= 2: dur_peaks = float(tp[-1] - tp[0]); freq_hz = (n_peaks - 1) / dur_peaks if dur_peaks > 0 else 0.0
        else: dur_peaks = 0.0; freq_hz = 0.0
        lbl_npicos.configure(text=f"Picos: {n_peaks}"); lbl_dur.configure(text=f"Tempo total: {dur_peaks:.2f} s"); lbl_freq.configure(text=f"Frequência: {freq_hz:.2f} Hz  ({60.0*freq_hz:.1f} BPM)")

def recomputa_e_desenha_fft(df_cur):
    ax_fft.clear()
    last_results["fft_pico"] = None # Reseta o valor
    if df_cur.empty:
        canvas_fft.draw_idle()
        return

    canal = channel_state["name"]; t = df_cur[TIME_COL].to_numpy(float); x = df_cur[canal].to_numpy(float)
    
    if len(t) < 10:
        ax_fft.text(0.5, 0.5, 'Sinal muito curto para análise FFT', ha='center', va='center')
        lbl_fft_pico.configure(text="Pico FFT: —")
        canvas_fft.draw_idle()
        return

    x = x - np.mean(x)
    N = len(x); fs = 1.0 / np.mean(np.diff(t))
    yf = np.fft.fft(x); xf = np.fft.fftfreq(N, 1 / fs)
    idx_pos = np.where(xf >= 0); xf_full = xf[idx_pos]; yf_full = 2.0/N * np.abs(yf[idx_pos])
    
    mask_analise = (xf_full >= fft_params["FREQ_ANALISE_MIN"]) & (xf_full <= fft_params["FREQ_ANALISE_MAX"])
    freqs_analise = xf_full[mask_analise]; amps_analise = yf_full[mask_analise]
    
    ax_fft.plot(xf_full, yf_full, color='c')
    
    if len(freqs_analise) > 0:
        idx_pico = np.argmax(amps_analise)
        freq_pico = freqs_analise[idx_pico]; amp_pico = amps_analise[idx_pico]
        lbl_fft_pico.configure(text=f"Pico FFT: {freq_pico:.2f} Hz")
        ax_fft.plot(freq_pico, amp_pico, 'ro', markersize=8, label=f'Pico ({freq_pico:.2f} Hz)')
        last_results["fft_pico"] = freq_pico # Guarda o pico da FFT para exportação
    else:
        lbl_fft_pico.configure(text="Pico FFT: —")
    
    ax_fft.axvspan(fft_params["FREQ_ANALISE_MIN"], fft_params["FREQ_ANALISE_MAX"], color='gray', alpha=0.2, label='Banda de Análise')
    ax_fft.grid(True, alpha=0.3); ax_fft.set_xlabel("Frequência (Hz)"); ax_fft.set_ylabel("Amplitude")
    ax_fft.set_xlim(0, 30); ax_fft.legend(loc="upper right"); canvas_fft.draw_idle()

# ===================== UI (Interface Gráfica) =====================
ctk.set_appearance_mode("system"); ctk.set_default_color_theme("blue")
root = ctk.CTk(); root.title("Análise de Sinais e Picos"); root.geometry("1450x900")
root.grid_columnconfigure(0, weight=3); root.grid_columnconfigure(1, weight=1); root.grid_rowconfigure(0, weight=1)

frame_plot = ctk.CTkFrame(root)
frame_plot.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
frame_plot.grid_columnconfigure(0, weight=1)
frame_plot.grid_rowconfigure(0, weight=1); frame_plot.grid_rowconfigure(2, weight=1) # Gráficos
frame_plot.grid_rowconfigure(1, weight=0); frame_plot.grid_rowconfigure(3, weight=0) # Toolbars
frame_plot.grid_rowconfigure(4, weight=0) # Sliders de tempo

fig_sinal, ax_sinal = plt.subplots(); canvas_sinal = FigureCanvasTkAgg(fig_sinal, master=frame_plot)
canvas_sinal.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
toolbar_sinal = NavigationToolbar2Tk(canvas_sinal, frame_plot, pack_toolbar=False)
toolbar_sinal.update(); toolbar_sinal.grid(row=1, column=0, sticky="ew", padx=5)

fig_fft, ax_fft = plt.subplots(); canvas_fft = FigureCanvasTkAgg(fig_fft, master=frame_plot)
canvas_fft.get_tk_widget().grid(row=2, column=0, sticky="nsew", padx=5, pady=5)
toolbar_fft = NavigationToolbar2Tk(canvas_fft, frame_plot, pack_toolbar=False)
toolbar_fft.update(); toolbar_fft.grid(row=3, column=0, sticky="ew", padx=5)

frame_sliders_tempo = ctk.CTkFrame(frame_plot, fg_color="transparent")
frame_sliders_tempo.grid(row=4, column=0, sticky="ew", padx=5, pady=5)
frame_sliders_tempo.grid_columnconfigure(0, weight=1)
lbl_inicio = ctk.CTkLabel(frame_sliders_tempo, text=f"Início: -- s"); lbl_inicio.grid(row=0, column=0, sticky="ew")
s_inicio = ctk.CTkSlider(frame_sliders_tempo, from_=0, to=1); s_inicio.set(0); s_inicio.grid(row=1, column=0, sticky="ew")
lbl_fim = ctk.CTkLabel(frame_sliders_tempo, text=f"Fim: -- s"); lbl_fim.grid(row=2, column=0, sticky="ew")
s_fim = ctk.CTkSlider(frame_sliders_tempo, from_=0, to=1); s_fim.set(1); s_fim.grid(row=3, column=0, sticky="ew")

span = {"obj": None}
def desenhar_span(lo, hi):
    if span["obj"] is not None:
        try: span["obj"].remove()
        except Exception: pass
    span["obj"] = ax_sinal.axvspan(lo, hi, alpha=0.18, color='lightblue')
    canvas_sinal.draw_idle()
state = {"t0": 0, "t1": 1}; _busy = {"v": False}
def on_inicio(v):
    if _busy["v"]: return
    v = float(v)
    if v >= state["t1"] - 0.01: _busy["v"] = True; s_inicio.set(state["t1"] - 0.01); _busy["v"] = False; v = s_inicio.get()
    state["t0"] = v; lbl_inicio.configure(text=f"Início: {v:.3f} s"); desenhar_span(*sorted([state["t0"], state["t1"]]))
def on_fim(v):
    if _busy["v"]: return
    v = float(v)
    if v <= state["t0"] + 0.01: _busy["v"] = True; s_fim.set(state["t0"] + 0.01); _busy["v"] = False; v = s_fim.get()
    state["t1"] = v; lbl_fim.configure(text=f"Fim: {v:.3f} s"); desenhar_span(*sorted([state["t0"], state["t1"]]))
s_inicio.configure(command=on_inicio); s_fim.configure(command=on_fim)

frame_side = ctk.CTkScrollableFrame(root, width=400)
frame_side.grid(row=0, column=1, sticky="ns", padx=(0,10), pady=10)

# botões de carregar e exportar
frame_arquivos = ctk.CTkFrame(frame_side)
frame_arquivos.pack(fill="x", padx=8, pady=(10, 8))
frame_arquivos.grid_columnconfigure((0,1), weight=1)
btn_carregar = ctk.CTkButton(frame_arquivos, text="Carregar Arquivo", command=carregar_novo_arquivo)
btn_carregar.grid(row=0, column=0, padx=(0,4), sticky="ew")
btn_exportar = ctk.CTkButton(frame_arquivos, text="Exportar Análise (.xlsx)", command=exportar_analise, fg_color="#28a745")
btn_exportar.grid(row=0, column=1, padx=(4,0), sticky="ew")

ctk.CTkLabel(frame_side, text="Canal", font=ctk.CTkFont(size=14, weight="bold")).pack(fill="x", padx=8, pady=(4, 4))
opt_channel = ctk.CTkOptionMenu(frame_side, values=["Nenhum arquivo carregado"]); opt_channel.pack(fill="x", padx=8, pady=(0, 8))

have_cut = {"v": False}; pending_update_picos = {"id": None}; pending_update_fft = {"id": None}
def schedule_update_picos(delay_ms=150):
    if pending_update_picos["id"] is not None: root.after_cancel(pending_update_picos["id"])
    pending_update_picos["id"] = root.after(delay_ms, lambda: recomputa_e_desenha_picos(df, atualizar_metricas=have_cut["v"]))
def schedule_update_fft(delay_ms=200):
    if pending_update_fft["id"] is not None: root.after_cancel(pending_update_fft["id"])
    pending_update_fft["id"] = root.after(delay_ms, lambda: recomputa_e_desenha_fft(df))
def on_channel_change(new_name):
    channel_state["name"] = new_name; schedule_update_picos(); schedule_update_fft()
opt_channel.configure(command=on_channel_change)

row_btns = ctk.CTkFrame(frame_side); row_btns.pack(fill="x", padx=8, pady=(4,8)); row_btns.grid_columnconfigure((0,1), weight=1)
def aplicar_corte():
    if df_orig.empty: return
    global df
    lo, hi = sorted([state["t0"], state["t1"]]); df = df_orig[(df_orig[TIME_COL] >= lo) & (df_orig[TIME_COL] <= hi)].copy()
    if len(df) >= 1: t = df[TIME_COL].to_numpy(float); lo_n, hi_n = float(t.min()), float(t.max())
    else: lo_n, hi_n = lo, hi
    _busy["v"] = True; s_inicio.configure(from_=lo_n, to=hi_n); s_fim.configure(from_=lo_n, to=hi_n); s_inicio.set(lo_n); s_fim.set(hi_n); _busy["v"] = False
    state["t0"], state["t1"] = lo_n, hi_n; lbl_inicio.configure(text=f"Início: {lo_n:.3f} s"); lbl_fim.configure(text=f"Fim: {hi_n:.3f} s")
    have_cut["v"] = True
    recomputa_e_desenha_picos(df, atualizar_metricas=True); recomputa_e_desenha_fft(df); desenhar_span(lo_n, hi_n)
def resetar():
    if df_orig.empty: return
    global df
    df = df_orig.copy(); t = df[TIME_COL].to_numpy(float); lo_n, hi_n = float(t.min()), float(t.max())
    _busy["v"] = True; s_inicio.configure(from_=lo_n, to=hi_n); s_fim.configure(from_=lo_n, to=hi_n); s_inicio.set(lo_n); s_fim.set(hi_n); _busy["v"] = False
    state["t0"], state["t1"] = lo_n, hi_n; lbl_inicio.configure(text=f"Início: {lo_n:.3f} s"); lbl_fim.configure(text=f"Fim: {hi_n:.3f} s")
    have_cut["v"] = False
    lbl_npicos.configure(text="Picos: —"); lbl_dur.configure(text="Tempo total: —"); lbl_freq.configure(text="Frequência: —")
    recomputa_e_desenha_picos(df, atualizar_metricas=False); recomputa_e_desenha_fft(df); desenhar_span(lo_n, hi_n)
btn_cortar = ctk.CTkButton(row_btns, text="Aplicar Corte", command=aplicar_corte); btn_cortar.grid(row=0, column=0, sticky="ew", padx=(0,4))
btn_reset = ctk.CTkButton(row_btns, text="Resetar Sinal", fg_color="gray", command=resetar); btn_reset.grid(row=0, column=1, sticky="ew", padx=(4,0))

ctk.CTkLabel(frame_side, text="Parâmetros de Detecção de Picos", font=ctk.CTkFont(size=14, weight="bold")).pack(fill="x", padx=8, pady=(8, 6))
def add_param_slider(parent, texto, from_, to_, get_val, set_val, step=0.01, p_type='picos'):
    frame = ctk.CTkFrame(parent, fg_color="transparent")
    ctk.CTkLabel(frame, text=texto).pack(side="left", padx=(0,5))
    val_label = ctk.CTkLabel(frame, text=f"{get_val():.3f}"); val_label.pack(side="right")
    frame.pack(fill="x", padx=8, pady=(0,2))
    sld = ctk.CTkSlider(parent, from_=from_, to=to_); sld.set(get_val()); sld.pack(fill="x", padx=8, pady=(0,6))
    def _on(v):
        v = float(v)
        if step > 0: v = round(v / step) * step
        set_val(v); val_label.configure(text=f"{v:.3f}")
        if p_type == 'picos': schedule_update_picos()
        else: schedule_update_fft()
    sld.configure(command=_on)
    return sld, val_label

add_param_slider(frame_side, "Distância mínima (s)", 0.005, 0.50, lambda: params["MIN_DIST_S"], lambda v: params.__setitem__("MIN_DIST_S", v), step=0.005)
add_param_slider(frame_side, "Largura mínima (s)", 0.001, 0.10, lambda: params["WIDTH_MIN_S"], lambda v: params.__setitem__("WIDTH_MIN_S", v), step=0.005)
add_param_slider(frame_side, "Prominência ×σ", 0.10, 2.00, lambda: params["PROM_FACTOR"], lambda v: params.__setitem__("PROM_FACTOR", v), step=0.05)
add_param_slider(frame_side, "Janela (s)", 0.50, 5.00, lambda: params["WLEN_S"], lambda v: params.__setitem__("WLEN_S", v), step=0.10)
sw_vales = ctk.CTkSwitch(frame_side, text="Detectar vales (mínimos)", command=lambda: (params.__setitem__("BUSCAR_VALES", bool(sw_vales.get())), schedule_update_picos()))
sw_vales.pack(fill="x", padx=8, pady=(4,8))
sw_vales.select() if params["BUSCAR_VALES"] else sw_vales.deselect()

ctk.CTkLabel(frame_side, text="Parâmetros de Frequência (FFT)", font=ctk.CTkFont(size=14, weight="bold")).pack(fill="x", padx=8, pady=(15, 6))
add_param_slider(frame_side, "Freq. Mínima (Hz)", 0, 15, lambda: fft_params["FREQ_ANALISE_MIN"], lambda v: fft_params.__setitem__("FREQ_ANALISE_MIN", v), step=0.5, p_type='fft')
add_param_slider(frame_side, "Freq. Máxima (Hz)", 3, 30, lambda: fft_params["FREQ_ANALISE_MAX"], lambda v: fft_params.__setitem__("FREQ_ANALISE_MAX", v), step=0.5, p_type='fft')
lbl_fft_pico = ctk.CTkLabel(frame_side, text="Pico FFT: —", font=ctk.CTkFont(size=14, weight="bold")); lbl_fft_pico.pack(fill="x", padx=8, pady=5)

ctk.CTkLabel(frame_side, text="Métricas de Picos", font=ctk.CTkFont(size=14, weight="bold")).pack(fill="x", padx=8, pady=(15, 2))
lbl_npicos = ctk.CTkLabel(frame_side, text="Picos: —"); lbl_npicos.pack(fill="x", padx=8, anchor="w")
lbl_dur    = ctk.CTkLabel(frame_side, text="Tempo total: —");  lbl_dur.pack(fill="x", padx=8, anchor="w")
lbl_freq   = ctk.CTkLabel(frame_side, text="Frequência: —");   lbl_freq.pack(fill="x", padx=8, anchor="w")

# Inicia a UI com gráficos vazios
ax_sinal.grid(True, alpha=0.3); ax_sinal.set_xlabel("tempo (s)")
ax_sinal.grid(True, alpha=0.3); ax_sinal.set_xlabel("tempo (s)")
ax_fft.grid(True, alpha=0.3); ax_fft.set_xlabel("Frequência (Hz)")
ax_sinal.text(0.5, 0.5, 'Carregue um arquivo para começar', ha='center', va='center', fontsize=12)
ax_fft.text(0.5, 0.5, 'Carregue um arquivo para começar', ha='center', va='center', fontsize=12)

root.mainloop()