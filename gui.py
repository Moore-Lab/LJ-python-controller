#!/usr/bin/env python3
"""
LabJack T7 GUI  —  Kipling-inspired test and calibration tool.

Every interactive button has a CLI shortcut:

  Connection
    --connect/-c <id>          Connect on start (serial#, IP, or ANY)
    --connection-type/-t       USB | TCP | WIFI | ANY  (default ANY)

  AIN
    --read-ain <ch> [<ch>...]  Read AIN channels after connect
    --ain-range <ch>:<v>       Set AIN range (10/1/0.1/0.01 V)

  DAC
    --write-dac <ch> <v>       Write DAC voltage (e.g. --write-dac DAC0 2.5)

  DIO
    --write-dio <ch> <v>       Write digital pin (e.g. --write-dio FIO0 1)

  Registers
    --read-register <name>     Read named register after connect
    --write-register <n> <v>   Write named register after connect

  PT100
    --pt100-channel <ch>       Pre-select PT100 AIN channel (default AIN0)
    --pt100-excitation <A>     Excitation current for manual mode (default 1 mA)
    --pt100-r0 <ohm>           PT100 R0 (default 100.0 Ω)

  Thermocouple
    --tc-channel <ch>          Pre-select TC AIN channel (default AIN0)
    --tc-type <type>           E|J|K|N|R|S|T|C  (default K)

  Polling
    --poll                     Auto-start polling immediately after connect
    --poll-interval <s>        Poll interval in seconds (default 1.0)

Examples:
  python gui.py --connect ANY
  python gui.py -c 192.168.1.50 -t TCP --poll --poll-interval 0.5
  python gui.py -c ANY --write-dac DAC0 2.5
  python gui.py -c ANY --read-ain AIN0 AIN1 AIN2
  python gui.py -c ANY --write-dio FIO0 1
  python gui.py -c ANY --pt100-channel AIN0 --pt100-r0 100.0
"""

from __future__ import annotations

import argparse
import json
import queue
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk
from typing import Any, Callable, Dict, List, Optional, Tuple

try:
    from labjack_t7 import AinChannelConfig, LabjackT7Config, LabJackT7Device
    _HAS_PKG = True
except ImportError:
    _HAS_PKG = False
    LabJackT7Device = None  # type: ignore[assignment,misc]
    LabjackT7Config = None  # type: ignore[assignment]
    AinChannelConfig = None  # type: ignore[assignment]

try:
    import paho.mqtt.client as _mqtt_mod
    _HAS_MQTT = True
except ImportError:
    _mqtt_mod = None  # type: ignore[assignment]
    _HAS_MQTT = False

# ── Channel lists ──────────────────────────────────────────────────────────────
AIN_CH   = [f"AIN{i}" for i in range(14)]
DAC_CH   = ["DAC0", "DAC1"]
FIO_CH   = [f"FIO{i}" for i in range(8)]
EIO_CH   = [f"EIO{i}" for i in range(8)]
CIO_CH   = [f"CIO{i}" for i in range(4)]
DIO_CH   = FIO_CH + EIO_CH + CIO_CH
AIN_RANGES = ["10.0", "1.0", "0.1", "0.01"]
TC_TYPES = ["E", "J", "K", "N", "R", "S", "T", "C"]

# EF indices visible in the reference legend
EF_INDEX_NAMES: Dict[int, str] = {
    0:  "Off (normal AIN)",
    1:  "Slope + Offset",
    10: "Binary Average",
    20: "Thermocouple E",
    21: "Thermocouple J",
    22: "Thermocouple K",
    23: "Thermocouple N",
    24: "Thermocouple R",
    25: "Thermocouple S",
    26: "Thermocouple T",
    27: "Thermocouple C",
    40: "RTD PT100/PT1000",
    44: "Resistance",
}

# EF TC indices for each TC type
EF_TC_IDX = {"E": 20, "J": 21, "K": 22, "N": 23, "R": 24, "S": 25, "T": 26, "C": 27}


class PublishRule:
    """One MQTT publish rule: read <register> every <interval_s> and publish to <topic>."""
    _counter = 0

    def __init__(self, register: str, topic: str, interval_s: float) -> None:
        PublishRule._counter += 1
        self.id           = PublishRule._counter
        self.register     = register
        self.topic        = topic
        self.interval_s   = interval_s
        self.enabled      = True
        self.job: Optional[str] = None   # tkinter after() handle
        self.last_val: Optional[float] = None
        self.pub_count    = 0

# ── PT100 / Callendar–Van Dusen ────────────────────────────────────────────────
_CVD_A =  3.9083e-3
_CVD_B = -5.775e-7
_CVD_C = -4.183e-12   # only for T < 0 °C


def cvd_resistance_to_celsius(R: float, R0: float = 100.0,
                               A: float = _CVD_A, B: float = _CVD_B,
                               C: float = _CVD_C) -> float:
    """Invert Callendar–Van Dusen via Newton–Raphson."""
    t = (R / R0 - 1.0) / A   # linear seed
    for _ in range(20):
        if t >= 0.0:
            f  = R0 * (1 + A*t + B*t**2) - R
            df = R0 * (A + 2*B*t)
        else:
            f  = R0 * (1 + A*t + B*t**2 + C*(t - 100)*t**3) - R
            df = R0 * (A + 2*B*t + C*(4*t**3 - 300*t**2))
        if abs(df) < 1e-15:
            break
        step = f / df
        t -= step
        if abs(step) < 1e-9:
            break
    return t


# ── Background worker ──────────────────────────────────────────────────────────
class DeviceWorker:
    """Serialises LJM calls onto a daemon thread; posts (status, tag, result) tuples."""

    def __init__(self, result_q: queue.Queue) -> None:
        import threading
        self._rq  = result_q
        self._cmd: queue.Queue = queue.Queue()
        self._t   = threading.Thread(target=self._run, name="ljm-worker", daemon=True)
        self._t.start()

    def submit(self, fn: Callable, *args: Any, tag: str = "", **kw: Any) -> None:
        self._cmd.put((tag, fn, args, kw))

    def stop(self) -> None:
        self._cmd.put(None)

    def _run(self) -> None:
        while True:
            item = self._cmd.get()
            if item is None:
                return
            tag, fn, args, kw = item
            try:
                self._rq.put(("ok", tag, fn(*args, **kw)))
            except Exception as exc:
                self._rq.put(("err", tag, str(exc)))


# ── Tiny widget helpers ────────────────────────────────────────────────────────
def _lbl(p, text, **kw):  return ttk.Label(p, text=text, **kw)
def _btn(p, text, cmd, **kw): return ttk.Button(p, text=text, command=cmd, **kw)
def _ent(p, var, w=10, **kw): return ttk.Entry(p, textvariable=var, width=w, **kw)
def _cb(p, var, vals, w=8):
    return ttk.Combobox(p, textvariable=var, values=vals, width=w, state="readonly")


# ── Main application ───────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.title("LabJack T7 Controller")
        self.geometry("980x680")
        self.resizable(True, True)

        self._args    = args
        self._rq: queue.Queue = queue.Queue()
        self._worker  = DeviceWorker(self._rq)
        self._device: Optional[LabJackT7Device] = None

        self._poll_job:     Optional[str] = None
        self._pt100_poll:   Optional[str] = None
        self._tc_poll:      Optional[str] = None

        # MQTT publish rules
        self._mqtt_client: Any = None
        self._mqtt_connected = False
        self._rules: Dict[int, PublishRule] = {}

        self._build_ui()
        self.after(50, self._poll_results)
        if args.connect:
            self.after(300, self._auto_connect)

    # ── UI assembly ───────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self._build_conn_bar()
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=4, pady=4)
        self._build_ain_tab(nb)
        self._build_dac_tab(nb)
        self._build_dio_tab(nb)
        self._build_ef_tab(nb)
        self._build_pt100_tab(nb)
        self._build_tc_tab(nb)
        self._build_reg_tab(nb)
        self._build_mqtt_tab(nb)
        self._build_console_tab(nb)

    # ── Connection bar ────────────────────────────────────────────────────────

    def _build_conn_bar(self) -> None:
        bar = ttk.Frame(self, relief="groove", borderwidth=1)
        bar.pack(fill="x", padx=4, pady=(4, 0))

        _lbl(bar, "Type:").pack(side="left", padx=(6, 2))
        self._v_ctype = tk.StringVar(value=self._args.connection_type)
        _cb(bar, self._v_ctype, ["ANY", "USB", "TCP", "WIFI"], 6).pack(side="left")

        _lbl(bar, "  ID:").pack(side="left", padx=(8, 2))
        self._v_ident = tk.StringVar(value=self._args.connect or "ANY")
        _ent(bar, self._v_ident, 16).pack(side="left")

        self._btn_conn = _btn(bar, "Connect",    self._do_connect)
        self._btn_disc = _btn(bar, "Disconnect", self._do_disconnect)
        self._btn_conn.pack(side="left", padx=(8, 2))
        self._btn_disc.pack(side="left")

        self._v_status = tk.StringVar(value="Disconnected")
        self._status_lbl = ttk.Label(bar, textvariable=self._v_status,
                                     foreground="red", font=("Consolas", 10, "bold"))
        self._status_lbl.pack(side="left", padx=14)

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=8, pady=3)

        _lbl(bar, "Poll (s):").pack(side="left")
        self._v_poll_iv = tk.StringVar(value=str(self._args.poll_interval))
        _ent(bar, self._v_poll_iv, 5).pack(side="left", padx=2)
        _btn(bar, "Start Poll", self._start_poll).pack(side="left", padx=(6, 2))
        _btn(bar, "Stop Poll",  self._stop_poll).pack(side="left")

    # ── AIN tab ───────────────────────────────────────────────────────────────

    def _build_ain_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="AIN")

        # Column headers
        hdr = ttk.Frame(f)
        hdr.pack(fill="x", padx=6, pady=(4, 0))
        for txt, w in [("Channel", 8), ("Value (V)", 13), ("Range (V)", 8),
                       ("Resolution", 8), ("EF Index", 7), ("Actions", 16)]:
            _lbl(hdr, txt, width=w, anchor="center",
                 font=("TkDefaultFont", 9, "bold")).pack(side="left", padx=2)

        # Scrollable row area
        outer = ttk.Frame(f)
        outer.pack(fill="both", expand=True, padx=4, pady=2)
        canvas = tk.Canvas(outer, highlightthickness=0)
        vsb    = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        inner  = ttk.Frame(canvas)
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")

        self._ain_val:  Dict[str, tk.StringVar] = {}
        self._ain_rng:  Dict[str, tk.StringVar] = {}
        self._ain_res:  Dict[str, tk.StringVar] = {}
        self._ain_ef:   Dict[str, tk.StringVar] = {}

        for ch in AIN_CH:
            row = ttk.Frame(inner)
            row.pack(fill="x", padx=2, pady=1)

            _lbl(row, ch, width=8, anchor="w").pack(side="left")

            v = tk.StringVar(value="---")
            self._ain_val[ch] = v
            _lbl(row, textvariable=v, width=13, anchor="e",
                 font=("Consolas", 10)).pack(side="left", padx=2)

            rng = tk.StringVar(value="10.0")
            self._ain_rng[ch] = rng
            _cb(row, rng, AIN_RANGES, 6).pack(side="left", padx=2)

            res = tk.StringVar(value="0")
            self._ain_res[ch] = res
            ttk.Spinbox(row, textvariable=res, from_=0, to=8,
                        width=4).pack(side="left", padx=2)

            ef = tk.StringVar(value="0")
            self._ain_ef[ch] = ef
            _ent(row, ef, 6).pack(side="left", padx=2)

            _btn(row, "Read",   lambda c=ch: self._read_ain(c),      width=6).pack(side="left", padx=2)
            _btn(row, "Config", lambda c=ch: self._configure_ain(c), width=6).pack(side="left")

        btn = ttk.Frame(f)
        btn.pack(fill="x", padx=6, pady=4)
        _btn(btn, "Read All AIN",       self._read_all_ain).pack(side="left", padx=4)
        _btn(btn, "Configure All AIN",  self._configure_all_ain).pack(side="left")

    def _read_ain(self, ch: str) -> None:
        if not self._ready(): return
        self._worker.submit(self._device.read_ain, ch, tag=f"ain:{ch}")

    def _read_all_ain(self) -> None:
        if not self._ready(): return
        self._worker.submit(self._device.read_ains, AIN_CH, tag="ain:all")

    def _configure_ain(self, ch: str) -> None:
        if not self._ready(): return
        cfg = AinChannelConfig(
            name=ch,
            range_v=float(self._ain_rng[ch].get()),
            resolution_index=int(self._ain_res[ch].get()),
        )
        ef_idx = int(self._ain_ef[ch].get())

        def _apply():
            self._device.configure_ain(cfg)
            if ef_idx != 0:
                self._device.write_register(f"{ch}_EF_INDEX", float(ef_idx))
            return f"{ch} configured (range={cfg.range_v}V res={cfg.resolution_index} EF={ef_idx})"

        self._worker.submit(_apply, tag=f"cfg:{ch}")

    def _configure_all_ain(self) -> None:
        for ch in AIN_CH:
            self._configure_ain(ch)

    # ── DAC tab ───────────────────────────────────────────────────────────────

    def _build_dac_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="DAC")

        self._dac_var: Dict[str, tk.StringVar] = {}

        for dac in DAC_CH:
            grp = ttk.LabelFrame(f, text=dac, padding=10)
            grp.pack(fill="x", padx=16, pady=10)

            v = tk.StringVar(value="0.0000")
            self._dac_var[dac] = v

            _lbl(grp, "Voltage (0 – 5 V):").grid(row=0, column=0, sticky="e", padx=6)
            _ent(grp, v, 12).grid(row=0, column=1, padx=6)

            sl = ttk.Scale(grp, from_=0.0, to=5.0, orient="horizontal", length=340,
                           command=lambda val, sv=v: sv.set(f"{float(val):.4f}"))
            sl.grid(row=0, column=2, padx=8)

            def _sync(name, idx, mode, s=sl, sv=v):
                try: s.set(float(sv.get()))
                except ValueError: pass
            v.trace_add("write", _sync)

            _btn(grp, f"Set {dac}",
                 lambda d=dac: self._write_dac(d)).grid(row=0, column=3, padx=6)
            _btn(grp, "Zero (0 V)",
                 lambda d=dac: self._write_dac(d, 0.0)).grid(row=0, column=4, padx=4)

    def _write_dac(self, ch: str, value: Optional[float] = None) -> None:
        if not self._ready(): return
        if value is None:
            try:
                value = float(self._dac_var[ch].get())
            except ValueError:
                messagebox.showerror("Invalid", f"Bad voltage for {ch}")
                return
        self._dac_var[ch].set(f"{value:.4f}")
        self._worker.submit(self._device.write_dac, ch, value, tag=f"dac:{ch}")

    # ── DIO tab ───────────────────────────────────────────────────────────────

    def _build_dio_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="DIO")

        self._dio_state: Dict[str, tk.IntVar]    = {}
        self._dio_dir:   Dict[str, tk.StringVar] = {}
        self._dio_ind:   Dict[str, ttk.Label]    = {}

        for group, channels in [("FIO (0 – 7)", FIO_CH),
                                 ("EIO (0 – 7)", EIO_CH),
                                 ("CIO (0 – 3)", CIO_CH)]:
            grp = ttk.LabelFrame(f, text=group, padding=6)
            grp.pack(fill="x", padx=12, pady=6)
            for i, ch in enumerate(channels):
                col = ttk.Frame(grp)
                col.grid(row=0, column=i, padx=8, pady=2)

                _lbl(col, ch, font=("Consolas", 9, "bold")).pack()

                dv = tk.StringVar(value="IN")
                self._dio_dir[ch] = dv
                _cb(col, dv, ["IN", "OUT"], 4).pack(pady=1)

                sv = tk.IntVar(value=0)
                self._dio_state[ch] = sv

                # LED indicator (label that changes colour)
                ind = ttk.Label(col, text="●", foreground="gray", font=("TkDefaultFont", 14))
                self._dio_ind[ch] = ind
                ind.pack()

                ttk.Checkbutton(col, variable=sv, text="High",
                                command=lambda c=ch: self._write_dio(c)).pack()
                _btn(col, "R", lambda c=ch: self._read_dio(c), width=3).pack(pady=1)

        btn = ttk.Frame(f)
        btn.pack(fill="x", padx=12, pady=4)
        _btn(btn, "Read All DIO", self._read_all_dio).pack(side="left")

    def _read_dio(self, ch: str) -> None:
        if not self._ready(): return
        self._worker.submit(self._device.read_dio, ch, tag=f"dio:{ch}")

    def _write_dio(self, ch: str) -> None:
        if self._dio_dir[ch].get() != "OUT":
            return
        if not self._ready(): return
        val = self._dio_state[ch].get()
        self._worker.submit(self._device.write_dio, ch, val, tag=f"dio_w:{ch}")

    def _read_all_dio(self) -> None:
        if not self._ready(): return
        for ch in DIO_CH:
            self._read_dio(ch)

    # ── Extended Features tab ─────────────────────────────────────────────────

    def _build_ef_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="Ext. Features")

        left = ttk.LabelFrame(f, text="AIN Extended Feature Configuration", padding=10)
        left.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        def lrow(label, var, r, vals=None, w=12):
            _lbl(left, label, width=14, anchor="e").grid(row=r, column=0, sticky="e", padx=4, pady=3)
            w_ = _cb(left, var, vals, w) if vals else _ent(left, var, w)
            w_.grid(row=r, column=1, sticky="w", padx=4)

        self._ef_ch    = tk.StringVar(value="AIN0")
        self._ef_idx   = tk.StringVar(value="0")
        self._ef_cfgA  = tk.StringVar(value="0")
        self._ef_cfgB  = tk.StringVar(value="0")
        self._ef_cfgC  = tk.StringVar(value="0")
        self._ef_cfgD  = tk.StringVar(value="0")

        lrow("Channel:",   self._ef_ch,   0, vals=AIN_CH)
        lrow("EF Index:",  self._ef_idx,  1)
        lrow("Config A:",  self._ef_cfgA, 2)
        lrow("Config B:",  self._ef_cfgB, 3)
        lrow("Config C:",  self._ef_cfgC, 4)
        lrow("Config D:",  self._ef_cfgD, 5)

        btn_row = ttk.Frame(left)
        btn_row.grid(row=6, column=0, columnspan=2, pady=6)
        _btn(btn_row, "Apply EF Config", self._apply_ef).pack(side="left", padx=4)
        _btn(btn_row, "Read EF",         self._read_ef).pack(side="left", padx=4)

        res_frame = ttk.LabelFrame(left, text="EF Results", padding=6)
        res_frame.grid(row=7, column=0, columnspan=2, sticky="ew", pady=4)
        self._ef_res: Dict[str, tk.StringVar] = {}
        for key in ("READ_A", "READ_B", "READ_C", "READ_D"):
            rv = tk.StringVar(value="---")
            self._ef_res[key] = rv
            r = ttk.Frame(res_frame)
            r.pack(anchor="w")
            _lbl(r, f"{key}:", width=10, anchor="e").pack(side="left")
            _lbl(r, textvariable=rv, font=("Consolas", 10), width=20).pack(side="left")

        # Legend on the right
        legend = ttk.LabelFrame(f, text="EF Index Reference", padding=8)
        legend.pack(side="left", fill="y", padx=(0, 8), pady=8)
        for idx, name in EF_INDEX_NAMES.items():
            _lbl(legend, f"  {idx:3d}  {name}", font=("Consolas", 9),
                 anchor="w").pack(fill="x")

    def _apply_ef(self) -> None:
        if not self._ready(): return
        ch = self._ef_ch.get()
        vals = {
            f"{ch}_EF_INDEX":    float(self._ef_idx.get()),
            f"{ch}_EF_CONFIG_A": float(self._ef_cfgA.get()),
            f"{ch}_EF_CONFIG_B": float(self._ef_cfgB.get()),
            f"{ch}_EF_CONFIG_C": float(self._ef_cfgC.get()),
            f"{ch}_EF_CONFIG_D": float(self._ef_cfgD.get()),
        }

        def _write():
            for name, v in vals.items():
                self._device.write_register(name, v)
            return f"EF applied to {ch}"

        self._worker.submit(_write, tag="ef:apply")

    def _read_ef(self) -> None:
        if not self._ready(): return
        ch = self._ef_ch.get()
        names = [f"{ch}_EF_READ_A", f"{ch}_EF_READ_B",
                 f"{ch}_EF_READ_C", f"{ch}_EF_READ_D"]

        def _read():
            vals = self._device.read_registers(names)
            return dict(zip(("READ_A", "READ_B", "READ_C", "READ_D"), vals))

        self._worker.submit(_read, tag="ef:read")

    # ── PT100 tab ─────────────────────────────────────────────────────────────

    def _build_pt100_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="PT100")

        cfg = ttk.LabelFrame(f, text="4-Wire PT100 Configuration", padding=10)
        cfg.pack(fill="x", padx=12, pady=8)

        def lrow(label, var, r, vals=None, w=12):
            _lbl(cfg, label, width=24, anchor="e").grid(row=r, column=0, sticky="e", padx=4, pady=3)
            w_ = _cb(cfg, var, vals, w) if vals else _ent(cfg, var, w)
            w_.grid(row=r, column=1, sticky="w", padx=4)
            return w_

        self._pt_ch     = tk.StringVar(value=self._args.pt100_channel)
        self._pt_method = tk.StringVar(value="EF (AIN_EF_INDEX=40, RTD mode)")
        self._pt_I      = tk.StringVar(value=str(self._args.pt100_excitation))
        self._pt_R0     = tk.StringVar(value=str(self._args.pt100_r0))
        self._pt_A      = tk.StringVar(value=str(_CVD_A))
        self._pt_B      = tk.StringVar(value=str(_CVD_B))
        self._pt_C      = tk.StringVar(value=str(_CVD_C))

        lrow("AIN channel (+):",         self._pt_ch,     0, vals=AIN_CH)
        lrow("Measurement method:",      self._pt_method, 1,
             vals=["EF (AIN_EF_INDEX=40, RTD mode)",
                   "Manual  R = V / I_excitation"])
        lrow("Excitation current I (A):", self._pt_I,    2)
        lrow("R₀ (Ω):",                   self._pt_R0,   3)

        sep = ttk.Separator(cfg, orient="horizontal")
        sep.grid(row=4, column=0, columnspan=2, sticky="ew", pady=6)
        _lbl(cfg, "Callendar–Van Dusen coefficients (IEC 60751):",
             font=("TkDefaultFont", 9, "bold")).grid(row=5, column=0, columnspan=2,
                                                      sticky="w", padx=8)
        lrow("A:",           self._pt_A, 6)
        lrow("B:",           self._pt_B, 7)
        lrow("C (T < 0°C):", self._pt_C, 8)

        btn = ttk.Frame(f)
        btn.pack(fill="x", padx=12, pady=4)
        _btn(btn, "Read Resistance", self._read_pt100).pack(side="left", padx=4)
        _btn(btn, "Start Poll",      self._start_pt100_poll).pack(side="left", padx=4)
        _btn(btn, "Stop Poll",       self._stop_pt100_poll).pack(side="left")

        res = ttk.LabelFrame(f, text="Measurement Result", padding=8)
        res.pack(fill="x", padx=12, pady=4)
        self._pt_raw    = tk.StringVar(value="---")
        self._pt_R      = tk.StringVar(value="---")
        self._pt_tempC  = tk.StringVar(value="---")
        self._pt_tempK  = tk.StringVar(value="---")
        for label, var, u in [
            ("Raw AIN voltage (V):", self._pt_raw,   ""),
            ("Resistance (Ω):",      self._pt_R,     ""),
            ("Temperature (°C):",    self._pt_tempC, ""),
            ("Temperature (K):",     self._pt_tempK, ""),
        ]:
            r = ttk.Frame(res)
            r.pack(anchor="w", pady=1)
            _lbl(r, label, width=22, anchor="e").pack(side="left")
            _lbl(r, textvariable=var, font=("Consolas", 11), width=18).pack(side="left")

    def _read_pt100(self) -> None:
        if not self._ready(): return
        ch     = self._pt_ch.get()
        method = self._pt_method.get()
        I_exc  = self._pt_I.get()

        if "EF" in method:
            def _read():
                # READ_B for EF40 = resistance in ohms
                R = self._device.read_register(f"{ch}_EF_READ_B")
                return ("ef", 0.0, R)
        else:
            I = float(I_exc)
            def _read():
                V = self._device.read_ain(ch)
                R = V / I if I != 0.0 else float("nan")
                return ("manual", V, R)

        self._worker.submit(_read, tag="pt100:read")

    def _update_pt100(self, result: Tuple) -> None:
        method, V, R = result
        self._pt_raw.set(f"{V:.6f}" if method == "manual" else "N/A (EF)")
        self._pt_R.set(f"{R:.5f}")
        try:
            R0 = float(self._pt_R0.get())
            A  = float(self._pt_A.get())
            B  = float(self._pt_B.get())
            C  = float(self._pt_C.get())
            Tc = cvd_resistance_to_celsius(R, R0, A, B, C)
            self._pt_tempC.set(f"{Tc:.5f}")
            self._pt_tempK.set(f"{Tc + 273.15:.5f}")
        except Exception:
            self._pt_tempC.set("calc error")
            self._pt_tempK.set("calc error")

    def _start_pt100_poll(self) -> None:
        self._read_pt100()
        try:   iv = int(float(self._v_poll_iv.get()) * 1000)
        except ValueError: iv = 1000
        self._pt100_poll = self.after(iv, self._start_pt100_poll)

    def _stop_pt100_poll(self) -> None:
        if self._pt100_poll:
            self.after_cancel(self._pt100_poll)
            self._pt100_poll = None

    # ── Thermocouple tab ──────────────────────────────────────────────────────

    def _build_tc_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="Thermocouple")

        cfg = ttk.LabelFrame(f, text="Thermocouple Configuration", padding=10)
        cfg.pack(fill="x", padx=12, pady=8)

        def lrow(label, var, r, vals=None, w=12):
            _lbl(cfg, label, width=26, anchor="e").grid(row=r, column=0, sticky="e", padx=4, pady=3)
            w_ = _cb(cfg, var, vals, w) if vals else _ent(cfg, var, w)
            w_.grid(row=r, column=1, sticky="w", padx=4)

        self._tc_ch    = tk.StringVar(value=self._args.tc_channel)
        self._tc_type  = tk.StringVar(value=self._args.tc_type)
        self._tc_diff  = tk.BooleanVar(value=True)
        self._tc_ref   = tk.StringVar(value=self._args.tc_ref or "Internal T7")
        self._tc_mode  = tk.StringVar(value="Raw EMF  (for calibration)")

        lrow("AIN channel (+):",     self._tc_ch,   0, vals=AIN_CH)
        lrow("TC type:",             self._tc_type, 1, vals=TC_TYPES)
        lrow("Measurement mode:",    self._tc_mode, 2,
             vals=["Raw EMF  (for calibration)",
                   "EF Temperature  (K, standard tables)"])
        lrow("Reference junction:",  self._tc_ref,  3,
             vals=["Internal T7"] + AIN_CH)

        _lbl(cfg, "Differential (uses ch+1 as −):", width=26, anchor="e").grid(
            row=4, column=0, sticky="e", padx=4, pady=3)
        ttk.Checkbutton(cfg, variable=self._tc_diff).grid(row=4, column=1, sticky="w", padx=4)

        _lbl(cfg, "Note: Raw EMF mode reads the raw junction voltage for your own\n"
             "calibration. EF mode applies the standard NIST polynomial directly.",
             foreground="gray", justify="left").grid(
             row=5, column=0, columnspan=2, sticky="w", padx=8, pady=(4, 0))

        btn = ttk.Frame(f)
        btn.pack(fill="x", padx=12, pady=4)
        _btn(btn, "Read",       self._read_tc).pack(side="left", padx=4)
        _btn(btn, "Start Poll", self._start_tc_poll).pack(side="left", padx=4)
        _btn(btn, "Stop Poll",  self._stop_tc_poll).pack(side="left")

        res = ttk.LabelFrame(f, text="Measurement Result", padding=8)
        res.pack(fill="x", padx=12, pady=4)
        self._tc_emf   = tk.StringVar(value="---")
        self._tc_ref_T = tk.StringVar(value="---")
        self._tc_temp  = tk.StringVar(value="---")
        for label, var in [
            ("EMF (mV):",               self._tc_emf),
            ("Reference junction (K):", self._tc_ref_T),
            ("Temperature (K):",        self._tc_temp),
        ]:
            r = ttk.Frame(res)
            r.pack(anchor="w", pady=1)
            _lbl(r, label, width=24, anchor="e").pack(side="left")
            _lbl(r, textvariable=var, font=("Consolas", 11), width=18).pack(side="left")

    def _read_tc(self) -> None:
        if not self._ready(): return
        ch   = self._tc_ch.get()
        mode = self._tc_mode.get()
        diff = self._tc_diff.get()
        ref  = self._tc_ref.get()

        if "EF" in mode:
            ef_idx = float(EF_TC_IDX.get(self._tc_type.get(), 22))
            def _read():
                self._device.write_register(f"{ch}_EF_INDEX", ef_idx)
                self._device.write_register(f"{ch}_EF_CONFIG_A", 1.0)  # use CJC
                temp_k = self._device.read_register(f"{ch}_EF_READ_A")
                return ("ef", 0.0, temp_k, 0.0)
        else:
            def _read():
                if diff:
                    ch_idx = int(ch.replace("AIN", ""))
                    self._device.write_register(f"{ch}_NEGATIVE_CH", float(ch_idx + 1))
                    V = self._device.read_ain(ch)
                    self._device.write_register(f"{ch}_NEGATIVE_CH", 199.0)  # restore to GND
                else:
                    V = self._device.read_ain(ch)
                emf_mv = V * 1000.0

                if ref == "Internal T7":
                    ref_k = self._device.read_internal_temp_k()
                else:
                    ref_v = self._device.read_ain(ref)
                    ref_r = ref_v / 0.001   # assume 1 mA excitation
                    ref_k = cvd_resistance_to_celsius(ref_r) + 273.15

                return ("raw", emf_mv, 0.0, ref_k)

        self._worker.submit(_read, tag="tc:read")

    def _update_tc(self, result: Tuple) -> None:
        mode, emf_mv, temp_k, ref_k = result
        if mode == "ef":
            self._tc_emf.set("N/A (EF mode)")
            self._tc_ref_T.set("N/A (EF mode)")
            self._tc_temp.set(f"{temp_k:.5f}")
        else:
            self._tc_emf.set(f"{emf_mv:.6f}")
            self._tc_ref_T.set(f"{ref_k:.4f}")
            self._tc_temp.set("— (raw mode, use EMF)")

    def _start_tc_poll(self) -> None:
        self._read_tc()
        try:   iv = int(float(self._v_poll_iv.get()) * 1000)
        except ValueError: iv = 1000
        self._tc_poll = self.after(iv, self._start_tc_poll)

    def _stop_tc_poll(self) -> None:
        if self._tc_poll:
            self.after_cancel(self._tc_poll)
            self._tc_poll = None

    # ── Registers tab ─────────────────────────────────────────────────────────

    def _build_reg_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="Registers")

        top = ttk.LabelFrame(f, text="Read / Write Named Register", padding=8)
        top.pack(fill="x", padx=12, pady=8)

        _lbl(top, "Register:").grid(row=0, column=0, sticky="e", padx=4)
        self._reg_name = tk.StringVar(value=self._args.read_register or "SERIAL_NUMBER")
        _ent(top, self._reg_name, 26).grid(row=0, column=1, padx=4)

        _lbl(top, "Value:").grid(row=0, column=2, padx=4)
        self._reg_val = tk.StringVar()
        _ent(top, self._reg_val, 14).grid(row=0, column=3, padx=4)

        self._reg_result = tk.StringVar(value="---")
        _lbl(top, textvariable=self._reg_result,
             font=("Consolas", 11), width=22).grid(row=0, column=5, padx=10)

        _btn(top, "Read",  self._read_register).grid(row=0, column=4, padx=4)
        _btn(top, "Write", self._write_register).grid(row=1, column=4, pady=4, padx=4)

        quick = ttk.LabelFrame(f, text="Quick access — click to read", padding=8)
        quick.pack(fill="x", padx=12, pady=4)
        common = [
            "SERIAL_NUMBER", "HARDWARE_VERSION", "FIRMWARE_VERSION",
            "TEMPERATURE_DEVICE_K", "TEMPERATURE_AIR_K",
            "DAC0", "DAC1",
            "AIN0", "AIN1", "AIN2", "AIN3",
            "FIO_STATE", "EIO_STATE", "CIO_STATE",
        ]
        for i, name in enumerate(common):
            _btn(quick, name,
                 lambda n=name: (self._reg_name.set(n), self._read_register()),
                 width=22).grid(row=i // 4, column=i % 4, padx=4, pady=2)

    def _read_register(self) -> None:
        if not self._ready(): return
        name = self._reg_name.get().strip()
        if not name: return
        self._worker.submit(self._device.read_register, name, tag="reg:read")

    def _write_register(self) -> None:
        if not self._ready(): return
        name = self._reg_name.get().strip()
        try:
            val = float(self._reg_val.get())
        except ValueError:
            messagebox.showerror("Invalid", "Value must be a number")
            return
        self._worker.submit(self._device.write_register, name, val, tag="reg:write")

    # ── MQTT Publish Rules tab ────────────────────────────────────────────────

    def _build_mqtt_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="MQTT Rules")

        if not _HAS_MQTT:
            _lbl(f, "paho-mqtt not installed.  Run:  pip install paho-mqtt",
                 foreground="red", font=("TkDefaultFont", 11)).pack(pady=40)
            return

        # ── Broker connection ──────────────────────────────────────────────
        broker = ttk.LabelFrame(f, text="MQTT Broker", padding=8)
        broker.pack(fill="x", padx=12, pady=8)

        _lbl(broker, "Host:").grid(row=0, column=0, sticky="e", padx=4)
        self._v_mqtt_host = tk.StringVar(value="localhost")
        _ent(broker, self._v_mqtt_host, 20).grid(row=0, column=1, padx=4)

        _lbl(broker, "Port:").grid(row=0, column=2, padx=(12, 4))
        self._v_mqtt_port = tk.StringVar(value="1883")
        _ent(broker, self._v_mqtt_port, 6).grid(row=0, column=3, padx=4)

        _lbl(broker, "Client ID:").grid(row=0, column=4, padx=(12, 4))
        self._v_mqtt_cid = tk.StringVar(value="lj-t7-gui")
        _ent(broker, self._v_mqtt_cid, 14).grid(row=0, column=5, padx=4)

        _btn(broker, "Connect",    self._mqtt_connect).grid(row=0, column=6, padx=8)
        _btn(broker, "Disconnect", self._mqtt_disconnect).grid(row=0, column=7)

        self._v_mqtt_status = tk.StringVar(value="Disconnected")
        self._mqtt_status_lbl = ttk.Label(broker, textvariable=self._v_mqtt_status,
                                           foreground="red", font=("Consolas", 10, "bold"))
        self._mqtt_status_lbl.grid(row=0, column=8, padx=14)

        # ── Add rule form ──────────────────────────────────────────────────
        add = ttk.LabelFrame(f, text="Add Publish Rule", padding=8)
        add.pack(fill="x", padx=12, pady=4)

        _lbl(add, "Register / channel:").grid(row=0, column=0, sticky="e", padx=4)
        self._v_rule_reg = tk.StringVar(value="AIN0")
        _ent(add, self._v_rule_reg, 22).grid(row=0, column=1, padx=4)

        _lbl(add, "MQTT topic:").grid(row=0, column=2, padx=(12, 4))
        self._v_rule_topic = tk.StringVar(value="xsphere/sensors/labjack_t7/ain/ain0")
        _ent(add, self._v_rule_topic, 36).grid(row=0, column=3, padx=4)

        _lbl(add, "Interval (s):").grid(row=0, column=4, padx=(12, 4))
        self._v_rule_iv = tk.StringVar(value="1.0")
        _ent(add, self._v_rule_iv, 6).grid(row=0, column=5, padx=4)

        _btn(add, "Add Rule", self._add_rule).grid(row=0, column=6, padx=8)

        _lbl(add, "Register can be any named T7 register (AIN0, TEMPERATURE_DEVICE_K, DAC0, …)",
             foreground="gray", font=("TkDefaultFont", 8)
             ).grid(row=1, column=0, columnspan=7, sticky="w", padx=4, pady=(2, 0))

        # ── Rules table ────────────────────────────────────────────────────
        tbl_frame = ttk.LabelFrame(f, text="Active Rules", padding=4)
        tbl_frame.pack(fill="both", expand=True, padx=12, pady=4)

        cols = ("id", "register", "topic", "interval", "enabled", "last_value", "count")
        self._rule_tree = ttk.Treeview(
            tbl_frame, columns=cols, show="headings", selectmode="browse", height=10
        )
        for col, heading, w in [
            ("id",         "ID",         4),
            ("register",   "Register",   14),
            ("topic",      "Topic",      38),
            ("interval",   "Interval s", 10),
            ("enabled",    "Enabled",    8),
            ("last_value", "Last Value", 14),
            ("count",      "Published",  10),
        ]:
            self._rule_tree.heading(col, text=heading)
            self._rule_tree.column(col, width=w * 9, anchor="center")

        vsb = ttk.Scrollbar(tbl_frame, orient="vertical",
                            command=self._rule_tree.yview)
        self._rule_tree.configure(yscrollcommand=vsb.set)
        self._rule_tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")

        # ── Table controls ─────────────────────────────────────────────────
        ctrl = ttk.Frame(f)
        ctrl.pack(fill="x", padx=12, pady=(0, 6))
        _btn(ctrl, "Enable Selected",  self._enable_rule).pack(side="left", padx=4)
        _btn(ctrl, "Disable Selected", self._disable_rule).pack(side="left", padx=4)
        _btn(ctrl, "Remove Selected",  self._remove_rule).pack(side="left", padx=4)
        ttk.Separator(ctrl, orient="vertical").pack(side="left", fill="y", padx=8, pady=2)
        _btn(ctrl, "Enable All",       self._enable_all_rules).pack(side="left", padx=4)
        _btn(ctrl, "Disable All",      self._disable_all_rules).pack(side="left", padx=4)
        _btn(ctrl, "Remove All",       self._remove_all_rules).pack(side="left", padx=4)

    # ── MQTT broker ───────────────────────────────────────────────────────────

    def _mqtt_connect(self) -> None:
        if not _HAS_MQTT:
            return
        host = self._v_mqtt_host.get().strip()
        port = int(self._v_mqtt_port.get())
        cid  = self._v_mqtt_cid.get().strip() or "lj-t7-gui"

        if self._mqtt_client:
            try: self._mqtt_client.disconnect()
            except Exception: pass

        client = _mqtt_mod.Client(client_id=cid, protocol=_mqtt_mod.MQTTv5)
        client.on_connect    = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client = client
        try:
            client.connect(host, port, keepalive=60)
            client.loop_start()
        except Exception as exc:
            self._v_mqtt_status.set(f"Error: {exc}")
            self._mqtt_status_lbl.configure(foreground="red")
            self._log(f"[MQTT] connect failed: {exc}")

    def _mqtt_disconnect(self) -> None:
        self._disable_all_rules()
        if self._mqtt_client:
            try:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
            except Exception:
                pass
        self._mqtt_connected = False
        if hasattr(self, "_v_mqtt_status"):
            self._v_mqtt_status.set("Disconnected")
            self._mqtt_status_lbl.configure(foreground="red")

    def _on_mqtt_connect(self, client, userdata, flags, rc, props=None) -> None:
        self._mqtt_connected = True
        self.after(0, lambda: (
            self._v_mqtt_status.set("Connected"),
            self._mqtt_status_lbl.configure(foreground="green"),
            self._log("[MQTT] connected to broker"),
        ))

    def _on_mqtt_disconnect(self, client, userdata, rc, props=None) -> None:
        self._mqtt_connected = False
        self.after(0, lambda: (
            self._v_mqtt_status.set("Disconnected"),
            self._mqtt_status_lbl.configure(foreground="red"),
        ))

    # ── Publish rules ─────────────────────────────────────────────────────────

    def _add_rule(self) -> None:
        reg  = self._v_rule_reg.get().strip()
        topic = self._v_rule_topic.get().strip()
        try:
            iv = float(self._v_rule_iv.get())
        except ValueError:
            messagebox.showerror("Invalid", "Interval must be a number")
            return
        if not reg or not topic:
            messagebox.showerror("Invalid", "Register and topic are required")
            return

        rule = PublishRule(reg, topic, iv)
        self._rules[rule.id] = rule
        self._tree_insert(rule)
        self._schedule_rule(rule)
        self._log(f"[MQTT] rule #{rule.id} added: {reg} → {topic} every {iv}s")

    def _tree_insert(self, rule: PublishRule) -> None:
        self._rule_tree.insert("", "end", iid=str(rule.id), values=(
            rule.id, rule.register, rule.topic,
            f"{rule.interval_s:.2f}",
            "✓" if rule.enabled else "✗",
            "---", 0,
        ))

    def _tree_refresh(self, rule: PublishRule) -> None:
        self._rule_tree.item(str(rule.id), values=(
            rule.id, rule.register, rule.topic,
            f"{rule.interval_s:.2f}",
            "✓" if rule.enabled else "✗",
            f"{rule.last_val:.6f}" if rule.last_val is not None else "---",
            rule.pub_count,
        ))

    def _selected_rule(self) -> Optional[PublishRule]:
        sel = self._rule_tree.selection()
        if not sel:
            return None
        return self._rules.get(int(sel[0]))

    def _enable_rule(self) -> None:
        r = self._selected_rule()
        if r and not r.enabled:
            r.enabled = True
            self._schedule_rule(r)
            self._tree_refresh(r)

    def _disable_rule(self) -> None:
        r = self._selected_rule()
        if r and r.enabled:
            r.enabled = False
            if r.job:
                self.after_cancel(r.job)
                r.job = None
            self._tree_refresh(r)

    def _remove_rule(self) -> None:
        r = self._selected_rule()
        if not r:
            return
        if r.job:
            self.after_cancel(r.job)
        del self._rules[r.id]
        self._rule_tree.delete(str(r.id))

    def _enable_all_rules(self) -> None:
        for r in self._rules.values():
            if not r.enabled:
                r.enabled = True
                self._schedule_rule(r)
            self._tree_refresh(r)

    def _disable_all_rules(self) -> None:
        for r in self._rules.values():
            r.enabled = False
            if r.job:
                self.after_cancel(r.job)
                r.job = None
            if hasattr(self, "_rule_tree"):
                self._tree_refresh(r)

    def _remove_all_rules(self) -> None:
        for r in list(self._rules.values()):
            if r.job:
                self.after_cancel(r.job)
        self._rules.clear()
        for iid in self._rule_tree.get_children():
            self._rule_tree.delete(iid)

    # ── Rule scheduling ───────────────────────────────────────────────────────

    def _schedule_rule(self, rule: PublishRule) -> None:
        if not rule.enabled:
            return
        if not self._ready():
            return
        rule.job = self.after(
            int(rule.interval_s * 1000),
            lambda r=rule: self._fire_rule(r),
        )

    def _fire_rule(self, rule: PublishRule) -> None:
        rule.job = None
        if not rule.enabled or rule.id not in self._rules:
            return
        if not self._device or not self._device.connected:
            # device gone — reschedule and wait
            self._schedule_rule(rule)
            return
        self._worker.submit(
            self._device.read_register, rule.register,
            tag=f"mqtt_rule:{rule.id}",
        )
        self._schedule_rule(rule)

    # ── Console tab ───────────────────────────────────────────────────────────

    def _build_console_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb)
        nb.add(f, text="Console")

        self._console = scrolledtext.ScrolledText(
            f, font=("Consolas", 10), state="disabled", wrap="word")
        self._console.pack(fill="both", expand=True, padx=4, pady=4)

        cmd = ttk.Frame(f)
        cmd.pack(fill="x", padx=4, pady=(0, 4))
        self._v_cmd = tk.StringVar()
        e = ttk.Entry(cmd, textvariable=self._v_cmd, font=("Consolas", 10))
        e.pack(side="left", fill="x", expand=True, padx=(0, 4))
        e.bind("<Return>", lambda _: self._run_cmd())
        _btn(cmd, "Run", self._run_cmd).pack(side="left")
        _lbl(cmd, "   read <name>  |  write <name> <value>  |  connect [id]",
             foreground="gray", font=("TkDefaultFont", 8)).pack(side="left", padx=8)

    def _run_cmd(self) -> None:
        raw = self._v_cmd.get().strip()
        if not raw: return
        self._v_cmd.set("")
        self._log(f">>> {raw}")
        parts = raw.split()
        cmd = parts[0].lower()
        if cmd == "read" and len(parts) >= 2:
            self._reg_name.set(parts[1])
            self._read_register()
        elif cmd == "write" and len(parts) >= 3:
            self._reg_name.set(parts[1])
            self._reg_val.set(parts[2])
            self._write_register()
        elif cmd in ("connect", "conn"):
            if len(parts) >= 2:
                self._v_ident.set(parts[1])
            self._do_connect()
        elif cmd in ("disconnect", "disc"):
            self._do_disconnect()
        else:
            self._log("Commands: read <name>  |  write <name> <value>  |  connect [id]  |  disconnect")

    # ── Connection ────────────────────────────────────────────────────────────

    def _do_connect(self) -> None:
        ident = self._v_ident.get().strip() or "ANY"
        ctype = self._v_ctype.get()
        self._log(f"Connecting — type={ctype} id={ident}")
        self._v_status.set("Connecting…")
        cfg = LabjackT7Config(connection_type=ctype, device_identifier=ident)
        self._device = LabJackT7Device(cfg)
        self._worker.submit(self._device.connect, tag="connect")

    def _do_disconnect(self) -> None:
        self._stop_poll()
        self._stop_pt100_poll()
        self._stop_tc_poll()
        if self._device:
            self._worker.submit(self._device.disconnect, tag="disconnect")

    def _auto_connect(self) -> None:
        self._do_connect()

    def _ready(self) -> bool:
        if not self._device or not self._device.connected:
            messagebox.showwarning("Not connected", "Connect to a LabJack T7 first.")
            return False
        return True

    # ── Polling ───────────────────────────────────────────────────────────────

    def _start_poll(self) -> None:
        self._read_all_ain()
        self._read_all_dio()
        try:   iv = int(float(self._v_poll_iv.get()) * 1000)
        except ValueError: iv = 1000
        self._poll_job = self.after(iv, self._start_poll)

    def _stop_poll(self) -> None:
        if self._poll_job:
            self.after_cancel(self._poll_job)
            self._poll_job = None

    # ── Result dispatch ───────────────────────────────────────────────────────

    def _poll_results(self) -> None:
        try:
            while True:
                status, tag, result = self._rq.get_nowait()
                self._dispatch(status, tag, result)
        except queue.Empty:
            pass
        self.after(50, self._poll_results)

    def _dispatch(self, status: str, tag: str, result: Any) -> None:
        if status == "err":
            self._log(f"[ERR] {tag}: {result}")
            if tag == "connect":
                self._v_status.set("Connection failed")
                self._status_lbl.configure(foreground="red")
            return

        # ── success ──────────────────────────────────────────────────────────
        if tag == "connect":
            sn = self._device.serial or "?"
            self._v_status.set(f"Connected  —  T7  SN#{sn}")
            self._status_lbl.configure(foreground="green")
            self._log(f"Connected to T7 serial={sn}")
            if self._args.poll:
                self._start_poll()
            self._post_connect_args()

        elif tag == "disconnect":
            self._v_status.set("Disconnected")
            self._status_lbl.configure(foreground="red")
            self._device = None
            self._log("Disconnected")

        elif tag.startswith("ain:"):
            sub = tag[4:]
            if sub == "all":
                for ch, v in zip(AIN_CH, result):
                    self._ain_val[ch].set(f"{v:.6f}")
            else:
                self._ain_val[sub].set(f"{result:.6f}")

        elif tag.startswith("dio:") and not tag.startswith("dio_w:"):
            ch = tag[4:]
            self._dio_state[ch].set(result)
            self._dio_ind[ch].configure(foreground="green" if result else "gray")

        elif tag == "ef:read":
            for k, v in result.items():
                self._ef_res[k].set(f"{v:.6f}")

        elif tag == "pt100:read":
            self._update_pt100(result)

        elif tag == "tc:read":
            self._update_tc(result)

        elif tag == "reg:read":
            self._reg_result.set(str(result))
            self._log(f"[reg] {self._reg_name.get()} = {result}")

        elif tag == "reg:write":
            self._reg_result.set(f"{result}  (written)")
            self._log(f"[reg] {self._reg_name.get()} ← {result}")

        elif tag.startswith("mqtt_rule:"):
            rule_id = int(tag.split(":", 1)[1])
            rule = self._rules.get(rule_id)
            if rule:
                rule.last_val = result
                rule.pub_count += 1
                self._tree_refresh(rule)
                if self._mqtt_connected and self._mqtt_client:
                    payload = json.dumps({
                        "value":    result,
                        "register": rule.register,
                        "ts":       time.time(),
                    })
                    self._mqtt_client.publish(rule.topic, payload, qos=1)

        else:
            self._log(f"[OK]  {tag}: {result}")

    def _post_connect_args(self) -> None:
        a = self._args
        if a.read_ain:
            for ch in a.read_ain:
                self._read_ain(ch.upper())
        if a.write_dac:
            ch, val = a.write_dac
            self._write_dac(ch.upper(), float(val))
        if a.write_dio:
            ch, val = a.write_dio
            self._dio_dir[ch.upper()].set("OUT")
            self._dio_state[ch.upper()].set(int(val))
            self._write_dio(ch.upper())
        if a.read_register:
            self._read_register()
        if a.write_register:
            name, val = a.write_register
            self._reg_name.set(name)
            self._reg_val.set(val)
            self._write_register()

    # ── Console ───────────────────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._console.configure(state="normal")
        self._console.insert("end", f"[{ts}] {msg}\n")
        self._console.see("end")
        self._console.configure(state="disabled")

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy(self) -> None:
        self._stop_poll()
        self._stop_pt100_poll()
        self._stop_tc_poll()
        self._disable_all_rules()
        self._mqtt_disconnect()
        self._worker.stop()
        super().destroy()


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="gui.py",
        description="LabJack T7 GUI — Kipling-inspired test and calibration tool.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    g = p.add_argument_group
    c = g("Connection")
    c.add_argument("--connect", "-c", metavar="ID",
                   help="Connect on start (serial#, IP, or ANY)")
    c.add_argument("--connection-type", "-t", default="ANY",
                   choices=["ANY", "USB", "TCP", "WIFI"])

    a = g("AIN")
    a.add_argument("--read-ain", nargs="+", metavar="CH",
                   help="Read AIN channel(s) after connect")

    d = g("DAC")
    d.add_argument("--write-dac", nargs=2, metavar=("CH", "V"),
                   help="Write DAC channel (e.g. --write-dac DAC0 2.5)")

    io = g("DIO")
    io.add_argument("--write-dio", nargs=2, metavar=("CH", "V"),
                    help="Write DIO pin (e.g. --write-dio FIO0 1)")

    r = g("Registers")
    r.add_argument("--read-register",  metavar="NAME",
                   help="Read named register after connect")
    r.add_argument("--write-register", nargs=2, metavar=("NAME", "V"),
                   help="Write named register after connect")

    pt = g("PT100")
    pt.add_argument("--pt100-channel",    default="AIN0", metavar="CH")
    pt.add_argument("--pt100-excitation", default=0.001,  type=float, metavar="A",
                    help="Excitation current for manual mode (default 1 mA)")
    pt.add_argument("--pt100-r0",         default=100.0,  type=float, metavar="Ω",
                    help="R0 reference resistance (default 100 Ω)")

    tc = g("Thermocouple")
    tc.add_argument("--tc-channel", default="AIN0", metavar="CH")
    tc.add_argument("--tc-type",    default="K",    choices=TC_TYPES)
    tc.add_argument("--tc-ref",     default=None,   metavar="SOURCE",
                    help="Reference junction: 'Internal T7' or AIN channel")

    po = g("Polling")
    po.add_argument("--poll",          action="store_true",
                    help="Auto-start polling after connect")
    po.add_argument("--poll-interval", default=1.0, type=float, metavar="S")

    return p.parse_args()


def main() -> None:
    args = _parse()
    if not _HAS_PKG:
        print("WARNING: labjack_t7 package not found — run: pip install -e .")
    app = App(args)
    app.mainloop()


if __name__ == "__main__":
    main()
