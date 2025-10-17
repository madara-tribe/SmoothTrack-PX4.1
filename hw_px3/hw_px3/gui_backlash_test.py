#!/usr/bin/env python3
import serial
import time
import threading
import tkinter as tk
from tkinter import messagebox

# ====== User settings ======
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# 送信プロトコル選択：
#   - True  : 整数角のみ送信（元スクリプト互換）
#   - False : 小数角も送信（Arduino 側が float で受けられる場合）
INTEGER_PROTOCOL = True

# 角度の反転（元スクリプトの挙動 180 - angle を継承）
INVERT_ANGLE = True

# ====== Serial setup ======
ser = None
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print(f"Connected to {ARDUINO_PORT} at {BAUD_RATE} baud")
except serial.SerialException as e:
    # GUI が無い環境だと messagebox が失敗することがあるため print も残す
    print(f"Serial Error: Could not connect to {ARDUINO_PORT}: {e}")
    try:
        messagebox.showerror("Serial Error", f"Could not connect to {ARDUINO_PORT}:\n{e}")
    except Exception:
        pass

def _format_angle_for_send(angle_deg: float) -> str:
    """Apply invert and integer/float protocol, return payload without newline."""
    v = 180.0 - angle_deg if INVERT_ANGLE else angle_deg
    if INTEGER_PROTOCOL:
        return f"{int(round(v))}"
    else:
        # 0.1° 単位で送信（Arduino 側が float 受け可能な場合のみ有効）
        return f"{v:.1f}"

def send_angle(angle_deg):
    """Send one angle to serial."""
    global ser
    if ser and ser.is_open:
        try:
            payload = _format_angle_for_send(float(angle_deg)) + "\n"
            ser.write(payload.encode('ascii'))
            print(f"Sent: {payload.strip()}")
        except Exception as e:
            print(f"Failed to send: {e}")

# ====== GUI and test logic ======
class BacklashTester:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.running = False
        self.thread = None

        # --- UI ---
        root.title("Servo Backlash / Micro-step Test")
        root.geometry("420x220")

        # Manual slider (kept from original)
        self.slider = tk.Scale(root, from_=0, to=360, orient="horizontal",
                               command=self._on_slider, label="Manual Angle")
        self.slider.set(90)
        self.slider.pack(fill="x", padx=12, pady=(10, 6))

        # Params frame
        frm = tk.Frame(root)
        frm.pack(fill="x", padx=12, pady=4)

        tk.Label(frm, text="Center(deg):").grid(row=0, column=0, sticky="w")
        tk.Label(frm, text="Step(deg):").grid(row=1, column=0, sticky="w")
        tk.Label(frm, text="Dwell(ms):").grid(row=2, column=0, sticky="w")
        tk.Label(frm, text="Cycles(0=∞):").grid(row=3, column=0, sticky="w")

        self.center_var = tk.DoubleVar(value=90.0)
        self.step_var   = tk.DoubleVar(value=0.5)
        self.dwell_var  = tk.IntVar(value=400)
        self.cycles_var = tk.IntVar(value=0)

        tk.Entry(frm, textvariable=self.center_var, width=8).grid(row=0, column=1, padx=6, pady=2, sticky="w")
        tk.Entry(frm, textvariable=self.step_var,   width=8).grid(row=1, column=1, padx=6, pady=2, sticky="w")
        tk.Entry(frm, textvariable=self.dwell_var,  width=8).grid(row=2, column=1, padx=6, pady=2, sticky="w")
        tk.Entry(frm, textvariable=self.cycles_var, width=8).grid(row=3, column=1, padx=6, pady=2, sticky="w")

        self.proto_lbl = tk.Label(frm, text=f"Protocol: {'INTEGER' if INTEGER_PROTOCOL else 'FLOAT'} / Invert: {INVERT_ANGLE}")
        self.proto_lbl.grid(row=0, column=2, padx=10, sticky="w", columnspan=2)

        # Buttons
        btn_frm = tk.Frame(root)
        btn_frm.pack(fill="x", padx=12, pady=8)

        self.btn_start = tk.Button(btn_frm, text="Start Test (±step around center)", command=self.start)
        self.btn_stop  = tk.Button(btn_frm, text="Stop", command=self.stop, state="disabled")
        self.btn_center = tk.Button(btn_frm, text="Go Center", command=self._go_center)

        self.btn_start.grid(row=0, column=0, padx=4)
        self.btn_stop.grid(row=0, column=1, padx=4)
        self.btn_center.grid(row=0, column=2, padx=4)

        # Footer note
        note = ("How to judge:\n"
                "  - If 0.5° commands cause NO movement → static friction/dead zone is large\n"
                "  - If overshoot or chatter occurs → backlash/hunting likely\n"
                "Tip: try INTEGER protocol first (firmware often expects integer degrees).")
        tk.Label(root, text=note, justify="left").pack(fill="x", padx=12, pady=(0, 6))

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _on_slider(self, value):
        try:
            send_angle(float(value))
        except ValueError:
            pass

    def _go_center(self):
        send_angle(self.center_var.get())
        self.slider.set(self.center_var.get())

    def start(self):
        if self.running:
            return
        self.running = True
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")

    def _run_loop(self):
        center = float(self.center_var.get())
        step   = float(self.step_var.get())
        dwell  = max(50, int(self.dwell_var.get()))  # ms
        cycles = int(self.cycles_var.get())

        k = 0
        # Sequence: center -> +step -> center -> -step -> center -> ...
        while self.running and (cycles == 0 or k < cycles):
            for target in (center, center + step, center, center - step):
                if not self.running:
                    break
                send_angle(target)
                self.slider.set(target)
                time.sleep(dwell / 1000.0)
            k += 1

        self.stop()

    def on_close(self):
        global ser
        self.stop()
        try:
            if ser and ser.is_open:
                ser.close()
                print("Serial closed.")
        except Exception as e:
            print(f"Error closing serial: {e}")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = BacklashTester(root)
    root.mainloop()

