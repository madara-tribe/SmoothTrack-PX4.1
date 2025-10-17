#!/usr/bin/env python3
import serial
import time
import tkinter as tk
from tkinter import messagebox

# === Settings ===
ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200          # Arduino と一致
INVERT_ANGLE = True         # PC側で反転（Arduinoは反転しない）

ser = None

def open_serial():
    global ser
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Arduino の自動リセット待ち
        print(f"Connected to {ARDUINO_PORT} @ {BAUD_RATE}")
        # 起動時センター（90.0°）を送信して整合確認
        send_angle_value(90.0)
        slider.set(90.0)
    except serial.SerialException as e:
        messagebox.showerror("Serial Error", f"Could not connect to {ARDUINO_PORT}:\n{e}")
        ser = None

def send_line(text: str):
    if ser and ser.is_open:
        try:
            ser.write(text.encode("ascii"))
        except Exception as e:
            print(f"Failed to send: {e}")

def send_angle_value(angle_deg: float):
    """角度(度)を小数1桁で送信。改行必須。反転はPC側で実施。"""
    if INVERT_ANGLE:
        angle_deg = 180.0 - angle_deg
    # 安全に 0..180 に収める
    angle_deg = max(0.0, min(180.0, float(angle_deg)))
    send_line(f"{angle_deg:.1f}\n")
    print(f"Sent angle(deg): {angle_deg:.1f}")

def on_slider(value):
    try:
        send_angle_value(float(value))
    except ValueError:
        pass

def go_center():
    send_angle_value(90.0)
    slider.set(90.0)

def nudge(delta: float):
    try:
        cur = float(slider.get())
        slider.set(max(0.0, min(180.0, cur + delta)))
        send_angle_value(float(slider.get()))
    except Exception:
        pass

def on_close():
    global ser
    try:
        if ser and ser.is_open:
            ser.close()
            print("Serial closed.")
    except Exception as e:
        print(f"Error closing serial: {e}")
    root.destroy()

# === Tkinter GUI ===
root = tk.Tk()
root.title("SG-5010 Servo Calibrate (float)")
root.geometry("360x160")

# スライダ：0..180 を小数で操作（見た目は整数目盛でも OK）
slider = tk.Scale(root, from_=0.0, to=180.0, resolution=0.1,
                  orient="horizontal", command=on_slider, label="Angle (deg)")
slider.set(90.0)
slider.pack(fill="x", padx=16, pady=(10, 8))

btn_row = tk.Frame(root)
btn_row.pack(fill="x", padx=16, pady=(0, 8))

tk.Button(btn_row, text="◀ -1.0°", command=lambda: nudge(-1.0), width=8).pack(side="left", padx=4)
tk.Button(btn_row, text="Center", command=go_center, width=8).pack(side="left", padx=4)
tk.Button(btn_row, text="+1.0° ▶", command=lambda: nudge(+1.0), width=8).pack(side="left", padx=4)

root.protocol("WM_DELETE_WINDOW", on_close)

open_serial()
root.mainloop()

