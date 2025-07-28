import serial
import time
import tkinter as tk
from tkinter import messagebox

# === Serial Setup ===
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

ser = None
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print(f"Connected to {ARDUINO_PORT}")
except serial.SerialException as e:
    messagebox.showerror("Serial Error", f"Could not connect to {ARDUINO_PORT}:\n{e}")

# === Send angle command to Arduino ===
def send_angle(value):
    if ser and ser.is_open:
        try:
            angle = int(180 - float(value))
            ser.write(f"{angle}\n".encode())
            print(f"Sent angle: {angle}")
        except Exception as e:
            print(f"Failed to send: {e}")

# === Handle GUI close ===
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
root.title("SG-5010 Servo Control")
root.geometry("300x100")

slider = tk.Scale(root, from_=0, to=180, orient="horizontal", command=send_angle, label="Angle")
slider.set(90)
slider.pack(fill="x", padx=20, pady=20)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
