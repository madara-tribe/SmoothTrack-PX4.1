#!/usr/bin/env python3
import cv2
import time
import argparse
from collections import deque
import numpy as np

# ---------- Blur metrics ----------
def var_laplacian(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

def tenengrad(gray):
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    tg = np.mean(gx*gx + gy*gy)
    return float(tg)

# ---------- Helper: robust exposure setting ----------
def set_manual_exposure(cap, exposure_s):
    """
    Try multiple backends:
    - Disable auto exposure (two common conventions)
    - Try CAP_PROP_EXPOSURE with "seconds" and with "log2 seconds (negative)"
    """
    # Disable auto exposure if possible
    # Linux(V4L2) often: 1 = manual, 3 = auto; some builds use 0.25 manual / 0.75 auto
    for v in [1, 0.25]:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, v)

    # Try to set exposure (best-effort across drivers)
    ok = False
    # Direct seconds (some cameras accept small positive values)
    ok |= cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_s))
    # log2 seconds negative (V4L2 style: e.g., 1/250s ≈ -7.97 → -8)
    try:
        log2s = np.log2(exposure_s)
        ok |= cap.set(cv2.CAP_PROP_EXPOSURE, float(log2s))
    except Exception:
        pass

    return ok

def human_exposure_str(exp_s):
    if exp_s <= 0:
        return "AUTO/UNKNOWN"
    return f"1/{int(round(1.0/exp_s))} s"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", type=int, default=2, help="/dev/video2")
    ap.add_argument("--w", type=int, default=960)
    ap.add_argument("--h", type=int, default=540)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--exposures", type=str, default="0.008333,0.004,0.002",  # 1/120, 1/250, 1/500
                    help="comma seconds (e.g. 0.008333,0.004,0.002)")
    args = ap.parse_args()

    exp_list = [float(x) for x in args.exposures.split(",")]
    exp_idx = 0
    target_exp = exp_list[exp_idx]

    cap = cv2.VideoCapture(args.dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.dev)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.dev}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    cap.set(cv2.CAP_PROP_FPS, 60)
    set_manual_exposure(cap, target_exp)

    # FPS measurement (sliding window 1s)
    times = deque(maxlen=120)
    lap_q = deque(maxlen=60)  # store recent blur metrics to build baseline
    tg_q  = deque(maxlen=60)

    warn_fps_under = 40.0
    warn_blur_ratio = 0.3  # current / baseline < 0.3 → blur warning

    font = cv2.FONT_HERSHEY_SIMPLEX

    last = time.perf_counter()
    while True:
        ok, frame = cap.read()
        now = time.perf_counter()
        if not ok or frame is None:
            # small sleep to avoid busy loop
            time.sleep(0.005)
            continue

        times.append(now)
        # Smooth FPS over ~1s
        fps = 0.0
        if len(times) >= 2:
            dt = times[-1] - times[0]
            if dt > 1e-6:
                fps = (len(times)-1) / dt

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lap = var_laplacian(gray)
        tg  = tenengrad(gray)
        lap_q.append(lap)
        tg_q.append(tg)

        # Baseline = top 20% median in recent window (robust to short drops)
        def robust_baseline(q):
            if not q:
                return 1.0
            arr = np.array(q)
            top = arr[np.argsort(arr)][int(0.8*len(arr)):]  # top 20%
            return float(np.median(top)) if len(top) else float(np.median(arr))

        lap_base = robust_baseline(lap_q)
        tg_base  = robust_baseline(tg_q)
        lap_ratio = (lap / lap_base) if lap_base > 0 else 1.0
        tg_ratio  = (tg / tg_base) if tg_base > 0 else 1.0

        # OSD text
        h, w = frame.shape[:2]
        y = 20
        def put(s, color=(255,255,255)):
            nonlocal y
            cv2.putText(frame, s, (10, y), font, 0.6, color, 2, cv2.LINE_AA)
            y += 24

        put(f"FPS(measured): {fps:5.1f}")
        put(f"Exposure: {human_exposure_str(target_exp)}  (AE: manual try)")
        put(f"Blur  LapVar: {lap:8.0f}  ratio:{lap_ratio:0.2f}")
        put(f"Blur  Tene  : {tg:10.0f}  ratio:{tg_ratio:0.2f}")

        # Warnings
        if fps < warn_fps_under:
            put(f"WARNING: FPS < {warn_fps_under:.0f} (tracking may stutter)", (0,0,255))
        if (lap_ratio < warn_blur_ratio) or (tg_ratio < warn_blur_ratio):
            put("WARNING: strong motion blur detected", (0,255,255))

        cv2.imshow("Test2: FPS & Exposure", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            break
        elif key == ord(' '):
            exp_idx = (exp_idx + 1) % len(exp_list)
            target_exp = exp_list[exp_idx]
            set_manual_exposure(cap, target_exp)
        elif key == ord('1'):
            target_exp = exp_list[0]; set_manual_exposure(cap, target_exp)
        elif key == ord('2') and len(exp_list) > 1:
            target_exp = exp_list[1]; set_manual_exposure(cap, target_exp)
        elif key == ord('3') and len(exp_list) > 2:
            target_exp = exp_list[2]; set_manual_exposure(cap, target_exp)
        elif key in (ord('a'), ord('A')):
            # Toggle auto exposure (best-effort)
            cur = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
            newv = 3 if (cur in (0.25, 1)) else 1  # try switch between auto and manual
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, newv)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

