"""
K230 Touch-Screen Red Tracking + Pan-Tilt (Standalone)
=====================================================
Touch UI with buttons for standalone operation without PC/IDE.

HOW TO RUN STANDALONE:
    1. Save this file as "main.py" on the K230 SD card root
    2. Power on K230 → auto-runs on boot
    3. No PC / IDE needed — everything on LCD touch screen

TOUCH BUTTONS (bottom bar):
    [追踪ON/OFF] [取色] [速度+] [速度-] [参数]

TOUCH CAMERA AREA:
    Tap anywhere on the video to sample that color as new tracking target.

Wiring: K230 PIN17(IO5)->STM32 PA10, PIN20(IO6)->STM32 PA9, GND->GND
"""

import time, os
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, UART, TOUCH

# ===================== CONFIG =====================
UART_TX  = 5
UART_RX  = 6
BAUD     = 128000

FRAME_W  = 800
FRAME_H  = 480

# Tracking params
DEAD_X     = 50
DEAD_Y     = 50
MAX_SPEED  = 350
MIN_SPEED  = 100
SEND_MS    = 150
SMOOTH_N   = 4

# Default red threshold (LAB)
RED_THRESHOLD = [(17, 59, 21, 71, -14, 59)]

# Predefined color thresholds for sampling (CanMV LAB format)
# Each: (name_display, [(L_min, L_max, A_min, A_max, B_min, B_max), ...])
COLOR_PRESETS = [
    ("Red",    [(17, 59,  21,  71,  -14,  59)]),
    ("Green",  [(30, 71, -52,  -8,   10,  54)]),
    ("Blue",   [(19, 55, -20,  33,  -63, -16)]),
    ("Yellow", [(53, 84, -17,  20,   36,  76)]),
    ("White",  [(75, 100, -20, 20,  -20,  20)]),
]
color_preset_idx = 0  # Start with Red

# Speed presets to cycle through
SPEED_PRESETS = [
    (100, 50),    # Slow
    (200, 80),    # Medium
    (350, 100),   # Fast (current default)
    (500, 150),   # Very fast
]

# ===================== TOUCH UI CONFIG =====================
BTN_Y      = 430          # Button bar top edge
BTN_H      = 50           # Button bar height
BTN_COLOR  = (70, 70, 70)  # Button background
BTN_ACTIVE = (0, 140, 0)   # Active button color
TEXT_WHITE = (255, 255, 255)
TEXT_BLACK = (0, 0, 0)

# Button layout: (x, width, label)
BUTTONS = [
    (0,   160, "ON"),
    (160, 160, "COLOR"),
    (320, 160, "SPD+"),
    (480, 160, "SPD-"),
    (640, 160, "INFO"),
]

# ===================== UART =====================
def send_speed(uart, sps_a, sps_b):
    def encode(sps):
        if sps == 0: return 0
        if sps > 0:
            v = sps // 4
            return max(min(v, 127), 1)
        else:
            v = (-sps) // 4
            return 256 - max(min(v, 127), 1)
    sa = encode(sps_a)
    sb = encode(sps_b)
    uart.write(bytes([0x7C, sa, sb, 0x7C ^ sa ^ sb, 0x7D]))

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ===================== TOUCH HELPERS =====================
def in_button(x, y):
    """Return button index if (x,y) is inside a button, else -1."""
    if y < BTN_Y or y > BTN_Y + BTN_H:
        return -1
    for i, (bx, bw, _) in enumerate(BUTTONS):
        if bx <= x < bx + bw:
            return i
    return -1

def draw_button_bar(img, tracking_on, speed_idx, show_info):
    """Draw the bottom button bar and status text."""
    # Button bar background
    img.draw_rectangle(0, BTN_Y, FRAME_W, BTN_H,
                       color=(50, 50, 50), thickness=-1, fill=True)

    # Divider line
    img.draw_line(0, BTN_Y, FRAME_W, BTN_Y,
                  color=(255, 255, 255), thickness=2)

    # Draw each button
    for i, (bx, bw, label) in enumerate(BUTTONS):
        if i == 0 and tracking_on:
            color = BTN_ACTIVE
        else:
            color = BTN_COLOR
        img.draw_rectangle(bx + 2, BTN_Y + 2, bw - 4, BTN_H - 4,
                           color=color, thickness=-1, fill=True)

        # Button label
        if i == 0:
            text = "TRACK" if tracking_on else "STOP"
        elif i == 1:
            # Show current color name
            text = COLOR_PRESETS[color_preset_idx][0][:5].upper()
        elif i == 2:
            text = "SPD+"
        elif i == 3:
            text = "SPD-"
        elif i == 4:
            text = "INFO"

        # Center text in button
        tx = bx + bw // 2 - 25
        ty = BTN_Y + 12
        if i == 0:
            c = (0, 255, 0) if tracking_on else (255, 100, 100)
        else:
            c = TEXT_WHITE
        img.draw_string_advanced(tx, ty, 22, text, color=c)

    # Info overlay (when INFO button pressed)
    if show_info:
        info_lines = [
            f"MAX:{MAX_SPEED} MIN:{MIN_SPEED}",
            f"DEAD:{DEAD_X}x{DEAD_Y} SEND:{SEND_MS}ms",
            f"THR:{len(THRESHOLDS)} preset(s)",
            "Tap INFO to close",
        ]
        # Background box
        img.draw_rectangle(10, BTN_Y - 130, 280, 125,
                           color=(0, 0, 0, 180), thickness=-1, fill=True)
        img.draw_rectangle(10, BTN_Y - 130, 280, 125,
                           color=(255, 255, 255), thickness=2)
        for j, line in enumerate(info_lines):
            img.draw_string_advanced(20, BTN_Y - 125 + j * 28, 22,
                                     line, color=TEXT_WHITE)

# ===================== COLOR SAMPLING =====================
def sample_color_from_roi(img, tx, ty, roi_size=30):
    """
    Try all predefined color thresholds on a small ROI around the tap point.
    Returns (threshold, color_name) of the best match, or (None, None).
    Uses only img.copy() and find_blobs() — no get_pixel() needed.
    """
    half = roi_size // 2
    x1 = max(0, tx - half)
    y1 = max(0, ty - half)
    x2 = min(FRAME_W, tx + half)
    y2 = min(FRAME_H, ty + half)

    # Ensure minimum size
    if x2 - x1 < 10 or y2 - y1 < 10:
        return None, None

    try:
        roi_img = img.copy(roi=(x1, y1, x2 - x1, y2 - y1))

        best_name = None
        best_threshold = None
        best_area = 0

        for name, thr in COLOR_PRESETS:
            blobs = roi_img.find_blobs(thr, merge=True,
                                       pixels_threshold=10, area_threshold=10)
            for b in blobs:
                area = b.w() * b.h()
                if area > best_area:
                    best_area = area
                    best_name = name
                    best_threshold = thr

        if best_threshold is not None and best_area > 5:
            return best_threshold, best_name
        return None, None
    except:
        return None, None

# ===================== MAIN =====================
sensor = None

try:
    print("K230 Touch Track — Standalone Mode")

    # ---- Init UART ----
    fpioa = FPIOA()
    fpioa.set_function(UART_TX, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, BAUD)

    # ---- Init Sensor ----
    sensor = Sensor(width=FRAME_W, height=FRAME_H)
    sensor.reset()
    sensor.set_framesize(width=FRAME_W, height=FRAME_H)
    sensor.set_pixformat(Sensor.RGB565)
    sensor.set_vflip(True)
    sensor.set_hmirror(True)

    # ---- Init Display ----
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # ---- Init Touch ----
    tp = TOUCH(0)

    # ---- State ----
    send_speed(uart, 0, 0)
    last_send = time.ticks_ms()
    hist_x = []
    hist_y = []

    tracking_on = True
    speed_idx = 2          # Start with preset 2: (350, 100)
    MAX_SPEED, MIN_SPEED = SPEED_PRESETS[speed_idx]
    THRESHOLDS = list(COLOR_PRESETS[color_preset_idx][1])  # Use preset thresholds
    show_info = False
    info_timer = 0

    # Touch state
    touch_debounce = 0
    last_touch_btn = -1
    touch_counter = 0       # Long-press counter for sample mode
    sample_mode = False
    sample_msg = ""
    sample_msg_timer = 0

    print("Touch UI ready. Buttons: [TRACK] [COLOR] [SPD+] [SPD-] [INFO]")
    print("Long-press video area (>0.5s) to auto-detect color.")
    print("Tap COLOR button to cycle: Red/Green/Blue/Yellow/White.")

    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # ==================== TOUCH HANDLING ====================
        if touch_debounce > 0:
            touch_debounce -= 1

        points = tp.read()
        if len(points) > 0 and touch_debounce == 0:
            tx = points[0].x
            ty = points[0].y
            btn = in_button(tx, ty)

            if btn >= 0:
                # ---- Button press ----
                touch_debounce = 15
                sample_mode = False  # Cancel sample mode on any button

                if btn == 0:  # [TRACK ON/OFF]
                    tracking_on = not tracking_on
                    if not tracking_on:
                        send_speed(uart, 0, 0)
                    print("Tracking:", "ON" if tracking_on else "OFF")

                elif btn == 1:  # [COLOR] — cycle color preset
                    color_preset_idx = (color_preset_idx + 1) % len(COLOR_PRESETS)
                    name, thr = COLOR_PRESETS[color_preset_idx]
                    THRESHOLDS = list(thr)
                    sample_msg = f"Color: {name}"
                    sample_msg_timer = 60
                    print(f"Switched to: {name}")

                elif btn == 2:  # [SPD+]
                    speed_idx = min(speed_idx + 1, len(SPEED_PRESETS) - 1)
                    MAX_SPEED, MIN_SPEED = SPEED_PRESETS[speed_idx]
                    sample_msg = f"Speed: MAX={MAX_SPEED} MIN={MIN_SPEED}"
                    sample_msg_timer = 60
                    print(f"Speed: MAX={MAX_SPEED} MIN={MIN_SPEED}")

                elif btn == 3:  # [SPD-]
                    speed_idx = max(speed_idx - 1, 0)
                    MAX_SPEED, MIN_SPEED = SPEED_PRESETS[speed_idx]
                    sample_msg = f"Speed: MAX={MAX_SPEED} MIN={MIN_SPEED}"
                    sample_msg_timer = 60
                    print(f"Speed: MAX={MAX_SPEED} MIN={MIN_SPEED}")

                elif btn == 4:  # [INFO]
                    show_info = not show_info

            else:
                # ---- Touch on video area (above button bar) ----
                sample_mode = True

            last_touch_btn = btn

        elif len(points) > 0 and sample_mode:
            # Finger is held down on video area — count for long-press
            tx = points[0].x
            ty = points[0].y
            if ty < BTN_Y:
                touch_counter += 1
                if touch_counter > 30:  # ~0.5 second hold
                    # Long press: auto-detect color
                    touch_counter = 0
                    sample_mode = False
                    touch_debounce = 20

                    thr, name = sample_color_from_roi(img, tx, ty)
                    if thr:
                        THRESHOLDS = thr
                        sample_msg = f"Detected: {name}!"
                        sample_msg_timer = 90
                        print(f"Auto-detected color: {name} -> {thr}")
                    else:
                        sample_msg = "No color found"
                        sample_msg_timer = 60
                        print("Sample failed: no color matched")
            else:
                touch_counter = 0
        else:
            # Finger released
            touch_counter = 0
            sample_mode = False

        # Message timer
        if sample_msg_timer > 0:
            sample_msg_timer -= 1
        else:
            sample_msg = ""

        # Info panel timer
        if info_timer > 0:
            info_timer -= 1

        # ==================== BLOB DETECTION ====================
        blobs = img.find_blobs(THRESHOLDS, merge=True,
                               pixels_threshold=200, area_threshold=200)

        best = None
        best_area = 0
        for b in blobs:
            area = b.w() * b.h()
            if area > best_area:
                best_area = area
                best = b

        # ==================== TRACKING LOGIC ====================
        if best and tracking_on:
            cx = best.x() + best.w() // 2
            cy = best.y() + best.h() // 2

            # Draw tracking visuals
            img.draw_rectangle(best.x(), best.y(), best.w(), best.h(),
                               color=(0, 255, 0), thickness=3)
            img.draw_cross(cx, cy, color=(255, 255, 255), size=10, thickness=2)
            img.draw_cross(FRAME_W // 2, FRAME_H // 2,
                           color=(255, 0, 0), size=15, thickness=2)

            # ---- Smoothing ----
            hist_x.append(cx)
            hist_y.append(cy)
            if len(hist_x) > SMOOTH_N:
                hist_x.pop(0)
                hist_y.pop(0)

            avg_cx = sum(hist_x) // len(hist_x)
            avg_cy = sum(hist_y) // len(hist_y)

            err_x = avg_cx - FRAME_W // 2
            err_y = avg_cy - FRAME_H // 2

            # ---- Compute speed ----
            sps_a = 0
            sps_b = 0

            if abs(err_x) > DEAD_X:
                ratio = clamp(abs(err_x) / (FRAME_W // 2), 0.0, 1.0)
                sps_a = int(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * ratio)
                if err_x > 0:
                    sps_a = -sps_a

            if abs(err_y) > DEAD_Y:
                ratio = clamp(abs(err_y) / (FRAME_H // 2), 0.0, 1.0)
                sps_b = int(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * ratio)
                if err_y < 0:
                    sps_b = -sps_b

            # ---- Send UART ----
            now = time.ticks_ms()
            if time.ticks_diff(now, last_send) > SEND_MS:
                last_send = now
                send_speed(uart, sps_a, sps_b)

            # Status text
            img.draw_string_advanced(5, 5, 22,
                "FPS:%.1f spd:(%d,%d)" % (clock.fps(), sps_a, sps_b),
                color=(255, 255, 255))

        elif best and not tracking_on:
            # Detected but tracking paused — draw blue rectangle
            img.draw_rectangle(best.x(), best.y(), best.w(), best.h(),
                               color=(0, 0, 255), thickness=2)
            img.draw_string_advanced(5, 5, 22,
                "TRACKING OFF", color=(255, 100, 100))

        else:
            # No target
            hist_x = []
            hist_y = []
            if tracking_on:
                send_speed(uart, 0, 0)
            img.draw_string_advanced(FRAME_W // 2 - 60, FRAME_H // 2, 30,
                                     "NO TARGET", color=(255, 0, 0))

        # ==================== DRAW UI OVERLAY ====================
        # Status messages
        if sample_mode:
            img.draw_string_advanced(FRAME_W // 2 - 130, BTN_Y - 30, 22,
                                     ">>> Hold finger on target <<<",
                                     color=(255, 255, 0))

        if sample_msg:
            img.draw_string_advanced(10, BTN_Y - 30, 20,
                                     sample_msg, color=(0, 255, 255))

        # Draw button bar
        draw_button_bar(img, tracking_on, speed_idx, show_info)

        # ==================== DISPLAY ====================
        img.compress_for_ide()
        Display.show_image(img)

except KeyboardInterrupt:
    send_speed(uart, 0, 0)
    print("User stopped")
except BaseException as e:
    import sys
    sys.print_exception(e)
finally:
    if sensor:
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
