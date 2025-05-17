import cv2
import numpy as np
import serial
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# —–––– Serial Setup —––––
ser = serial.Serial(port='COM5', baudrate=9600, timeout=1)
time.sleep(2)

# —–––– PID parameters —––––
Kp, Ki, Kd = 1.2, 0.30, 0.01
## adjust_kp = 0.05
sum_error = last_error = 0.0
last_d_error = 0.0

# —–––– Frame & Speed parameters —––––
frame_width = 640
frame_center = frame_width / 2
max_speed = 180
min_speed = -180

# —–––– Timing control —––––
last_time = time.perf_counter()

# —–––– Open camera —––––
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)
cap.set(cv2.CAP_PROP_CONTRAST, 50)
cap.set(cv2.CAP_PROP_SATURATION, 50)
cap.set(cv2.CAP_PROP_EXPOSURE, -4)

if not cap.isOpened():
    print("Error: could not open camera index 0")
    exit(1)

# Constants
START_BYTE = 0xFF

# Previous mask
prev_red_mask = None

# —–––– Global storage for plotting —––––
plot_data = {
    'error': [],
    'speed': [],
    'dt': [],
    'time': [],
    'cross_times': []
}
start_plot_time = time.time()

# —–––– Plotting function —––––
def live_plot():
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))
    fig.suptitle('Real-Time PID Signals')

    def animate(i):
        if len(plot_data['time']) < 2:
            return
        
        ax1.clear()
        ax2.clear()
        ax3.clear()

        ax1.plot(plot_data['time'], plot_data['error'], label='Error')
        ax2.plot(plot_data['time'], plot_data['speed'], label='Speed', color='g')
        ax3.plot(plot_data['time'], plot_data['dt'], label='dt (s)', color='r')

        ax1.legend()
        ax2.legend()
        ax3.legend()

        ax1.set_ylabel('Error (px)')
        ax2.set_ylabel('Speed')
        ax3.set_ylabel('dt (s)')
        ax3.set_xlabel('Time (s)')

        ax1.grid()
        ax2.grid()
        ax3.grid()

    ani = animation.FuncAnimation(fig, animate, interval=200, cache_frame_data=False)
    plt.show()

# —–––– Start the plotter thread —––––
threading.Thread(target=live_plot, daemon=True).start()

# —–––– RED DETECTION FUNCTION —––––
def detect_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 80, 80])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([150, 80, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((6, 6), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask_clean = np.zeros_like(red_mask)

    for cnt in contours:
        if cv2.contourArea(cnt) > 100:
            cv2.drawContours(mask_clean, [cnt], -1, 255, thickness=cv2.FILLED)

    return mask_clean, hsv

# —–––– HSV Debug Mouse Click Callback —––––
def mouse_callback(event, x, y, flags, param):
    global hsv_debug
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_pixel = hsv_debug[y, x]
        print(f"HSV at ({x},{y}) = {hsv_pixel}")

cv2.namedWindow('Original')
cv2.setMouseCallback('Original', mouse_callback)

# —–––– Main Loop —––––
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: frame not captured")
            break

        now = time.perf_counter()
        dt = now - last_time
        last_time = now

        # --- Draw center crosshair ---
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)
        cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)

        # --- Red Detection ---
        red_mask, hsv_debug = detect_red(frame)

        # --- Tracking Mask ---
        track_mask = red_mask

        # --- Centroid Calculation ---
        cols = track_mask.sum(axis=0)
        if cols.sum() == 0:
            searching = True
            column_location = frame_center  # Stay centered when no detection
        else:
            indices = np.arange(frame_width)
            column_location = (cols * indices).sum() / cols.sum()
            searching = False

        # Draw center tracking circle
        cv2.circle(frame, (int(column_location), cy), 5, (255, 0, 0), -1)

        # --- PID Control ---
        error = column_location - frame_center
        d_error = (error - last_error) / dt
        d_error = 0.7 * last_d_error + 0.3 * d_error
        sum_error += error * dt
        sum_error = max(min(sum_error, 1000), -1000)

        raw_speed = Kp * error + Ki * sum_error + Kd * d_error
        raw_speed = max(min(raw_speed, max_speed), min_speed)

        # --- Motor Command ---
        if searching:
            # --- Search sweeping mode ---
            sweep_speed = 40  # motor search speed
            sweep_period = 5.0  # seconds for full sweep
            t = time.time() - start_plot_time
            sweep_position = np.sin(2 * np.pi * (1 / sweep_period) * t)
            sweep_command_speed = int(sweep_speed * sweep_position)

            direction = 1 if sweep_command_speed >= 0 else 2
            speed_val = abs(sweep_command_speed)
            ser.write(bytes([START_BYTE, speed_val, direction]))
        else:
            # --- Normal tracking mode ---
            direction = 1 if raw_speed >= 0 else 2
            speed_val = int(abs(raw_speed))
            ser.write(bytes([START_BYTE, speed_val, direction]))

        # --- Data Storage ---
        elapsed_time = time.time() - start_plot_time
        plot_data['time'].append(elapsed_time)
        plot_data['error'].append(error)
        plot_data['speed'].append(raw_speed)
        plot_data['dt'].append(dt)

        if len(plot_data['time']) > 300:
            for k in plot_data.keys():
                if isinstance(plot_data[k], list):
                    plot_data[k] = plot_data[k][-300:]

        if len(plot_data['error']) >= 2:
            if (plot_data['error'][-2] > 0 and error <= 0) or (plot_data['error'][-2] < 0 and error >= 0):
                plot_data['cross_times'].append(elapsed_time)

        last_error = error
        last_d_error = d_error

        # --- Show Windows ---
        cv2.imshow('Original', frame)
        cv2.imshow('Red Mask', red_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break
        elif key == ord('u'):
            Kp += adjust_kp
            print(f"Kp increased to {Kp:.2f}")
        elif key == ord('d'):
            Kp = max(0, Kp - adjust_kp)
            print(f"Kp decreased to {Kp:.2f}")

finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()

    # Analyze Oscillation
    if len(plot_data['cross_times']) >= 6:
        periods = np.diff(plot_data['cross_times'][1:-1:2])
        Tu = periods.mean()
        print(f"\nDetected Oscillation Period Tu = {Tu:.2f} seconds")
        print(f"Ultimate Gain Ku = {Kp:.2f}")

        kp_final = 0.6 * Kp
        ki_final = 1.2 * Kp / Tu
        kd_final = 3 * Kp * Tu / 40

        print("\nRecommended PID parameters:")
        print(f"Kp = {kp_final:.2f}")
        print(f"Ki = {ki_final:.2f}")
        print(f"Kd = {kd_final:.2f}")
    else:
        print("\nNot enough oscillations detected for auto-tuning.")
