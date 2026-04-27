#THIS IS THE CODE TO IMPLEMENT INTO STP

import csv
import time
import math
import os
import random

#xbox controller support
try:
    import inputs
    gamepads = inputs.devices.gamepads
    if len(gamepads) == 0:
        print("No controller detected. Running in keyboard fallback mode.\n")
        CONTROLLER_AVAILABLE = False
    else:
        print(f"Controller detected: {gamepads[0]}")
        CONTROLLER_AVAILABLE = True
except ImportError:
    print("Warning: 'inputs' library not found. Run: pip install inputs")
    print("Running in keyboard fallback mode.\n")
    CONTROLLER_AVAILABLE = False

#gravity per environment in m/s^2
GRAVITY = {
    "earth": 9.81,
    "moon":  1.62,
    "mars":  3.72,
    #"agss" added below
}

#agss rotating frame stuff
AGSS_RADIUS  = 50.0
AGSS_OMEGA   = math.sqrt(9.81 / AGSS_RADIUS)
AGSS_GRAVITY    = AGSS_OMEGA ** 2 * AGSS_RADIUS
GRAVITY["agss"] = round(AGSS_GRAVITY, 4)
AGSS_SPIN_AXIS = (1.0, 0.0, 0.0)

#throw power
MIN_SPEED     = 3.0
MAX_SPEED     = 15.0
MAX_HOLD_TIME = 2.0

#right stick to angle mapping
MAX_ANGLE_V = 30.0
MAX_ANGLE_H = 15.0

#dartboard position
DARTBOARD_X = 2.37
DARTBOARD_Y = 1.73
DARTBOARD_Z = 0.0

#drift and noise
DRIFT_RATE     = 0.01
VELOCITY_NOISE = 0.2

#sim timestep
DT = 0.001

#output files
desktop = os.path.join(os.path.expanduser("~"), "Desktop")
os.makedirs(desktop, exist_ok=True)
CSV_FILE     = os.path.join(desktop, "throw_log_3d.csv")
SUMMARY_FILE = os.path.join(desktop, "trial_summary_3d.csv")


class ControllerState:
    def __init__(self):
        self.rt_value = 0.0
        self.rt_held = False
        self.rt_hold_start = None
        self.hold_duration = 0.0
        self.stick_rx = 0.0
        self.stick_ry = 0.0
        self.released = False

    def update(self):
        global CONTROLLER_AVAILABLE
        if not CONTROLLER_AVAILABLE:
            return False
        try:
            events = inputs.get_gamepad()
        except inputs.UnpluggedError:
            print("Controller disconnected! Switching to keyboard mode.")
            CONTROLLER_AVAILABLE = False
            return False
        except Exception:
            print("Controller error! Switching to keyboard mode.")
            CONTROLLER_AVAILABLE = False
            return False

        released = False
        for event in events:
            if event.code == "ABS_RZ":
                new_value = event.state / 255.0
                if new_value > 0.1 and not self.rt_held:
                    self.rt_held       = True
                    self.rt_hold_start = time.time()
                    print("  RT held - charging throw...")
                elif new_value <= 0.1 and self.rt_held:
                    self.rt_held = False
                    self.hold_duration = time.time() - self.rt_hold_start
                    released = True
                self.rt_value = new_value
            if event.code == "ABS_RX":
                self.stick_rx = event.state / 32767.0
            if event.code == "ABS_RY":
                self.stick_ry = -(event.state / 32767.0)
        return released


def map_hold_to_speed(hold_duration):
    t = min(hold_duration, MAX_HOLD_TIME)
    speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (t / MAX_HOLD_TIME)
    return round(speed, 3)


def map_stick_to_angles(stick_rx, stick_ry):
    deadzone = 0.1
    if abs(stick_rx) < deadzone:
        stick_rx = 0.0
    if abs(stick_ry) < deadzone:
        stick_ry = 0.0
    vertical_angle   = stick_ry * MAX_ANGLE_V
    horizontal_angle = stick_rx * MAX_ANGLE_H
    return round(vertical_angle, 2), round(horizontal_angle, 2)


def get_score(distance_from_center):
    if distance_from_center <= 0.006:
        return 50
    elif distance_from_center <= 0.016:
        return 25
    elif distance_from_center <= 0.107:
        return 10
    elif distance_from_center <= 0.170:
        return 5
    else:
        return 0


def cross(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )


def rotating_frame_accel(pos, vel, omega_vec):
    omega_cross_r = cross(omega_vec, pos)
    omega_cross_omega_cross_r = cross(omega_vec, omega_cross_r)
    ax_c = -omega_cross_omega_cross_r[0]
    ay_c = -omega_cross_omega_cross_r[1]
    az_c = -omega_cross_omega_cross_r[2]
    omega_cross_v = cross(omega_vec, vel)
    ax_cor = -2.0 * omega_cross_v[0]
    ay_cor = -2.0 * omega_cross_v[1]
    az_cor = -2.0 * omega_cross_v[2]
    return (ax_c + ax_cor, ay_c + ay_cor, az_c + az_cor)


def simulate_throw_3d(environment, speed, vertical_angle, horizontal_angle):
    g       = GRAVITY[environment]
    is_agss = (environment == "agss")
    omega_vec = tuple(AGSS_OMEGA * c for c in AGSS_SPIN_AXIS)

    v_rad = math.radians(vertical_angle)
    h_rad = math.radians(horizontal_angle)

    noise_x = random.uniform(-VELOCITY_NOISE, VELOCITY_NOISE)
    noise_y = random.uniform(-VELOCITY_NOISE, VELOCITY_NOISE)
    noise_z = random.uniform(-VELOCITY_NOISE, VELOCITY_NOISE)

    vx = speed * math.cos(v_rad) * math.cos(h_rad) + noise_x
    vy = speed * math.sin(v_rad)                    + noise_y
    vz = speed * math.cos(v_rad) * math.sin(h_rad) + noise_z

    x = 0.0
    y = 1.73
    z = 0.0

    trajectory   = []
    current_time = 0.0
    next_log     = 0.0

    while x < DARTBOARD_X:
        if current_time >= next_log:
            trajectory.append({
                "trial":        None,
                "time_s":       round(current_time, 4),
                "pos_x_m":      round(x, 4),
                "pos_y_m":      round(y, 4),
                "pos_z_m":      round(z, 4),
                "vel_x_ms":     round(vx, 4),
                "vel_y_ms":     round(vy, 4),
                "vel_z_ms":     round(vz, 4),
                "environment":  environment,
                "speed_ms":     speed,
                "v_angle_deg":  vertical_angle,
                "h_angle_deg":  horizontal_angle,
            })
            next_log += 0.01

        ax_total = 0.0
        ay_total = -g
        az_total = DRIFT_RATE

        if is_agss:
            r_y     = y - AGSS_RADIUS
            r_z     = z
            pos_rel = (0.0, r_y, r_z)
            vel_rel = (vx, vy, vz)
            pa_x, pa_y, pa_z = rotating_frame_accel(pos_rel, vel_rel, omega_vec)
            ax_total  = pa_x
            ay_total  = pa_y
            az_total += pa_z

        vx += ax_total * DT
        vy += ay_total * DT
        vz += az_total * DT
        x  += vx * DT
        y  += vy * DT
        z  += vz * DT
        current_time += DT

        if y < 0:
            break

    dist_from_center = math.sqrt(
        (y - DARTBOARD_Y) ** 2 + (z - DARTBOARD_Z) ** 2
    )
    score = get_score(dist_from_center)

    return {
        "landing_x":        round(x, 4),
        "landing_y":        round(y, 4),
        "landing_z":        round(z, 4),
        "dist_from_center": round(dist_from_center, 4),
        "score":            score,
        "flight_time":      round(current_time, 4),
        "trajectory":       trajectory,
    }


def export_csv(data, filename):
    if not data:
        return
    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=data[0].keys())
        writer.writeheader()
        writer.writerows(data)
    print(f"Saved: {os.path.abspath(filename)}")


def keyboard_throw():
    print("\n--- keyboard input ---")
    while True:
        try:
            hold = float(input("hold duration (0.0 - 2.0 seconds): ").strip())
            hold = max(0.0, min(hold, MAX_HOLD_TIME))
            break
        except ValueError:
            print("  invalid input, enter a number like 1.5")
    while True:
        try:
            rx = float(input("right stick x (-1.0 to 1.0, left/right aim): ").strip())
            rx = max(-1.0, min(rx, 1.0))
            break
        except ValueError:
            print("invalid input, enter a number like 0.0")
    while True:
        try:
            ry = float(input("right stick y (-1.0 to 1.0, up/down aim): ").strip())
            ry = max(-1.0, min(ry, 1.0))
            break
        except ValueError:
            print("  invalid input, enter a number like 0.5")
    return hold, rx, ry


def run_trial_manager():
    print("=" * 60)
    print("dart throw simulation - 3d + xbox controller")
    print("=" * 60)
    print(f"agss spin rate:    {AGSS_OMEGA:.4f} rad/s")
    print(f"agss radius:       {AGSS_RADIUS:.1f} m  (floor to spin axis)")
    print(f"agss eff. gravity: {GRAVITY['agss']:.4f} m/s^2")
    print()

    if CONTROLLER_AVAILABLE:
        print("controller detected!")
        print("hold RT to charge throw, release to throw.")
        print("right stick controls aim direction.")
    else:
        print("keyboard fallback mode active.")
        print("you'll be prompted to type values for each throw.")

    print("press ctrl+c to quit.\n")

    controller       = ControllerState()
    all_trajectories = []
    trial_results    = []
    trial_number     = 1
    environments     = ["earth", "moon", "mars", "agss"]
    env_index        = 0

    try:
        while True:
            environment = environments[env_index % len(environments)]
            print(f"\ntrial {trial_number} | environment: {environment.upper()} "
                  f"| gravity: {GRAVITY[environment]} m/s^2"
                  + (" (rotating frame - coriolis & centripetal active)"
                     if environment == "agss" else ""))

            if CONTROLLER_AVAILABLE:
                print("waiting for throw (hold and release RT)...")
                released = False
                while not released:
                    released = controller.update()
                    time.sleep(0.001)
                hold_duration = controller.hold_duration
                stick_rx      = controller.stick_rx
                stick_ry      = controller.stick_ry
            else:
                hold_duration, stick_rx, stick_ry = keyboard_throw()

            speed                            = map_hold_to_speed(hold_duration)
            vertical_angle, horizontal_angle = map_stick_to_angles(stick_rx, stick_ry)

            print(f"hold duration:    {round(hold_duration, 3)}s")
            print(f"throw speed:      {speed} m/s")
            print(f"vertical angle:   {vertical_angle} deg")
            print(f"horizontal angle: {horizontal_angle} deg")

            result = simulate_throw_3d(
                environment, speed, vertical_angle, horizontal_angle
            )

            for point in result["trajectory"]:
                point["trial"] = trial_number
                all_trajectories.append(point)

            trial_results.append({
                "trial":            trial_number,
                "environment":      environment,
                "gravity_ms2":      GRAVITY[environment],
                "agss_omega_rads":  AGSS_OMEGA if environment == "agss" else "N/A",
                "agss_radius_m":    AGSS_RADIUS if environment == "agss" else "N/A",
                "hold_duration_s":  round(hold_duration, 3),
                "throw_speed_ms":   speed,
                "v_angle_deg":      vertical_angle,
                "h_angle_deg":      horizontal_angle,
                "landing_x_m":      result["landing_x"],
                "landing_y_m":      result["landing_y"],
                "landing_z_m":      result["landing_z"],
                "dist_center_m":    result["dist_from_center"],
                "score":            result["score"],
                "flight_time_s":    result["flight_time"],
            })

            print(f"  landing pos:      ({result['landing_x']}, "
                  f"{result['landing_y']}, {result['landing_z']})")
            print(f"  dist from center: {result['dist_from_center']} m")
            print(f"  score:            {result['score']}")
            print(f"  flight time:      {result['flight_time']} s")

            trial_number += 1
            env_index    += 1

    except KeyboardInterrupt:
        print("\n\nquitting...")

    if all_trajectories:
        export_csv(all_trajectories, CSV_FILE)
        export_csv(trial_results, SUMMARY_FILE)

    if trial_results:
        print("\n" + "=" * 60)
        print("final summary")
        print("=" * 60)
        for t in trial_results:
            print(f"trial {t['trial']} | {t['environment'].upper()} | "
                  f"speed: {t['throw_speed_ms']} m/s | "
                  f"score: {t['score']} | "
                  f"dist: {t['dist_center_m']} m")


#################################################################
# DON'T CHANGE ANY OF THE BELOW; NECESSARY FOR JOINING SIMULATION
import os, sys, traceback
import spaceteams as st

def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
    exit(1)

sys.excepthook = custom_exception_handler

def before_init():
    pass  # Nothing needed during Sync/Init for this system

def before_runtime():
    st.logger_info("Dart throw simulation starting...")
    run_trial_manager()
    st.logger_info("Dart throw simulation complete.")

st.BeforeInit(before_init)
st.BeforeRuntime(before_runtime)
st.connect_to_sim(sys.argv)
import numpy as np
# DON'T CHANGE ANY OF THE ABOVE; NECESSARY FOR JOINING SIMULATION
#################################################################

st.leave_sim()
