import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons, TextBox

# -----------------------------
# VP configuration database
# -----------------------------
CONTROLLER_MODES = [
    "Classic PID",
    "Velocity Differential",
    "Adaptive Trim PID",
    "Deadband Potential",
    "Three State",
]

VP_CONFIGS = {
    "LBCC 2026": {
        "vehicle": {
            "mass": 1.076,
            "rho": 999.0,
            "neutral_volume_m3": 0.001077,
            "reference_area_m2": 0.0070,
            "drag_coefficient": 0.9,
            "buoyancy_engine_delta_m3": 19.6e-6,
            # Positive = too buoyant by this many grams at neutral actuator command.
            # Negative = too heavy by this many grams at neutral actuator command.
            "true_buoyancy_offset_g": 0.0,
        },
        "actuator": {
            "min_us": 500,
            "max_us": 2500,
            "neutral_us": 1500,
            "idle_us": 500,
            "direction": -1,
            "full_travel_time_s": 1.2,
        },
        "sensor": {
            "surface_pressure_mbar": 1013.25,
            "depth_noise_std_m": 0.002,
        },
        "pid": {
            "kp": 400.0,
            "ki": 0.0,
            "kd": 0.0,
            "interval_s": 0.10,
            "integral_min": -2.0,
            "integral_max": 2.0,
        },
        "controller": {
            "mode": "Adaptive Trim PID",
            "kp": 450.0,
            "ki": 25.0,
            "kd": 1200.0,
            "deadband_m": 0.05,
            "output_limit_us": 700.0,
            "trim_limit_us": 75.0,
            "trim_velocity_max_mps": 0.025,
            "velocity_alpha": 0.25,
            "deeper_command_us": 1850.0,
            "shallower_command_us": 1150.0,
        },
        "mission": {
            "deep_m": 2.50,
            "shallow_m": 0.40,
            "surface_m": 0.02,
            "hold_s": 30.0,
            "tol_m": 0.05,
            "surface_tol_m": 0.05,
            "max_transit_s": 90.0,
            "max_hold_s": 60.0,
            "max_recover_s": 90.0,
            "self_recover": True,
        },
    },

    "Rays 2026": {
        "vehicle": {
            "mass": 1.2,
            "rho": 1023.0,
            "neutral_volume_m3": 0.001173,
            "reference_area_m2": 0.0070,
            "drag_coefficient": 0.9,
            "buoyancy_engine_delta_m3": 19.6e-6,
            "true_buoyancy_offset_g": 0.0,
        },
        "actuator": {
            "min_us": 500,
            "max_us": 2500,
            "neutral_us": 1500,
            "idle_us": 500,
            "direction": -1,
            "full_travel_time_s": 1.2,
        },
        "sensor": {
            "surface_pressure_mbar": 1013.25,
            "depth_noise_std_m": 0.002,
        },
        "pid": {
            "kp": 700.0,
            "ki": 0.0,
            "kd": 0.0,
            "interval_s": 0.10,
            "integral_min": -2.0,
            "integral_max": 2.0,
        },
        "controller": {
            "mode": "Adaptive Trim PID",
            "kp": 300.0,
            "ki": 20.0,
            "kd": 1500.0,
            "deadband_m": 0.03,
            "output_limit_us": 650.0,
            "trim_limit_us": 75.0,
            "trim_velocity_max_mps": 0.025,
            "velocity_alpha": 0.25,
            "deeper_command_us": 1800.0,
            "shallower_command_us": 1200.0,
        },
        "mission": {
            "deep_m": 0.60,
            "shallow_m": 0.40,
            "surface_m": 0.02,
            "hold_s": 30.0,
            "tol_m": 0.05,
            "surface_tol_m": 0.05,
            "max_transit_s": 90.0,
            "max_hold_s": 60.0,
            "max_recover_s": 90.0,
            "self_recover": True,
        },
    },

    "Rays 2024": {
        "vehicle": {
            "mass": 1.8,
            "rho": 997.0,
            "neutral_volume_m3": 0.001805,
            "reference_area_m2": 0.0080,
            "drag_coefficient": 0.8,
            "buoyancy_engine_delta_m3": 26.0e-6,
            "true_buoyancy_offset_g": 0.0,
        },
        "actuator": {
            "min_us": 1000,
            "max_us": 1300,
            "neutral_us": 1150,
            "idle_us": 1000,
            "direction": -1,
            "full_travel_time_s": 1.5,
        },
        "sensor": {
            "surface_pressure_mbar": 1026.25,
            "depth_noise_std_m": 0.002,
        },
        "pid": {
            "kp": 0.3,
            "ki": 0.0,
            "kd": 0.0,
            "interval_s": 0.10,
            "integral_min": -2.0,
            "integral_max": 2.0,
        },
        "controller": {
            "mode": "Adaptive Trim PID",
            "kp": 45.0,
            "ki": 8.0,
            "kd": 500.0,
            "deadband_m": 0.06,
            "output_limit_us": 120.0,
            "trim_limit_us": 30.0,
            "trim_velocity_max_mps": 0.025,
            "velocity_alpha": 0.25,
            "deeper_command_us": 1240.0,
            "shallower_command_us": 1060.0,
        },
        "mission": {
            "deep_m": 2.5,
            "shallow_m": 0.0,
            "surface_m": 0.05,
            "hold_s": 30.0,
            "tol_m": 0.25,
            "surface_tol_m": 0.10,
            "max_transit_s": 120.0,
            "max_hold_s": 60.0,
            "max_recover_s": 120.0,
            "self_recover": False,
        },
    },
}

SELECTED_VP = "LBCC 2026"


class VPSim:
    IDLE = 0
    DESCEND_1 = 1
    HOLD_DEEP_1 = 2
    ASCEND_1 = 3
    HOLD_SHALLOW_1 = 4
    DESCEND_2 = 5
    HOLD_DEEP_2 = 6
    ASCEND_2 = 7
    HOLD_SHALLOW_2 = 8
    STATION_KEEP = 9
    RECOVER_SURFACE = 10
    DONE = 11

    STATE_NAMES = {
        IDLE: "IDLE",
        DESCEND_1: "DESCEND_1",
        HOLD_DEEP_1: "HOLD_DEEP_1",
        ASCEND_1: "ASCEND_1",
        HOLD_SHALLOW_1: "HOLD_SHALLOW_1",
        DESCEND_2: "DESCEND_2",
        HOLD_DEEP_2: "HOLD_DEEP_2",
        ASCEND_2: "ASCEND_2",
        HOLD_SHALLOW_2: "HOLD_SHALLOW_2",
        STATION_KEEP: "STATION_KEEP",
        RECOVER_SURFACE: "RECOVER_SURFACE",
        DONE: "DONE",
    }

    def __init__(self, cfg_name: str):
        self.cfg_name = cfg_name
        self.cfg = VP_CONFIGS[cfg_name]
        self.rng = np.random.default_rng(42)
        self.reset()

    def load_config(self, cfg_name: str):
        self.cfg_name = cfg_name
        self.cfg = VP_CONFIGS[cfg_name]
        self.reset()

    def apply_settings(
        self,
        deep_m,
        shallow_m,
        surface_m,
        hold_s,
        tol_m,
        surface_tol_m,
        true_buoyancy_offset_g,
        engine_ml,
        kp,
        ki,
        kd,
        deadband_m,
        output_limit_us,
        trim_limit_us,
        trim_velocity_max_mps,
        velocity_alpha,
        deeper_command_us,
        shallower_command_us,
    ):
        self.cfg["mission"]["deep_m"] = deep_m
        self.cfg["mission"]["shallow_m"] = shallow_m
        self.cfg["mission"]["surface_m"] = surface_m
        self.cfg["mission"]["hold_s"] = hold_s
        self.cfg["mission"]["tol_m"] = tol_m
        self.cfg["mission"]["surface_tol_m"] = surface_tol_m

        self.cfg["vehicle"]["true_buoyancy_offset_g"] = true_buoyancy_offset_g
        self.cfg["vehicle"]["buoyancy_engine_delta_m3"] = engine_ml * 1.0e-6

        # Keep the old pid fields useful for Classic PID, but the shared
        # controller fields are what the new modes use.
        self.cfg["pid"]["kp"] = kp
        self.cfg["pid"]["ki"] = ki
        self.cfg["pid"]["kd"] = kd

        self.cfg["controller"]["kp"] = kp
        self.cfg["controller"]["ki"] = ki
        self.cfg["controller"]["kd"] = kd
        self.cfg["controller"]["deadband_m"] = max(0.0, deadband_m)
        self.cfg["controller"]["output_limit_us"] = max(1.0, output_limit_us)
        self.cfg["controller"]["trim_limit_us"] = max(0.0, trim_limit_us)
        self.cfg["controller"]["trim_velocity_max_mps"] = max(0.0, trim_velocity_max_mps)
        self.cfg["controller"]["velocity_alpha"] = self.clamp(velocity_alpha, 0.01, 1.0)
        self.cfg["controller"]["deeper_command_us"] = deeper_command_us
        self.cfg["controller"]["shallower_command_us"] = shallower_command_us

        self.reset()

    def set_controller_mode(self, mode: str):
        if mode not in CONTROLLER_MODES:
            return
        self.cfg["controller"]["mode"] = mode
        self.controller_mode = mode
        self.reset_controller_memory(reset_trim=True)

    def reset(self):
        cfg = self.cfg

        self.mass = cfg["vehicle"]["mass"]
        self.rho = cfg["vehicle"]["rho"]
        self.g = 9.81
        self.neutral_volume_m3 = cfg["vehicle"]["neutral_volume_m3"]
        self.reference_area_m2 = cfg["vehicle"]["reference_area_m2"]
        self.drag_coefficient = cfg["vehicle"]["drag_coefficient"]
        self.buoyancy_engine_delta_m3 = cfg["vehicle"]["buoyancy_engine_delta_m3"]
        self.true_buoyancy_offset_g = cfg["vehicle"].get("true_buoyancy_offset_g", 0.0)

        self.weight_force = self.mass * self.g
        self.neutral_buoyancy_force = self.rho * self.g * self.neutral_volume_m3
        self.max_buoyancy_delta = self.rho * self.g * (self.buoyancy_engine_delta_m3 / 2.0)

        self.act_min = cfg["actuator"]["min_us"]
        self.act_max = cfg["actuator"]["max_us"]
        self.act_neutral = cfg["actuator"]["neutral_us"]
        self.act_idle = cfg["actuator"]["idle_us"]
        self.act_direction = cfg["actuator"]["direction"]

        self.surface_pressure_mbar = cfg["sensor"]["surface_pressure_mbar"]
        self.depth_noise_std_m = cfg["sensor"]["depth_noise_std_m"]

        self.kp = cfg["controller"].get("kp", cfg["pid"]["kp"])
        self.ki = cfg["controller"].get("ki", cfg["pid"]["ki"])
        self.kd = cfg["controller"].get("kd", cfg["pid"]["kd"])
        self.pid_interval = cfg["pid"]["interval_s"]
        self.pid_integral_min = cfg["pid"].get("integral_min", -2.0)
        self.pid_integral_max = cfg["pid"].get("integral_max", 2.0)

        self.controller_mode = cfg["controller"].get("mode", "Classic PID")
        self.deadband_m = cfg["controller"].get("deadband_m", 0.05)
        self.output_limit_us = cfg["controller"].get("output_limit_us", 700.0)
        self.trim_limit_us = cfg["controller"].get("trim_limit_us", 75.0)
        self.trim_velocity_max_mps = cfg["controller"].get("trim_velocity_max_mps", 0.025)
        self.velocity_alpha = cfg["controller"].get("velocity_alpha", 0.25)
        self.deeper_command_us = cfg["controller"].get("deeper_command_us", self.act_neutral + 300.0)
        self.shallower_command_us = cfg["controller"].get("shallower_command_us", self.act_neutral - 300.0)

        self.deep_m = cfg["mission"]["deep_m"]
        self.shallow_m = cfg["mission"]["shallow_m"]
        self.surface_m = cfg["mission"]["surface_m"]
        self.hold_s = cfg["mission"]["hold_s"]
        self.tol_m = cfg["mission"]["tol_m"]
        self.surface_tol_m = cfg["mission"]["surface_tol_m"]

        self.max_transit_s = cfg["mission"].get("max_transit_s", 90.0)
        self.max_hold_s = cfg["mission"].get("max_hold_s", 60.0)
        self.max_recover_s = cfg["mission"].get("max_recover_s", 90.0)
        self.self_recover = cfg["mission"].get("self_recover", True)

        self.t = 0.0
        self.true_depth = 0.0
        self.true_velocity = 0.0
        self.true_accel = 0.0
        self.measured_depth = 0.0
        self.actuator_us = self.act_idle

        self.running = False

        self.current_state = self.DESCEND_1
        self.state_entry_time = 0.0
        self.in_tolerance_start = None
        self.current_target_depth = self.deep_m
        self.current_target_tol = self.tol_m

        self.reset_controller_memory(reset_trim=True)

        self.time_hist = []
        self.depth_hist = []
        self.measured_depth_hist = []
        self.target_hist = []
        self.velocity_hist = []
        self.filtered_velocity_hist = []
        self.actuator_hist = []
        self.trim_hist = []
        self.state_hist = []
        self.hold_hist = []

    def reset_controller_memory(self, reset_trim=False):
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_timer = 0.0
        self.filtered_velocity = 0.0
        self.last_controller_depth = self.measured_depth
        if reset_trim:
            self.adaptive_trim_us = 0.0

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def sign(self, x):
        if x > 0:
            return 1.0
        if x < 0:
            return -1.0
        return 0.0

    def actuator_us_to_command(self, us):
        half_range = (self.act_max - self.act_min) / 2.0
        centered = us - self.act_neutral
        cmd = centered / half_range if half_range > 0 else 0.0
        cmd = self.clamp(cmd, -1.0, 1.0)
        return self.act_direction * cmd

    def depth_output_to_actuator_us(self, output_us):
        """
        output_us is positive when the controller wants the VP to go deeper.
        act_direction converts that to the correct actuator polarity.
        For the LBCC/Rays direction = -1, positive output becomes higher microseconds.
        """
        us = self.act_neutral - self.act_direction * output_us
        return self.clamp(us, self.act_min, self.act_max)

    def update_velocity_filter(self, control_dt):
        if control_dt <= 0:
            return self.filtered_velocity
        raw_velocity = (self.measured_depth - self.last_controller_depth) / control_dt
        self.last_controller_depth = self.measured_depth
        self.filtered_velocity += self.velocity_alpha * (raw_velocity - self.filtered_velocity)
        return self.filtered_velocity

    def state_name(self):
        return self.STATE_NAMES[self.current_state]

    def state_has_target(self):
        return self.current_state in {
            self.DESCEND_1,
            self.HOLD_DEEP_1,
            self.ASCEND_1,
            self.HOLD_SHALLOW_1,
            self.DESCEND_2,
            self.HOLD_DEEP_2,
            self.ASCEND_2,
            self.HOLD_SHALLOW_2,
            self.STATION_KEEP,
            self.RECOVER_SURFACE,
        }

    def get_target_for_state(self, state):
        if state in {self.DESCEND_1, self.HOLD_DEEP_1, self.DESCEND_2, self.HOLD_DEEP_2}:
            return self.deep_m, self.tol_m
        if state in {self.ASCEND_1, self.HOLD_SHALLOW_1, self.ASCEND_2, self.HOLD_SHALLOW_2, self.STATION_KEEP}:
            return self.shallow_m, self.tol_m
        if state == self.RECOVER_SURFACE:
            return self.surface_m, self.surface_tol_m
        return 0.0, self.tol_m

    def enter_state(self, new_state):
        self.current_state = new_state
        self.state_entry_time = self.t
        self.in_tolerance_start = None
        self.reset_controller_memory(reset_trim=False)
        self.current_target_depth, self.current_target_tol = self.get_target_for_state(new_state)

    def state_time(self):
        return self.t - self.state_entry_time

    def in_tolerance(self):
        return abs(self.measured_depth - self.current_target_depth) <= self.current_target_tol

    def hold_elapsed(self):
        if self.in_tolerance_start is None:
            return 0.0
        return self.t - self.in_tolerance_start

    def run_classic_pid(self, error, control_dt):
        self.pid_integral += error * control_dt
        self.pid_integral = self.clamp(self.pid_integral, self.pid_integral_min, self.pid_integral_max)

        derivative = (error - self.pid_prev_error) / control_dt if control_dt > 0 else 0.0
        self.pid_prev_error = error

        output_us = self.kp * error + self.ki * self.pid_integral + self.kd * derivative
        output_us = self.clamp(output_us, -self.output_limit_us, self.output_limit_us)
        return self.depth_output_to_actuator_us(output_us)

    def run_velocity_differential(self, error, control_dt):
        vel = self.update_velocity_filter(control_dt)

        self.pid_integral += error * control_dt
        self.pid_integral = self.clamp(self.pid_integral, self.pid_integral_min, self.pid_integral_max)

        # Positive velocity means moving deeper. Subtracting velocity slows descent and fights overshoot.
        output_us = self.kp * error + self.ki * self.pid_integral - self.kd * vel
        output_us = self.clamp(output_us, -self.output_limit_us, self.output_limit_us)
        return self.depth_output_to_actuator_us(output_us)

    def run_adaptive_trim_pid(self, error, control_dt):
        vel = self.update_velocity_filter(control_dt)

        # The trim only learns when the VP is not near target and is not moving much.
        # This lets 1500 us remain the desired mechanical neutral, while the sim can
        # adapt if the real buoyancy is slightly off.
        if abs(error) > self.deadband_m and abs(vel) < self.trim_velocity_max_mps:
            self.adaptive_trim_us += self.ki * error * control_dt
            self.adaptive_trim_us = self.clamp(self.adaptive_trim_us, -self.trim_limit_us, self.trim_limit_us)

        control_error = 0.0 if abs(error) < self.deadband_m else error
        output_us = self.adaptive_trim_us + self.kp * control_error - self.kd * vel
        output_us = self.clamp(output_us, -self.output_limit_us, self.output_limit_us)
        return self.depth_output_to_actuator_us(output_us)

    def run_deadband_potential(self, error, control_dt):
        vel = self.update_velocity_filter(control_dt)

        # Inside the deadband, the controller outputs neutral and does not build integral.
        if abs(error) <= self.deadband_m:
            self.pid_integral = 0.0
            return self.depth_output_to_actuator_us(0.0)

        # Outside the deadband, only the distance beyond the deadband is controlled.
        # This avoids a step in output right at the edge of the window.
        effective_error = self.sign(error) * (abs(error) - self.deadband_m)

        self.pid_integral += effective_error * control_dt
        self.pid_integral = self.clamp(self.pid_integral, self.pid_integral_min, self.pid_integral_max)

        output_us = self.kp * effective_error + self.ki * self.pid_integral - self.kd * vel
        output_us = self.clamp(output_us, -self.output_limit_us, self.output_limit_us)
        return self.depth_output_to_actuator_us(output_us)

    def run_three_state(self, error):
        # Positive error means target is deeper than measured depth.
        # VP is above/shallower than target: send the deeper command.
        if error > self.deadband_m:
            return self.clamp(self.deeper_command_us, self.act_min, self.act_max)

        # VP is below/deeper than target: send the shallower command.
        if error < -self.deadband_m:
            return self.clamp(self.shallower_command_us, self.act_min, self.act_max)

        # Inside the window: do nothing, command neutral.
        return self.act_neutral

    def run_controller(self, control_dt):
        error = self.current_target_depth - self.measured_depth
        mode = self.controller_mode

        if mode == "Classic PID":
            return self.run_classic_pid(error, control_dt)
        if mode == "Velocity Differential":
            return self.run_velocity_differential(error, control_dt)
        if mode == "Adaptive Trim PID":
            return self.run_adaptive_trim_pid(error, control_dt)
        if mode == "Deadband Potential":
            return self.run_deadband_potential(error, control_dt)
        if mode == "Three State":
            return self.run_three_state(error)

        return self.depth_output_to_actuator_us(0.0)

    def step(self, dt):
        if not self.running:
            return

        self.t += dt

        # sensor
        self.measured_depth = self.true_depth + self.rng.normal(0.0, self.depth_noise_std_m)

        # update tolerance timer
        if self.state_has_target() and self.current_state in {
            self.HOLD_DEEP_1, self.HOLD_SHALLOW_1, self.HOLD_DEEP_2, self.HOLD_SHALLOW_2
        }:
            if self.in_tolerance():
                if self.in_tolerance_start is None:
                    self.in_tolerance_start = self.t
            else:
                self.in_tolerance_start = None

        # controller
        self.current_target_depth, self.current_target_tol = self.get_target_for_state(self.current_state)

        if self.current_state in {self.IDLE, self.DONE}:
            self.actuator_us = self.act_idle
        else:
            self.pid_timer += dt
            if self.pid_timer >= self.pid_interval:
                control_dt = self.pid_timer
                self.pid_timer = 0.0
                self.actuator_us = self.run_controller(control_dt)

        # mission state machine
        st = self.current_state
        timed_out_transit = self.state_time() >= self.max_transit_s
        timed_out_hold = self.state_time() >= self.max_hold_s
        timed_out_recover = self.state_time() >= self.max_recover_s

        if st == self.DESCEND_1:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_DEEP_1)

        elif st == self.HOLD_DEEP_1:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                self.enter_state(self.ASCEND_1)

        elif st == self.ASCEND_1:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_SHALLOW_1)

        elif st == self.HOLD_SHALLOW_1:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                self.enter_state(self.DESCEND_2)

        elif st == self.DESCEND_2:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_DEEP_2)

        elif st == self.HOLD_DEEP_2:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                self.enter_state(self.ASCEND_2)

        elif st == self.ASCEND_2:
            if self.in_tolerance() or timed_out_transit:
                self.enter_state(self.HOLD_SHALLOW_2)

        elif st == self.HOLD_SHALLOW_2:
            if self.hold_elapsed() >= self.hold_s or timed_out_hold:
                if self.self_recover:
                    self.enter_state(self.RECOVER_SURFACE)
                else:
                    self.enter_state(self.STATION_KEEP)

        elif st == self.STATION_KEEP:
            pass

        elif st == self.RECOVER_SURFACE:
            if self.in_tolerance() or timed_out_recover:
                self.enter_state(self.DONE)
                self.actuator_us = self.act_idle

        # physics
        u = self.actuator_us_to_command(self.actuator_us)

        buoyancy_offset_force = (self.true_buoyancy_offset_g / 1000.0) * self.g
        buoyancy_force = self.neutral_buoyancy_force + u * self.max_buoyancy_delta + buoyancy_offset_force
        weight_force = self.weight_force

        drag_force = (
            -0.5
            * self.rho
            * self.drag_coefficient
            * self.reference_area_m2
            * self.true_velocity
            * abs(self.true_velocity)
        )

        net_force = weight_force - buoyancy_force + drag_force
        self.true_accel = net_force / self.mass
        self.true_velocity += self.true_accel * dt
        self.true_depth += self.true_velocity * dt

        if self.true_depth < 0.0:
            self.true_depth = 0.0
            self.true_velocity = 0.0

        # log
        self.time_hist.append(self.t)
        self.depth_hist.append(self.true_depth)
        self.measured_depth_hist.append(self.measured_depth)
        self.target_hist.append(self.current_target_depth if self.state_has_target() else 0.0)
        self.velocity_hist.append(self.true_velocity)
        self.filtered_velocity_hist.append(self.filtered_velocity)
        self.actuator_hist.append(self.actuator_us)
        self.trim_hist.append(self.adaptive_trim_us)
        self.state_hist.append(self.current_state)
        self.hold_hist.append(self.hold_elapsed())

        max_points = 1500
        if len(self.time_hist) > max_points:
            self.time_hist = self.time_hist[-max_points:]
            self.depth_hist = self.depth_hist[-max_points:]
            self.measured_depth_hist = self.measured_depth_hist[-max_points:]
            self.target_hist = self.target_hist[-max_points:]
            self.velocity_hist = self.velocity_hist[-max_points:]
            self.filtered_velocity_hist = self.filtered_velocity_hist[-max_points:]
            self.actuator_hist = self.actuator_hist[-max_points:]
            self.trim_hist = self.trim_hist[-max_points:]
            self.state_hist = self.state_hist[-max_points:]
            self.hold_hist = self.hold_hist[-max_points:]


sim = VPSim(SELECTED_VP)

# -----------------------------
# Figure layout
# -----------------------------
fig = plt.figure(figsize=(16, 9))

# Leave space on the left for controls and on the bottom for buttons
gs = fig.add_gridspec(
    2, 3,
    left=0.31, right=0.98, top=0.92, bottom=0.12,
    width_ratios=[1.2, 1.2, 0.9],
    height_ratios=[1, 1],
    wspace=0.30, hspace=0.30
)

ax_depth_plot = fig.add_subplot(gs[0, 0:2])
ax_act_plot = fig.add_subplot(gs[1, 0:2])
ax_depth_graphic = fig.add_subplot(gs[:, 2])

fig.suptitle(f"VP Simulator - {SELECTED_VP}", fontsize=14)

# Depth plot
line_true_depth, = ax_depth_plot.plot([], [], label="True Depth (m)")
line_meas_depth, = ax_depth_plot.plot([], [], label="Measured Depth (m)")
line_target_depth, = ax_depth_plot.plot([], [], "--", label="Target Depth (m)")
ax_depth_plot.set_xlabel("Time (s)")
ax_depth_plot.set_ylabel("Depth (m)")
ax_depth_plot.invert_yaxis()
ax_depth_plot.grid(True)
ax_depth_plot.legend()

# Actuator plot
line_actuator, = ax_act_plot.plot([], [], label="Actuator (us)")
line_trim, = ax_act_plot.plot([], [], "--", label="Adaptive Trim (us offset)")
neutral_line = ax_act_plot.axhline(sim.act_neutral, linestyle="--", label="Neutral")
ax_act_plot.set_xlabel("Time (s)")
ax_act_plot.set_ylabel("Actuator / Trim")
ax_act_plot.grid(True)
ax_act_plot.legend()

# Depth graphic
ax_depth_graphic.set_title("Depth View")
ax_depth_graphic.set_xlim(0, 1)
ax_depth_graphic.set_ylim(3.0, 0.0)
ax_depth_graphic.set_xticks([])
ax_depth_graphic.set_ylabel("Depth (m)")
ax_depth_graphic.grid(True, axis="y")

# Draw water column
ax_depth_graphic.fill_between([0.25, 0.75], [0, 0], [3.0, 3.0], alpha=0.15)

current_marker, = ax_depth_graphic.plot([0.5], [0.0], marker="o", markersize=10, linestyle="None", label="VP")
target_marker, = ax_depth_graphic.plot([0.5], [sim.current_target_depth], marker="x", markersize=10, linestyle="None", label="Target")
tol_top = sim.current_target_depth - sim.current_target_tol
tol_bot = sim.current_target_depth + sim.current_target_tol
tol_band = ax_depth_graphic.fill_between([0.35, 0.65], [tol_top, tol_top], [tol_bot, tol_bot], alpha=0.25)
ax_depth_graphic.legend(loc="upper right")

# Status text
status_text = ax_depth_graphic.text(
    0.05, 0.98, "",
    transform=ax_depth_graphic.transAxes,
    va="top",
    ha="left",
    fontsize=9,
    family="monospace"
)

# -----------------------------
# Controls
# -----------------------------
ax_radio_vp = fig.add_axes([0.02, 0.76, 0.125, 0.17])
radio_vp = RadioButtons(
    ax_radio_vp,
    labels=list(VP_CONFIGS.keys()),
    active=list(VP_CONFIGS.keys()).index(SELECTED_VP)
)
ax_radio_vp.set_title("VP Select", fontsize=10)

ax_radio_controller = fig.add_axes([0.16, 0.70, 0.13, 0.23])
radio_controller = RadioButtons(
    ax_radio_controller,
    labels=CONTROLLER_MODES,
    active=CONTROLLER_MODES.index(sim.controller_mode)
)
ax_radio_controller.set_title("Controller", fontsize=10)

# Buttons
ax_btn_start = fig.add_axes([0.02, 0.04, 0.08, 0.045])
ax_btn_pause = fig.add_axes([0.11, 0.04, 0.08, 0.045])
ax_btn_reset = fig.add_axes([0.20, 0.04, 0.08, 0.045])
ax_btn_apply = fig.add_axes([0.02, 0.095, 0.10, 0.04])

btn_start = Button(ax_btn_start, "Start")
btn_pause = Button(ax_btn_pause, "Pause")
btn_reset = Button(ax_btn_reset, "Reset")
btn_apply = Button(ax_btn_apply, "Apply")

# Mission card
fig.text(0.02, 0.665, "Mission", fontsize=10, weight="bold")

ax_box_deep = fig.add_axes([0.02, 0.625, 0.105, 0.03])
ax_box_shallow = fig.add_axes([0.02, 0.585, 0.105, 0.03])
ax_box_surface = fig.add_axes([0.02, 0.545, 0.105, 0.03])
ax_box_hold = fig.add_axes([0.02, 0.505, 0.105, 0.03])
ax_box_tol = fig.add_axes([0.02, 0.465, 0.105, 0.03])
ax_box_stol = fig.add_axes([0.02, 0.425, 0.105, 0.03])

box_deep = TextBox(ax_box_deep, "Deep ", initial=str(sim.cfg["mission"]["deep_m"]))
box_shallow = TextBox(ax_box_shallow, "Shallow ", initial=str(sim.cfg["mission"]["shallow_m"]))
box_surface = TextBox(ax_box_surface, "Surface ", initial=str(sim.cfg["mission"]["surface_m"]))
box_hold = TextBox(ax_box_hold, "Hold ", initial=str(sim.cfg["mission"]["hold_s"]))
box_tol = TextBox(ax_box_tol, "Tol ", initial=str(sim.cfg["mission"]["tol_m"]))
box_stol = TextBox(ax_box_stol, "Surf tol ", initial=str(sim.cfg["mission"]["surface_tol_m"]))

# Physics card
fig.text(0.02, 0.375, "Physics", fontsize=10, weight="bold")
ax_box_buoy = fig.add_axes([0.02, 0.335, 0.105, 0.03])
ax_box_engine = fig.add_axes([0.02, 0.295, 0.105, 0.03])
box_buoy = TextBox(ax_box_buoy, "Buoy g ", initial=str(sim.cfg["vehicle"].get("true_buoyancy_offset_g", 0.0)))
box_engine = TextBox(ax_box_engine, "Engine mL ", initial=str(sim.cfg["vehicle"]["buoyancy_engine_delta_m3"] * 1.0e6))

# Controller card
fig.text(0.155, 0.665, "Controller Settings", fontsize=10, weight="bold")

ax_box_kp = fig.add_axes([0.155, 0.625, 0.115, 0.03])
ax_box_ki = fig.add_axes([0.155, 0.585, 0.115, 0.03])
ax_box_kd = fig.add_axes([0.155, 0.545, 0.115, 0.03])
ax_box_deadband = fig.add_axes([0.155, 0.505, 0.115, 0.03])
ax_box_output_limit = fig.add_axes([0.155, 0.465, 0.115, 0.03])
ax_box_trim_limit = fig.add_axes([0.155, 0.425, 0.115, 0.03])
ax_box_trim_vel = fig.add_axes([0.155, 0.385, 0.115, 0.03])
ax_box_vel_alpha = fig.add_axes([0.155, 0.345, 0.115, 0.03])
ax_box_deeper_cmd = fig.add_axes([0.155, 0.305, 0.115, 0.03])
ax_box_shallower_cmd = fig.add_axes([0.155, 0.265, 0.115, 0.03])

box_kp = TextBox(ax_box_kp, "Kp ", initial=str(sim.cfg["controller"]["kp"]))
box_ki = TextBox(ax_box_ki, "Ki/trim ", initial=str(sim.cfg["controller"]["ki"]))
box_kd = TextBox(ax_box_kd, "Kd/vel ", initial=str(sim.cfg["controller"]["kd"]))
box_deadband = TextBox(ax_box_deadband, "DB m ", initial=str(sim.cfg["controller"]["deadband_m"]))
box_output_limit = TextBox(ax_box_output_limit, "Out lim ", initial=str(sim.cfg["controller"]["output_limit_us"]))
box_trim_limit = TextBox(ax_box_trim_limit, "Trim lim ", initial=str(sim.cfg["controller"]["trim_limit_us"]))
box_trim_vel = TextBox(ax_box_trim_vel, "Trim vel ", initial=str(sim.cfg["controller"]["trim_velocity_max_mps"]))
box_vel_alpha = TextBox(ax_box_vel_alpha, "Vel alpha ", initial=str(sim.cfg["controller"]["velocity_alpha"]))
box_deeper_cmd = TextBox(ax_box_deeper_cmd, "Deep us ", initial=str(sim.cfg["controller"]["deeper_command_us"]))
box_shallower_cmd = TextBox(ax_box_shallower_cmd, "Shallow us ", initial=str(sim.cfg["controller"]["shallower_command_us"]))


def on_start(event):
    sim.running = True


def on_pause(event):
    sim.running = False


def refresh_textboxes():
    box_deep.set_val(str(sim.cfg["mission"]["deep_m"]))
    box_shallow.set_val(str(sim.cfg["mission"]["shallow_m"]))
    box_surface.set_val(str(sim.cfg["mission"]["surface_m"]))
    box_hold.set_val(str(sim.cfg["mission"]["hold_s"]))
    box_tol.set_val(str(sim.cfg["mission"]["tol_m"]))
    box_stol.set_val(str(sim.cfg["mission"]["surface_tol_m"]))

    box_buoy.set_val(str(sim.cfg["vehicle"].get("true_buoyancy_offset_g", 0.0)))
    box_engine.set_val(str(sim.cfg["vehicle"]["buoyancy_engine_delta_m3"] * 1.0e6))

    box_kp.set_val(str(sim.cfg["controller"]["kp"]))
    box_ki.set_val(str(sim.cfg["controller"]["ki"]))
    box_kd.set_val(str(sim.cfg["controller"]["kd"]))
    box_deadband.set_val(str(sim.cfg["controller"]["deadband_m"]))
    box_output_limit.set_val(str(sim.cfg["controller"]["output_limit_us"]))
    box_trim_limit.set_val(str(sim.cfg["controller"]["trim_limit_us"]))
    box_trim_vel.set_val(str(sim.cfg["controller"]["trim_velocity_max_mps"]))
    box_vel_alpha.set_val(str(sim.cfg["controller"]["velocity_alpha"]))
    box_deeper_cmd.set_val(str(sim.cfg["controller"]["deeper_command_us"]))
    box_shallower_cmd.set_val(str(sim.cfg["controller"]["shallower_command_us"]))


def on_reset(event):
    sim.reset()
    refresh_textboxes()


def on_select_vp(label):
    sim.load_config(label)
    refresh_textboxes()


def on_select_controller(label):
    sim.set_controller_mode(label)


def on_apply(event):
    try:
        deep_m = float(box_deep.text)
        shallow_m = float(box_shallow.text)
        surface_m = float(box_surface.text)
        hold_s = float(box_hold.text)
        tol_m = float(box_tol.text)
        surface_tol_m = float(box_stol.text)

        true_buoyancy_offset_g = float(box_buoy.text)
        engine_ml = float(box_engine.text)

        kp = float(box_kp.text)
        ki = float(box_ki.text)
        kd = float(box_kd.text)
        deadband_m = float(box_deadband.text)
        output_limit_us = float(box_output_limit.text)
        trim_limit_us = float(box_trim_limit.text)
        trim_velocity_max_mps = float(box_trim_vel.text)
        velocity_alpha = float(box_vel_alpha.text)
        deeper_command_us = float(box_deeper_cmd.text)
        shallower_command_us = float(box_shallower_cmd.text)

        sim.apply_settings(
            deep_m=deep_m,
            shallow_m=shallow_m,
            surface_m=surface_m,
            hold_s=hold_s,
            tol_m=tol_m,
            surface_tol_m=surface_tol_m,
            true_buoyancy_offset_g=true_buoyancy_offset_g,
            engine_ml=engine_ml,
            kp=kp,
            ki=ki,
            kd=kd,
            deadband_m=deadband_m,
            output_limit_us=output_limit_us,
            trim_limit_us=trim_limit_us,
            trim_velocity_max_mps=trim_velocity_max_mps,
            velocity_alpha=velocity_alpha,
            deeper_command_us=deeper_command_us,
            shallower_command_us=shallower_command_us,
        )
    except ValueError:
        print("Invalid input in one or more fields")


btn_start.on_clicked(on_start)
btn_pause.on_clicked(on_pause)
btn_reset.on_clicked(on_reset)
btn_apply.on_clicked(on_apply)
radio_vp.on_clicked(on_select_vp)
radio_controller.on_clicked(on_select_controller)

# -----------------------------
# Animation update
# -----------------------------
def update(frame):
    # Run a few physics steps per visual frame so it feels smoother
    for _ in range(3):
        sim.step(0.05)

    # Update depth plot
    line_true_depth.set_data(sim.time_hist, sim.depth_hist)
    line_meas_depth.set_data(sim.time_hist, sim.measured_depth_hist)
    line_target_depth.set_data(sim.time_hist, sim.target_hist)

    if len(sim.time_hist) > 2:
        ax_depth_plot.set_xlim(max(0, sim.time_hist[0]), sim.time_hist[-1] + 1.0)

        max_depth = max(max(sim.depth_hist), max(sim.target_hist), 1.0)
        ax_depth_plot.set_ylim(max_depth + 0.2, -0.05)

    # Update actuator plot
    line_actuator.set_data(sim.time_hist, sim.actuator_hist)
    # Plot trim as neutral + trim so it is visible on the same scale as actuator us.
    trim_visible = [sim.act_neutral + x for x in sim.trim_hist]
    line_trim.set_data(sim.time_hist, trim_visible)
    if len(sim.time_hist) > 2:
        ax_act_plot.set_xlim(max(0, sim.time_hist[0]), sim.time_hist[-1] + 1.0)
        ax_act_plot.set_ylim(sim.act_min - 50, sim.act_max + 50)

    # Update depth graphic
    current_marker.set_data([0.5], [sim.true_depth])
    target_marker.set_data([0.5], [sim.current_target_depth])

    max_graph_depth = max(3.0, sim.current_target_depth + 1.0, sim.true_depth + 0.5)
    ax_depth_graphic.set_ylim(max_graph_depth, 0.0)

    # redraw tolerance band
    global tol_band
    try:
        tol_band.remove()
    except Exception:
        pass

    top = sim.current_target_depth - sim.current_target_tol
    bot = sim.current_target_depth + sim.current_target_tol
    tol_band = ax_depth_graphic.fill_between(
        [0.35, 0.65],
        [top, top],
        [bot, bot],
        alpha=0.25
    )

    error = sim.current_target_depth - sim.measured_depth
    status_text.set_text(
        f"VP: {sim.cfg_name}\n"
        f"mode: {sim.controller_mode}\n"
        f"running: {sim.running}\n"
        f"state: {sim.state_name()}\n"
        f"t: {sim.t:6.1f} s\n"
        f"depth: {sim.true_depth:6.3f} m\n"
        f"meas: {sim.measured_depth:6.3f} m\n"
        f"error: {error:6.3f} m\n"
        f"vel: {sim.true_velocity:6.3f} m/s\n"
        f"filt vel: {sim.filtered_velocity:6.3f}\n"
        f"target: {sim.current_target_depth:6.3f} m\n"
        f"hold: {sim.hold_elapsed():6.1f} s\n"
        f"act: {sim.actuator_us:6.1f} us\n"
        f"trim: {sim.adaptive_trim_us:6.1f} us\n"
        f"buoy: {sim.true_buoyancy_offset_g:6.1f} g"
    )

    fig.suptitle(f"VP Simulator - {sim.cfg_name} - {sim.controller_mode}", fontsize=14)

    return (
        line_true_depth,
        line_meas_depth,
        line_target_depth,
        line_actuator,
        line_trim,
        current_marker,
        target_marker,
        status_text,
    )


ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()
