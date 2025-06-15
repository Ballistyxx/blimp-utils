"""
Minimal Motor module for the blimp-utils library, specifically for the Texas Instruments DRV8212p Motor Driver,
tailored to run test_motor_functionality.py on a Raspberry Pi Zero 2W.
"""

import RPi.GPIO as GPIO
import pigpio
import time
import math

pi = None
motor_instances_module_scope = {} # Stores motor instances for cleanup

class Motor:
    """
    Motor class for controlling a single motor connected via DRV8212p.
    Assumes pigpio daemon is running (sudo pigpiod).
    After calling Motor.init(), initialized motor instances are available via Motor.instances dict.
    """
    PREDEFINED_MOTORS_PINS = {
        "motor1": {"pin1": 21, "pin2": 13},  # m1p0, m1p1 (BCM pins)
        "motor2": {"pin1": 20, "pin2": 6},   # m2p0, m2p1
        "motor3": {"pin1": 16, "pin2": 5},   # m3p0, m3p1
        "motor4": {"pin1": 19, "pin2": 26}   # m4p0, m4p1
    }
    VALID_RAMPS = ["linear", "quadratic", "log", "bezier"]

    instances = {}  # Class attribute to hold all initialized motor instances

    def __init__(self, in1_pin: int, in2_pin: int, pi_instance: pigpio.pi):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pi = pi_instance
        self.current_speed = 0  # Current speed percentage (0-100)
        # Ensure motor is stopped initially by setting PWM directly
        self._set_pwm_direct(0)

    def _set_pwm_direct(self, speed_percent: int):
        """Sets the motor PWM directly without affecting current_speed tracking for ramps."""
        # Clamp speed to the expected range [0, 100]
        clamped_speed = max(0, min(100, speed_percent))

        duty_cycle = int((clamped_speed / 100.0) * 255)

        if clamped_speed > 0:
            self.pi.set_PWM_dutycycle(self.in1_pin, duty_cycle)
            self.pi.set_PWM_dutycycle(self.in2_pin, 0)
        else:  # speed_percent == 0
            self.pi.set_PWM_dutycycle(self.in1_pin, 0)
            self.pi.set_PWM_dutycycle(self.in2_pin, 0)

    def spin(self, target_speed: int, ramp_type: str = None, ramp_duration: float = 0):
        target_speed = max(0, min(100, target_speed)) # Ensure target_speed is valid

        if ramp_type and ramp_type not in self.VALID_RAMPS:
            print(f"Warning: Invalid ramp_type '{ramp_type}'. Valid types: {self.VALID_RAMPS}. Ignoring ramp.")
            ramp_type = None
        
        if ramp_type and ramp_duration > 0:
            start_speed = self.current_speed
            speed_difference = target_speed - start_speed

            if speed_difference == 0:
                if self.current_speed != target_speed: # Ensure PWM is set if current_speed was out of sync
                    self._set_pwm_direct(target_speed)
                self.current_speed = target_speed
                return

            # Aim for updates roughly every 20-50ms
            num_steps = max(1, int(ramp_duration / 0.05)) 
            time_step = ramp_duration / num_steps

            for i in range(num_steps + 1):
                t_norm = i / num_steps  # Normalized time from 0 to 1
                ramp_factor = 0

                if ramp_type == "linear":
                    ramp_factor = t_norm
                elif ramp_type == "quadratic":  # Ease-in quadratic
                    ramp_factor = t_norm ** 2
                elif ramp_type == "log":  # Logarithmic-like curve (starts fast, decelerates rate of change)
                                          # Using math.log1p for numerical stability with t_norm near 0.
                                          # (math.e - 1) ensures the argument to log1p is > 0 for t_norm > 0.
                    if t_norm == 0: ramp_factor = 0
                    elif t_norm == 1: ramp_factor = 1
                    else: ramp_factor = math.log1p(t_norm * (math.e - 1)) / math.log1p(math.e - 1)
                elif ramp_type == "bezier":  # Smoothstep (cubic Hermite interpolation like Bezier)
                    ramp_factor = t_norm * t_norm * (3.0 - 2.0 * t_norm)
                
                ramped_speed_val = start_speed + speed_difference * ramp_factor
                actual_set_speed = int(round(ramped_speed_val))
                
                self._set_pwm_direct(actual_set_speed)
                self.current_speed = actual_set_speed # Update current speed during ramp

                if i < num_steps:
                    time.sleep(time_step)
            
            # Ensure final speed is set precisely and current_speed reflects it
            self._set_pwm_direct(target_speed)
            self.current_speed = target_speed
        else:
            # No ramping or invalid ramp parameters, set speed directly
            self._set_pwm_direct(target_speed)
            self.current_speed = target_speed

    def stop(self):
        """Stops the motor immediately and updates current_speed."""
        self._set_pwm_direct(0)
        self.current_speed = 0

    @staticmethod
    def init():
        global pi, motor_instances_module_scope

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        Motor.instances.clear()
        motor_instances_module_scope.clear()

        if pi is None or not pi.connected:
            if pi is not None: # If pi object exists but not connected
                try:
                    pi.stop()
                except Exception: pass # Ignore errors if already disconnected or invalid
            pi = pigpio.pi()
            if not pi.connected:
                GPIO.setwarnings(True)
                raise RuntimeError("Failed to connect to pigpio daemon. Is it running? Try: sudo pigpiod")
        
        for motor_name, pins_config in Motor.PREDEFINED_MOTORS_PINS.items():
            try:
                motor_obj = Motor(pins_config["pin1"], pins_config["pin2"], pi)
                Motor.instances[motor_name] = motor_obj
                motor_instances_module_scope[motor_name] = motor_obj
                print(f"Successfully initialized {motor_name}")
            except Exception as e:
                print(f"Error initializing motor {motor_name} in motor.py: {e}")

    @staticmethod
    def cleanup():
        global pi, motor_instances_module_scope

        for motor_name, motor_obj in motor_instances_module_scope.items():
            try:
                motor_obj.stop()
            except Exception as e:
                print(f"Error stopping motor {motor_name} during cleanup: {e}")
        
        motor_instances_module_scope.clear()
        Motor.instances.clear()

        if pi is not None and pi.connected:
            try:
                pi.stop()
            except Exception as e:
                print(f"Error stopping pigpio: {e}")
            pi = None

        GPIO.cleanup()
        GPIO.setwarnings(True)

