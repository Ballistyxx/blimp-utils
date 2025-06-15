"""
Motor module for the blimp-utils library, specifically for the Texas Instruments DRV8871 Motor Driver.
"""

from typing import Literal, Union
import time
import RPi.GPIO as GPIO
import pigpio # Added for pigpio PWM

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor:
    """
    A class to represent a bidirectional brushed DC motor controlled by a DRV8871-like driver.

    The DRV8871 uses two pins for direction.
    This class simplifies it to two pins for direction and assumes speed is managed by PWM
    on one of these pins or a separate enable pin if the driver supports it.

    :param pin1: The first GPIO pin connected to the motor driver.
    :type pin1: int
    :param pin2: The second GPIO pin connected to the motor driver.
    :type pin2: int
    :param pwm_pin: Optional PWM pin if speed control is separate. For DRV8871, often one of the direction pins is PWM'd.
    :type pwm_pin: int, optional
    """
    VALID_RAMPS = ["linear", "quadratic", "log", "bezier"]

    def __init__(self, pin1: int, pin2: int, pwm_pin: Union[int, None] = None):
        """
        Initialize the motor.

        Attempts to use pigpio for PWM, falling back to RPi.GPIO if pigpio fails.

        :param pin1: The first GPIO pin (e.g., IN1 on DRV8871).
        :type pin1: int
        :param pin2: The second GPIO pin (e.g., IN2 on DRV8871).
        :type pin2: int
        :param pwm_pin: Optional dedicated PWM pin for speed control. 
                        If None, `pin1` will be used for PWM.
                        If `pwm_pin` is specified as `pin1` or `pin2`, that pin will be used for PWM.
        :type pwm_pin: int, optional
        :raises TypeError: if pin1 or pin2 are not integers, or if pwm_pin is not an integer when provided.
        """
        if not isinstance(pin1, int) or not isinstance(pin2, int):
            raise TypeError("Motor pins must be integers.")
        if pwm_pin is not None and not isinstance(pwm_pin, int):
            raise TypeError("PWM pin must be an integer if provided.")

        self.pin1 = pin1
        self.pin2 = pin2
        
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

        if pwm_pin is None:
            self.pwm_pin_actual = self.pin1 # Default PWM to pin1
        else:
            self.pwm_pin_actual = pwm_pin

        if self.pwm_pin_actual != self.pin1 and self.pwm_pin_actual != self.pin2:
            GPIO.setup(self.pwm_pin_actual, GPIO.OUT)
        
        self.pi = None
        self.using_pigpio_pwm = False
        self.pwm_controller = None # For RPi.GPIO fallback

        # Attempt to initialize pigpio PWM
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                print(f"Failed to connect to pigpiod for pin {self.pwm_pin_actual}. Will attempt RPi.GPIO PWM.")
                self.pi = None # Ensure it's None if connection failed
            else:
                self.pi.set_mode(self.pwm_pin_actual, pigpio.OUTPUT)
                # pigpio default PWM frequency varies; can set if needed:
                # self.pi.set_PWM_frequency(self.pwm_pin_actual, 1000) 
                # pigpio default range is 255, which matches our speed input.
                # self.pi.set_PWM_range(self.pwm_pin_actual, 255) 
                self.pi.set_PWM_dutycycle(self.pwm_pin_actual, 0) # Start with 0 duty cycle
                self.using_pigpio_pwm = True
                print(f"pigpio PWM initialized on pin {self.pwm_pin_actual}")
        except Exception as e_pigpio: # Broad exception for pigpio issues (e.g., daemon not running)
            print(f"Failed to initialize pigpio PWM on pin {self.pwm_pin_actual}: {e_pigpio}")
            print("Ensure pigpiod daemon is running. Will attempt RPi.GPIO PWM.")
            if self.pi and self.pi.connected: # If connection was made but subsequent calls failed
                try:
                    self.pi.stop()
                except Exception: # Ignore errors during stop if already problematic
                    pass
            self.pi = None

        # Fallback to RPi.GPIO PWM if pigpio failed
        if not self.using_pigpio_pwm:
            print(f"Attempting RPi.GPIO PWM fallback for pin {self.pwm_pin_actual}...")
            try:
                self.pwm_controller = GPIO.PWM(self.pwm_pin_actual, 1000) # 1kHz frequency
                self.pwm_controller.start(0) # Start with 0% duty cycle
                print(f"RPi.GPIO PWM initialized on pin {self.pwm_pin_actual}")
            except Exception as e_rpi:
                print(f"Failed to initialize RPi.GPIO PWM on pin {self.pwm_pin_actual}: {e_rpi}")
                self.pwm_controller = None
        
        print(f"Motor initialized with pin1={self.pin1}, pin2={self.pin2}, pwm_pin_actual={self.pwm_pin_actual}. Using pigpio: {self.using_pigpio_pwm}")

    def _validate_speed(self, speed: int) -> None:
        """
        Validate the motor speed.

        :param speed: The motor speed (0-255).
        :type speed: int
        :raises ValueError: if speed is not between 0 and 255.
        """
        if not (0 <= speed <= 255):
            raise ValueError("Speed must be between 0 and 255.")

    def _validate_ramp(self, ramp: str) -> None:
        """
        Validate the ramp profile.

        :param ramp: The ramp profile name.
        :type ramp: str
        :raises ValueError: if the ramp profile is invalid.
        """
        if ramp not in self.VALID_RAMPS:
            raise ValueError(f"Invalid ramp profile. Choose from: {', '.join(self.VALID_RAMPS)}")

    def _set_pwm_duty_cycle(self, speed_value: int) -> None:
        """
        Sets the PWM duty cycle for speed control using pigpio or RPi.GPIO.

        :param speed_value: The motor speed (0-255).
        :type speed_value: int
        """
        clamped_speed = max(0, min(255, speed_value))

        if self.using_pigpio_pwm and self.pi:
            try:
                self.pi.set_PWM_dutycycle(self.pwm_pin_actual, clamped_speed)
                # print(f"pigpio PWM on pin {self.pwm_pin_actual}: Duty Cycle {clamped_speed}/255")
            except Exception as e:
                print(f"Error setting pigpio PWM duty cycle on pin {self.pwm_pin_actual}: {e}")
                # Potentially mark pigpio as unusable if errors persist
        elif self.pwm_controller: # RPi.GPIO fallback
            duty_cycle_percent = (clamped_speed / 255.0) * 100.0
            clamped_duty_cycle_percent = max(0.0, min(100.0, duty_cycle_percent))
            try:
                self.pwm_controller.ChangeDutyCycle(clamped_duty_cycle_percent)
                # print(f"RPi.GPIO PWM on pin {self.pwm_pin_actual}: Duty Cycle {clamped_duty_cycle_percent:.2f}%")
            except Exception as e:
                print(f"Error setting RPi.GPIO PWM duty cycle on pin {self.pwm_pin_actual}: {e}")
        # else:
            # print(f"No PWM controller available for pin {self.pwm_pin_actual}. Speed {clamped_speed} results in ON/OFF.")


    def spin_forward(self, speed: int, ramp: str = "linear") -> None:
        """
        Spin the motor forward.

        :param speed: The motor speed (0-255). If PWM is not available, any speed > 0 means ON.
        :type speed: int
        :param ramp: The ramp profile. Defaults to "linear". (Currently not implemented)
        :type ramp: str, optional
        :raises ValueError: if speed or ramp is invalid.
        :example: ``motor.spin_forward(150, ramp="quadratic")``
        """
        self._validate_speed(speed)
        self._validate_ramp(ramp)

        # Determine direction pin states
        if self.pwm_pin_actual == self.pin1:
            GPIO.output(self.pin2, GPIO.LOW)
        elif self.pwm_pin_actual == self.pin2:
            GPIO.output(self.pin1, GPIO.HIGH)
        else: # Dedicated PWM pin
            GPIO.output(self.pin1, GPIO.HIGH)
            GPIO.output(self.pin2, GPIO.LOW)
        
        # Apply speed via PWM or fallback
        if (self.using_pigpio_pwm and self.pi) or self.pwm_controller:
            self._set_pwm_duty_cycle(speed)
            duty_cycle_for_msg = (speed / 255.0) * 100.0
            effective_speed_message = f"speed {speed} (PWM duty cycle ~{duty_cycle_for_msg:.1f}%)"
        else: # No PWM controller at all (neither pigpio nor RPi.GPIO worked)
            if speed > 0:
                if self.pwm_pin_actual == self.pin1: GPIO.output(self.pin1, GPIO.HIGH)
                elif self.pwm_pin_actual == self.pin2: GPIO.output(self.pin2, GPIO.HIGH)
                # If dedicated PWM pin and no controller, its state is not changed here (remains OUT)
            else: # speed is 0
                if self.pwm_pin_actual == self.pin1: GPIO.output(self.pin1, GPIO.LOW)
                elif self.pwm_pin_actual == self.pin2: GPIO.output(self.pin2, GPIO.LOW)
            effective_speed_message = f"speed {speed} (PWM N/A, {'ON' if speed > 0 else 'OFF'})"

        print(f"Motor ({self.pin1},{self.pin2}) spinning forward, {effective_speed_message}, ramp {ramp}")
        if ramp != "linear":
            print(f"Ramping profile '{ramp}' selected (actual ramp not implemented).")


    def spin_reverse(self, speed: int, ramp: str = "linear") -> None:
        """
        Spin the motor reverse.

        :param speed: The motor speed (0-255). If PWM is not available, any speed > 0 means ON.
        :type speed: int
        :param ramp: The ramp profile. Defaults to "linear". (Currently not implemented)
        :type ramp: str, optional
        :raises ValueError: if speed or ramp is invalid.
        :example: ``motor.spin_reverse(100)``
        """
        self._validate_speed(speed)
        self._validate_ramp(ramp)

        # Determine direction pin states
        if self.pwm_pin_actual == self.pin1:
            GPIO.output(self.pin2, GPIO.HIGH)
        elif self.pwm_pin_actual == self.pin2:
            GPIO.output(self.pin1, GPIO.LOW)
        else: # Dedicated PWM pin
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.HIGH)

        # Apply speed via PWM or fallback
        if (self.using_pigpio_pwm and self.pi) or self.pwm_controller:
            self._set_pwm_duty_cycle(speed)
            duty_cycle_for_msg = (speed / 255.0) * 100.0
            effective_speed_message = f"speed {speed} (PWM duty cycle ~{duty_cycle_for_msg:.1f}%)"
        else: # No PWM controller at all
            if speed > 0:
                if self.pwm_pin_actual == self.pin1: GPIO.output(self.pin1, GPIO.HIGH)
                elif self.pwm_pin_actual == self.pin2: GPIO.output(self.pin2, GPIO.HIGH)
            else: # speed is 0
                if self.pwm_pin_actual == self.pin1: GPIO.output(self.pin1, GPIO.LOW)
                elif self.pwm_pin_actual == self.pin2: GPIO.output(self.pin2, GPIO.LOW)
            effective_speed_message = f"speed {speed} (PWM N/A, {'ON' if speed > 0 else 'OFF'})"

        print(f"Motor ({self.pin1},{self.pin2}) spinning reverse, {effective_speed_message}, ramp {ramp}")
        if ramp != "linear":
            print(f"Ramping profile '{ramp}' selected (actual ramp not implemented).")

    def stop(self) -> None:
        """
        Stop the motor (brake).
        Sets PWM to 0 and direction pins to LOW for braking.
        """
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        
        if self.using_pigpio_pwm and self.pi:
            try:
                self.pi.set_PWM_dutycycle(self.pwm_pin_actual, 0)
            except Exception as e:
                print(f"Error setting pigpio PWM to 0 on stop for pin {self.pwm_pin_actual}: {e}")
        elif self.pwm_controller:
            try:
                self.pwm_controller.ChangeDutyCycle(0)
            except Exception as e:
                print(f"Error setting RPi.GPIO PWM to 0 on stop for pin {self.pwm_pin_actual}: {e}")
        
        print(f"Motor ({self.pin1},{self.pin2}) stopped. PWM on pin {self.pwm_pin_actual} set to 0.")

    def cleanup(self):
        """
        Clean up GPIO resources. Call this when the motor is no longer needed.
        """
        self.stop() # Ensure motor is stopped and PWM is off

        if self.using_pigpio_pwm and self.pi:
            print(f"Stopping pigpio for pin {self.pwm_pin_actual}.")
            # PWM duty cycle already set to 0 by self.stop()
            try:
                self.pi.stop() # Disconnect from pigpiod
            except Exception as e:
                print(f"Error stopping pigpio: {e}")
            self.pi = None
            self.using_pigpio_pwm = False
        
        if self.pwm_controller: # RPi.GPIO PWM
            print(f"Stopping RPi.GPIO PWM for pin {self.pwm_pin_actual}.")
            try:
                self.pwm_controller.stop()
            except Exception as e:
                print(f"Error stopping RPi.GPIO PWM: {e}")
            self.pwm_controller = None
        
        # GPIO.cleanup() # Avoid general cleanup if other parts of the system use GPIO
        # Only cleanup pins used by this motor instance for direction if not PWM pin
        pins_to_cleanup_rpi = set()
        if self.pin1 != self.pwm_pin_actual or not (self.using_pigpio_pwm or self.pwm_controller):
             pins_to_cleanup_rpi.add(self.pin1)
        if self.pin2 != self.pwm_pin_actual or not (self.using_pigpio_pwm or self.pwm_controller):
             pins_to_cleanup_rpi.add(self.pin2)
        
        # If pwm_pin_actual was a dedicated pin and RPi.GPIO PWM was used for it (or no PWM)
        if self.pwm_pin_actual not in [self.pin1, self.pin2] and not self.using_pigpio_pwm :
            pins_to_cleanup_rpi.add(self.pwm_pin_actual)

        if pins_to_cleanup_rpi:
            GPIO.cleanup(list(pins_to_cleanup_rpi))
            print(f"Cleaned up RPi.GPIO pins: {list(pins_to_cleanup_rpi)}.")
        else:
            print("No RPi.GPIO pins (excluding pigpio PWM pin) to clean up for this motor instance.")
