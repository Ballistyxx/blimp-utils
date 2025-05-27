"""
Motor module for the blimp-utils library, specifically for the Texas Instruments DRV8871 Motor Driver.


:license: The MIT License (MIT)
:author: Eli Ferrara 
:email: eli.ferrara256@gmail.com
:version: V1.0.0
:date: 2025-05-27
:url: https://github.com/Ballistyxx/blimp-utils
"""

from typing import Literal, Union
import time
import RPi.GPIO as GPIO

# Setup GPIO mode and warnings
GPIO.setmode(GPIO.BCM) # Or GPIO.BOARD
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

        :param pin1: The first GPIO pin (e.g., IN1 on DRV8871).
        :type pin1: int
        :param pin2: The second GPIO pin (e.g., IN2 on DRV8871).
        :type pin2: int
        :param pwm_pin: Optional dedicated PWM pin for speed control. If None, assumes pin1 or pin2 will be PWM'd.
        :type pwm_pin: int, optional
        :raises TypeError: if pin1 or pin2 are not integers.
        """
        if not isinstance(pin1, int) or not isinstance(pin2, int):
            raise TypeError("Motor pins must be integers.")
        if pwm_pin is not None and not isinstance(pwm_pin, int):
            raise TypeError("PWM pin must be an integer if provided.")

        self.pin1 = pin1
        self.pin2 = pin2
        self.pwm_pin_actual = pwm_pin if pwm_pin is not None else self.pin1 # Actual pin for PWM
        self.pwm_controller = None

        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        
        # Setup PWM
        # If a dedicated pwm_pin is provided and it's different from pin1 and pin2, set it up.
        # Otherwise, pwm is assumed to be on self.pwm_pin_actual (which defaults to pin1).
        if self.pwm_pin_actual != self.pin1 and self.pwm_pin_actual != self.pin2:
            GPIO.setup(self.pwm_pin_actual, GPIO.OUT)
        
        self.pwm_controller = GPIO.PWM(self.pwm_pin_actual, 1000) # 1kHz frequency
        self.pwm_controller.start(0) # Start with 0% duty cycle
        
        print(f"Motor initialized with pin1={self.pin1}, pin2={self.pin2}, pwm_pin={self.pwm_pin_actual}")

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

    def _set_pwm_duty_cycle(self, duty_cycle_percent: float) -> None:
        """
        Sets the PWM duty cycle for speed control.
        Converts 0-255 speed to 0-100% duty cycle.

        :param duty_cycle_percent: The duty cycle (0.0 to 100.0).
        :type duty_cycle_percent: float
        """
        if self.pwm_controller:
            self.pwm_controller.ChangeDutyCycle(duty_cycle_percent)
        # print(f"PWM on pin {self.pwm_pin_actual}: Duty Cycle {duty_cycle_percent:.2f}%")

    def _spin(self, direction: int, speed: int, ramp: str) -> None:
        """
        Internal method to control motor spin.

        :param direction: 0 for forward, 1 for reverse.
        :type direction: int
        :param speed: The motor speed (0-255).
        :type speed: int
        :param ramp: The ramp profile ("linear", "quadratic", "log", "bezier").
        :type ramp: str
        """
        self._validate_speed(speed)
        self._validate_ramp(ramp)

        duty_cycle = (speed / 255.0) * 100.0

        if direction == 0:  # Forward
            GPIO.output(self.pin1, GPIO.HIGH if self.pwm_pin_actual != self.pin1 else GPIO.LOW) # If pin1 is PWM, it's handled by PWM
            GPIO.output(self.pin2, GPIO.LOW)
            print(f"Motor ({self.pin1},{self.pin2}) spinning forward, speed {speed}, ramp {ramp}")
        elif direction == 1:  # Reverse
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.HIGH if self.pwm_pin_actual != self.pin2 else GPIO.LOW) # If pin2 is PWM, it's handled by PWM
            print(f"Motor ({self.pin1},{self.pin2}) spinning reverse, speed {speed}, ramp {ramp}")
        else: # Brake
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.LOW)
            print(f"Motor ({self.pin1},{self.pin2}) stopping/braking")
            duty_cycle = 0 # Ensure speed is zero when stopping
        
        # Apply speed via PWM
        self._set_pwm_duty_cycle(duty_cycle)
        if ramp != "linear": # Placeholder for future ramp implementation
            print(f"Ramping profile '{ramp}' selected (actual ramp not implemented).")

    def spin_forward(self, speed: int, ramp: str = "linear") -> None:
        """
        Spin the motor forward.

        :param speed: The motor speed (0-255).
        :type speed: int
        :param ramp: The ramp profile. Defaults to "linear".
        :type ramp: str, optional
        :raises ValueError: if speed or ramp is invalid.
        :example: ``motor.spin_forward(150, ramp="quadratic")``
        """
        # If pwm_pin is pin1, pin1 controls speed, pin2 direction (LOW for forward)
        # If pwm_pin is pin2, pin2 controls speed, pin1 direction (HIGH for forward)
        # If pwm_pin is separate, pin1=HIGH, pin2=LOW for forward
        if self.pwm_pin_actual == self.pin1:
            GPIO.output(self.pin2, GPIO.LOW)
        elif self.pwm_pin_actual == self.pin2:
             # This case is tricky: if pin2 is PWM, pin1 must be set for direction.
             # To make pin2 PWM for forward, pin1 would typically be HIGH.
             # However, our _spin logic assumes pin1=LOW, pin2=HIGH for reverse.
             # This setup is more aligned with IN1/IN2 on H-bridge where one is PWM, other is dir.
             # For now, let's assume forward means pin1=HIGH, pin2=LOW (or pin1=PWM, pin2=LOW)
            GPIO.output(self.pin1, GPIO.HIGH) # This might need adjustment based on H-bridge logic
        else: # Dedicated PWM pin
            GPIO.output(self.pin1, GPIO.HIGH)
            GPIO.output(self.pin2, GPIO.LOW)
        
        self._spin(direction=0, speed=speed, ramp=ramp)


    def spin_reverse(self, speed: int, ramp: str = "linear") -> None:
        """
        Spin the motor reverse.

        :param speed: The motor speed (0-255).
        :type speed: int
        :param ramp: The ramp profile. Defaults to "linear".
        :type ramp: str, optional
        :raises ValueError: if speed or ramp is invalid.
        :example: ``motor.spin_reverse(100)``
        """
        # If pwm_pin is pin1, pin1 controls speed, pin2 direction (HIGH for reverse)
        # If pwm_pin is pin2, pin2 controls speed, pin1 direction (LOW for reverse)
        # If pwm_pin is separate, pin1=LOW, pin2=HIGH for reverse
        if self.pwm_pin_actual == self.pin1:
            GPIO.output(self.pin2, GPIO.HIGH)
        elif self.pwm_pin_actual == self.pin2:
            GPIO.output(self.pin1, GPIO.LOW)
        else: # Dedicated PWM pin
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.HIGH)

        self._spin(direction=1, speed=speed, ramp=ramp)

    def stop(self) -> None:
        """
        Stop the motor (brake).

        This typically involves setting both control pins to LOW or both to HIGH
        for a DRV8871 driver to brake the motor.
        :example: ``motor.stop()``
        """
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        if self.pwm_controller:
            self.pwm_controller.ChangeDutyCycle(0)
        print(f"Motor ({self.pin1},{self.pin2}) stopped.")

    def cleanup(self):
        """
        Clean up GPIO resources. Call this when the motor is no longer needed.
        """
        self.stop() # Ensure motor is stopped
        if self.pwm_controller:
            self.pwm_controller.stop()
        
        # GPIO.cleanup() # Avoid general cleanup if other parts of the system use GPIO
        # Only cleanup pins used by this motor instance
        pins_to_cleanup = {self.pin1, self.pin2}
        if self.pwm_pin_actual:
            pins_to_cleanup.add(self.pwm_pin_actual)
        GPIO.cleanup(list(pins_to_cleanup))
        print(f"Cleaned up GPIO for motor ({self.pin1},{self.pin2}, pwm_pin={self.pwm_pin_actual}).")
