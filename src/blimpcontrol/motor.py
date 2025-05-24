"""
Motor module for the blimpcontrol library.
"""

from typing import Literal
import time

# In a real scenario, you would use a library like RPi.GPIO
# For simulation purposes, we'll mock GPIO operations.
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM) # Or GPIO.BOARD
    GPIO.setwarnings(False)
    IS_RASPBERRY_PI = True
except (ImportError, RuntimeError):
    print("RPi.GPIO not available. Simulating GPIO operations.")
    IS_RASPBERRY_PI = False
    # Mock GPIO class for non-Pi environments
    class MockGPIO:
        OUT = "out"
        BCM = "bcm"
        HIGH = 1
        LOW = 0
        def __init__(self):
            self._pin_states = {}
            self._pin_modes = {}

        def setmode(self, mode):
            print(f"MockGPIO: setmode({mode})")

        def setwarnings(self, flag):
            print(f"MockGPIO: setwarnings({flag})")

        def setup(self, pin, mode):
            print(f"MockGPIO: setup({pin}, {mode})")
            self._pin_modes[pin] = mode
            self._pin_states[pin] = self.LOW # Default to LOW

        def output(self, pin, state):
            if pin not in self._pin_modes or self._pin_modes[pin] != self.OUT:
                raise RuntimeError(f"Pin {pin} not set up for output.")
            print(f"MockGPIO: output({pin}, {state})")
            self._pin_states[pin] = state

        def cleanup(self):
            print("MockGPIO: cleanup()")
            self._pin_states = {}
            self._pin_modes = {}

    GPIO = MockGPIO()


class Motor:
    """
    A class to represent a bidirectional brushed DC motor controlled by a DRV8871-like driver.

    The DRV8871 typically uses two pins for direction and PWM for speed.
    This class simplifies it to two pins for direction and assumes speed is managed by PWM
    on one of these pins or a separate enable pin if the driver supports it.
    For this example, we'll treat `pin1` and `pin2` as controlling direction,
    and speed will be an abstract concept passed to a PWM controller (simulated).

    :param pin1: The first GPIO pin connected to the motor driver.
    :type pin1: int
    :param pin2: The second GPIO pin connected to the motor driver.
    :type pin2: int
    :param pwm_pin: Optional PWM pin if speed control is separate. For DRV8871, often one of the direction pins is PWM'd.
    :type pwm_pin: int, optional
    """
    VALID_RAMPS = ["linear", "quadratic", "log", "bezier"]

    def __init__(self, pin1: int, pin2: int, pwm_pin: int | None = None):
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
        self.pwm_pin = pwm_pin if pwm_pin is not None else self.pin1 # Default to PWM on pin1

        if IS_RASPBERRY_PI or isinstance(GPIO, MockGPIO):
            GPIO.setup(self.pin1, GPIO.OUT)
            GPIO.setup(self.pin2, GPIO.OUT)
            if self.pwm_pin != self.pin1 and self.pwm_pin != self.pin2 : # If dedicated PWM pin
                 GPIO.setup(self.pwm_pin, GPIO.OUT)
                 # self.pwm_controller = GPIO.PWM(self.pwm_pin, 1000) # 1kHz frequency
                 # self.pwm_controller.start(0) # Start with 0% duty cycle
            # else: # PWM on one of the direction pins
                # self.pwm_controller = GPIO.PWM(self.pwm_pin, 1000)
                # self.pwm_controller.start(0)
            print(f"Motor initialized with pin1={self.pin1}, pin2={self.pin2}, pwm_pin={self.pwm_pin}")
        else:
            raise RuntimeError("GPIO library not available or not mocked.")


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
        # self.pwm_controller.ChangeDutyCycle(duty_cycle_percent)
        print(f"Simulating PWM on pin {self.pwm_pin}: Duty Cycle {duty_cycle_percent:.2f}%")


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
            GPIO.output(self.pin1, GPIO.HIGH)
            GPIO.output(self.pin2, GPIO.LOW)
            print(f"Motor ({self.pin1},{self.pin2}) spinning forward, speed {speed}, ramp {ramp}")
        elif direction == 1:  # Reverse
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.HIGH)
            print(f"Motor ({self.pin1},{self.pin2}) spinning reverse, speed {speed}, ramp {ramp}")
        else: # Brake or Coast (depending on driver, DRV8871 brakes with LOW,LOW or HIGH,HIGH)
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.LOW)
            print(f"Motor ({self.pin1},{self.pin2}) stopping/braking")
            duty_cycle = 0 # Ensure speed is zero when stopping

        # Apply speed via PWM
        # This is a simplified model. Real ramping would involve changing duty_cycle over time.
        self._set_pwm_duty_cycle(duty_cycle)
        if ramp != "linear": # Placeholder for future ramp implementation
            print(f"Ramping profile '{ramp}' selected (simulation only).")


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
        # self.pwm_controller.ChangeDutyCycle(0)
        self._set_pwm_duty_cycle(0)
        print(f"Motor ({self.pin1},{self.pin2}) stopped.")

    def cleanup(self):
        """
        Clean up GPIO resources. Call this when the motor is no longer needed.
        """
        if IS_RASPBERRY_PI or isinstance(GPIO, MockGPIO):
            self.stop() # Ensure motor is stopped
            # self.pwm_controller.stop()
            GPIO.cleanup([self.pin1, self.pin2, self.pwm_pin] if self.pwm_pin else [self.pin1, self.pin2])
            print(f"Cleaned up GPIO for motor ({self.pin1},{self.pin2}).")

if __name__ == '__main__':
    # Example Usage (will use MockGPIO if not on RPi)
    try:
        # Motor(pin1, pin2)
        # For DRV8871, pin1=IN1, pin2=IN2. Speed is controlled by PWM on IN1 or IN2.
        # Let's assume pwm_pin is one of the INx pins or a separate EN/PWM pin if the driver has one.
        # If DRV8871 is used with RPi, one pin is PWM, other is direction.
        # e.g. motor1_in1 = 17 (PWM), motor1_in2 = 18 (Direction HIGH/LOW)

        # Simulating a setup where pin1 is PWM'd for speed, pin2 sets direction part
        # This is a common way to use H-bridges like L298N or DRV8871 with limited PWM pins
        # For DRV8871:
        # Forward: IN1=PWM, IN2=LOW
        # Reverse: IN1=LOW, IN2=PWM (or IN1=PWM, IN2=HIGH, depending on how you define "forward")
        # For this class, we'll assume:
        # spin_forward: pin1=HIGH/PWM, pin2=LOW
        # spin_reverse: pin1=LOW, pin2=HIGH/PWM

        motor1 = Motor(pin1=17, pin2=18) # Assume pin 17 can be PWM'd
        motor2 = Motor(pin1=27, pin2=22, pwm_pin=23) # Example with a dedicated PWM pin

        print("\n--- Motor 1 Test ---")
        motor1.spin_forward(150, ramp="quadratic")
        time.sleep(1)
        motor1.spin_reverse(100)
        time.sleep(1)
        motor1.stop()
        time.sleep(0.5)

        print("\n--- Motor 2 Test (with dedicated PWM pin) ---")
        motor2.spin_forward(255, ramp="linear")
        time.sleep(1)
        motor2.stop()

        # Test validation
        try:
            motor1.spin_forward(300)
        except ValueError as e:
            print(f"Caught expected error: {e}")

        try:
            motor1.spin_forward(100, ramp="invalid_ramp")
        except ValueError as e:
            print(f"Caught expected error: {e}")

    finally:
        if IS_RASPBERRY_PI or isinstance(GPIO, MockGPIO):
            motor1.cleanup()
            motor2.cleanup()
            # GPIO.cleanup() # General cleanup if other GPIOs were used
            print("All motors cleaned up.")
