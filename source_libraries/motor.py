import drivetrain as drivetrain
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#motor0 = 21
#motor1 = 13

m1p0 = 21
m1p1 = 13
m2p0 = 20
m2p1 = 6
m3p0 = 16
m3p1 = 5
m4p0 = 19
m4p1 = 26


GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(m1p0, GPIO.OUT)
GPIO.setup(m1p1, GPIO.OUT)
GPIO.setup(m2p0, GPIO.OUT)
GPIO.setup(m2p1, GPIO.OUT)
GPIO.setup(m3p0, GPIO.OUT)
GPIO.setup(m3p1, GPIO.OUT)
GPIO.setup(m4p0, GPIO.OUT)
GPIO.setup(m4p1, GPIO.OUT)

#redundant nSleep
GPIO.output(23, True)
GPIO.output(24, True)

GPIO.output(m1p0, True)
GPIO.output(m1p1, False)
time.sleep(1)
GPIO.output(m1p0, False)
GPIO.output(m1p1, False)
time.sleep(1)

GPIO.output(m2p0, True)
GPIO.output(m2p1, False)
time.sleep(1)
GPIO.output(m2p0, False)
GPIO.output(m2p1, False)
time.sleep(1)

GPIO.output(m3p0, True)
GPIO.output(m3p1, False)
time.sleep(1)
GPIO.output(m3p0, False)
GPIO.output(m3p1, False)
time.sleep(1)

GPIO.output(m4p0, True)
GPIO.output(m4p1, False)
time.sleep(1)
GPIO.output(m4p0, False)
GPIO.output(m4p1, False)
time.sleep(1)


GPIO.output(23, False)
GPIO.output(24, False)
