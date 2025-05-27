from src.blimputils import init, Accelerometer, Gyroscope, Magnetometer, Motor
accel, gyro, mag, all_motors = init()
motor1 = all_motors["motor1"]
motor1.spin_forward(100)
print(accel.get_xyz())