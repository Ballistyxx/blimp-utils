from blimputils import Accelerometer, Gyroscope, Magnetometer
import time
try:
    accel = Accelerometer()
    gyro = Gyroscope()
    mag = Magnetometer()
    for i in range(10):
        accel_data = accel.get_xyz()
        gyro_data = gyro.get_xyz()
        mag_data = mag.get_xyz()
        mag_temp = mag.get_t()
        print("Accelerometer Data:", accel_data)
        print("Gyroscope Data:", gyro_data)
        print("Magnetometer Data:", mag_data)
        print("Magnetometer Temp:", mag_temp)
        time.sleep(0.25)

except Exception as e:
    print(f"An error occurred: {e}")
