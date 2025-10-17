#!/usr/bin/env python3
import math
import time
import serial
import statistics

import adafruit_bno055
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# BNO055 operation-mode register values
OPMODE_CONFIG       = 0x00
OPMODE_ACCONLY      = 0x01
OPMODE_MAGONLY      = 0x02
OPMODE_GYROONLY     = 0x03
OPMODE_ACCMAG       = 0x04
OPMODE_ACCGYRO      = 0x05
OPMODE_MAGGYRO      = 0x06
OPMODE_IMUPLUS      = 0x08
OPMODE_COMPASS      = 0x09
OPMODE_M4G          = 0x0A
OPMODE_NDOF_FMC_OFF = 0x0B
OPMODE_NDOF         = 0x0C

class BNO055UARTNode(Node):
    def __init__(self):
        super().__init__('bno055_uart_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Open the FT232H UART
        self.uart = serial.Serial(port, baudrate=baud, timeout=1)

        # --- Retry sensor construction to survive a second launch ---
        max_tries = 3
        for attempt in range(1, max_tries + 1):
            try:
                self.sensor = adafruit_bno055.BNO055_UART(self.uart)
                break
            except RuntimeError as e:
                self.get_logger().warn(
                    f"BNO055 init attempt {attempt}/{max_tries} failed: {e!r} – retrying..."
                )
                self.uart.reset_input_buffer()
                time.sleep(0.1)
        else:
            raise RuntimeError(f"Could not initialize BNO055 after {max_tries} tries")

        # Allow the chip to boot
        time.sleep(1.0)

        # --- Calibrate X/Y/Z accel bias over 2 seconds in ACC-ONLY mode ---
        self.sensor.mode = OPMODE_ACCONLY
        time.sleep(0.05)

        x_samples, y_samples, z_samples = [], [], []
        calib_duration = 2.0
        t_end = time.time() + calib_duration
        self.get_logger().info(f"Calibrating accel bias for {calib_duration:.1f}s…")
        while time.time() < t_end:
            try:
                a = self.sensor.acceleration
            except RuntimeError:
                self.uart.reset_input_buffer()
                time.sleep(0.01)
                continue

            if a and None not in a:
                x_samples.append(a[0])
                y_samples.append(a[1])
                z_samples.append(a[2])
            time.sleep(1/20.0)

        if x_samples and y_samples and z_samples:
            raw_x = statistics.median(x_samples)
            raw_y = statistics.median(y_samples)
            raw_z = statistics.median(z_samples)
            self.accel_x_offset = raw_x
            self.accel_y_offset = raw_y
            # offset so that at rest linear_acceleration.z == -9.81
            self.accel_z_offset = raw_z - (-9.81)
        else:
            self.get_logger().warning(
                "No accel data collected—offsets set to zero, Z target -9.81"
            )
            self.accel_x_offset = self.accel_y_offset = 0.0
            self.accel_z_offset = 0.0

        self.get_logger().info(
            f"Computed accel offsets → X: {self.accel_x_offset:.3f}, "
            f"Y: {self.accel_y_offset:.3f}, Z-bias: {self.accel_z_offset:.3f} "
            "(static Z→-9.81)"
        )

        # --- Switch once into full 9-axis fusion (NDOF) mode ---
        self.get_logger().info("Switching BNO055 to NDOF fusion mode…")
        self.sensor.mode = OPMODE_NDOF
        time.sleep(0.05)

        ###### ─── NEW FILTER SETUP ─── ######
        # These variables hold the previous filtered value for each axis
        self.filt_ax = 0.0
        self.filt_ay = 0.0
        self.filt_az = 0.0

        # α = how much of the *old* filtered value to keep [0..1].
        # 0.8 means: filtered_new = 0.8*filtered_old + 0.2*raw
        # Feel free to tune between [0.5 .. 0.95] to get the right balance.
        self.accel_alpha = 0.8
        #####################################

        # --- ROS2 publisher & timer (50 Hz) ---
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1/20.0, self.publish_imu)

    def publish_imu(self):
        msg = Imu()

        # Read orientation, gyro, accel all in one try block
        try:
            q = self.sensor.quaternion
            g = self.sensor.gyro
            a = self.sensor.acceleration
        except RuntimeError as e:
            self.get_logger().warn(
                f"BNO055 UART read error in publish_imu: {e!r}. Flushing & skipping."
            )
            self.uart.reset_input_buffer()
            return

        # --- Orientation (fused quaternion) ---
        if q:
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = q

        # --- Angular velocity (°/s → rad/s) ---
        if g:
            msg.angular_velocity.x = math.radians(g[0])
            msg.angular_velocity.y = math.radians(g[1])
            msg.angular_velocity.z = math.radians(g[2])

        # --- Linear acceleration (zero X/Y and Z→-9.81) + FILTERING ---
        if a and None not in a[:3]:
            # 1) remove the static bias
            raw_ax = a[0] - self.accel_x_offset
            raw_ay = a[1] - self.accel_y_offset
            raw_az = a[2] - self.accel_z_offset

            # 2) exponential smoothing
            α = self.accel_alpha
            self.filt_ax = α * self.filt_ax + (1.0 - α) * raw_ax
            self.filt_ay = α * self.filt_ay + (1.0 - α) * raw_ay
            self.filt_az = α * self.filt_az + (1.0 - α) * raw_az

            # 3) assign the filtered result into the ROS msg:
            msg.linear_acceleration.x = self.filt_ax
            msg.linear_acceleration.y = self.filt_ay
            # negative sign here because BNO055’s “up” is +9.81, but ROS’s convention is
            # gravity is negative on z when stationary. Adjust if needed by your TF frames.
            msg.linear_acceleration.z = -self.filt_az

        # --- Covariances (unchanged) ---
        msg.orientation_covariance = [
            5.93538014e-05,  2.46092450e-05,  5.73900220e-05,
            2.46092450e-05,  9.40919295e-05, -6.95037616e-05,
            5.73900220e-05, -6.95037616e-05,  4.19494878e-04
        ]
        msg.angular_velocity_covariance = [
            9.87172793e-07,  1.88839813e-07, -5.99431703e-08,
            1.88839813e-07,  1.18246078e-06, -6.43276779e-08,
            -5.99431703e-08, -6.43276779e-08,  3.74352765e-06
        ]
        msg.linear_acceleration_covariance = [
            1.23037230e-04,  7.89407979e-06,  1.50405038e-05,
            7.89407979e-06,  1.33099373e-04,  4.11002608e-06,
            1.50405038e-05,  4.11002608e-06,  2.17352827e-04
        ]

        # --- Stamp & publish ---
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055UARTNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()