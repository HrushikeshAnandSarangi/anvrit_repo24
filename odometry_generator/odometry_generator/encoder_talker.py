import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import numpy as np
from KalmanFilter import KalmanFilter

class EncoderTalker(Node):
    def __init__(self):
        super().__init__('encoder_talker')
        self.publisher_ = self.create_publisher(Int32, 'filtered_encoder_data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Kalman filter initialization
        h = 0.1  # discretization step
        A = np.matrix([[1, h, 0.5*(h**2)], [0, 1, h], [0, 0, 1]])  # State transition matrix
        B = np.matrix([[0], [0], [0]])  # Control matrix (no control input)
        C = np.matrix([[1, 0, 0]])  # Measurement matrix (only position)
        R = np.matrix([[1]])  # Measurement noise covariance
        Q = np.matrix(np.zeros((3, 3)))  # Process noise covariance (assume no noise for simplicity)

        x0 = np.matrix([[0], [0], [0]])  # Initial state guess (position, velocity, acceleration)
        P0 = np.matrix(np.eye(3))  # Initial guess of error covariance

        # Create Kalman filter object
        self.kalman_filter = KalmanFilter(x0, P0, A, B, C, Q, R)
        self.inputValue = np.matrix([[0]])  # No control input for this system

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            try:
                # Read the raw encoder value
                encoder_value = int(line)
                
                # Kalman filter update step
                self.kalman_filter.propagateDynamics(self.inputValue)
                self.kalman_filter.computeAposterioriEstimate(encoder_value)

                # Get the filtered position estimate from the Kalman filter
                filtered_position = self.kalman_filter.estimates_aposteriori[-1][0, 0]

                # Publish the filtered encoder data
                self.get_logger().info(f'Publishing filtered position: {filtered_position}')
                self.publisher_.publish(Int32(data=int(filtered_position)))

            except ValueError:
                self.get_logger().warn(f'Invalid data received: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderTalker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

