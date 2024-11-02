#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from matplotlib import pyplot as plt

# Data Logging Variables
TimeVector = [] 
Ctrl_Inputs = []
Error_Vector = []

class P_Controller(Node):
    """
    A ROS 2 Node for a Proportional Controller
    """

    def __init__(self):
        super().__init__('p_controller_node')  # Initialize the Node with a name
        self.get_logger().info('Proportional controller node initialized.')  # Log that the node is ready

        # Set up a QoS profile for reliability and message history
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Best effort delivery
            history=HistoryPolicy.KEEP_LAST,  # Keep only the last few messages
            depth=10  # Depth of message queue
        )

        # Publisher to send velocity commands to the velocity controller
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
       
        # Publisher to send joint position commands to the joint position controller
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Subscriber to get IMU data from the 'imu_plugin/out' topic
        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
        self.imu_sub  # Prevent unused variable warning

        # Timer to call the timer_callback function at regular intervals
        timer_period = 0.1  # Timer period in seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables
        self.counter = 0  # Counter to manage publishing duration
        self.timestamp = 0.0  # Initialize timestamp
        self.current_heading = 0.0  # Initialize current heading

    def imu_callback(self, msg):
        """
        Callback function for the IMU subscriber
        """
        # Extract timestamp from IMU message
        imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        # Extract the yaw (z-orientation) from the IMU message
        current_heading = msg.orientation.z

        # Update the class variables with the received values
        self.timestamp = imu_timestamp
        self.current_heading = current_heading

    def timer_callback(self):
        """
        Timer callback function to compute and publish control commands
        """
        K = 10  # Proportional gain for the controller

        # Desired heading in radians (adjusted for IMU error)
        des_phi = 0.375

        # Set a constant forward velocity
        linear_vel = 3.0
        # Initialize the steering angle
        steer_angle = 0.0

        # Calculate the error between the desired and current heading
        phi_error = des_phi - self.current_heading
        # Compute the control input using the proportional controller
        steer_angle = -K * phi_error

        # Apply joint limits to prevent excessive steering
        steer_angle = max(min(steer_angle, 1.0), -1.0)

        # Initialize messages for publishing
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()

        # If within the desired duration, set wheel and joint commands
        if self.counter < 638:
            wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel]
            joint_positions.data = [steer_angle, steer_angle]
        else:
            # Stop the robot after the desired duration
            wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
            joint_positions.data = [0.0, 0.0]

        # Publish the commands
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

        # Log the current heading and control input
        self.get_logger().info(f'Current Heading: {self.current_heading}, Steer Angle: {steer_angle}')

        # Append data for plotting later
        TimeVector.append(self.timestamp)
        Ctrl_Inputs.append(steer_angle)
        Error_Vector.append(phi_error)

        # Increment the counter
        self.counter += 1

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node
    """
    print('Starting Proportional Control: Moving from (0,0) to (10,10)')
    rclpy.init(args=args)  # Initialize ROS 2

    # Create the controller node
    controller = P_Controller()

    try:
        # Keep the node running
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully

    # Cleanup and shutdown
    controller.destroy_node()  # Destroy the node
    rclpy.shutdown()  # Shutdown ROS 2

    # Plot the results after the node is stopped
    print('End of monitoring: prepare for graphs!')

    # Plot Yaw Error vs. Time
    plt.title('Yaw Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Error (rad)')
    plt.plot(TimeVector, Error_Vector, 'b-', label='Yaw Error')
    plt.legend()
    plt.show()

    # Plot Steer Angle vs. Time
    plt.title('Steer Angle vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Steer Angle (rad)')
    plt.plot(TimeVector, Ctrl_Inputs, 'b-', label='Steer Angle')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
