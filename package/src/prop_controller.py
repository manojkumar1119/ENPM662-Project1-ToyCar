#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from matplotlib import pyplot as plt

# Data Logging Variables
time_log = []
steering_commands = []
heading_errors = []

# Initialize position tracking
x_positions = []
y_positions = []

class ProportionalController(Node):  # Define the Proportional Controller Node
    def __init__(self):  # Init Function
        super().__init__('proportional_controller_node')  # Naming the Node
        self.get_logger().info('Proportional controller node started.')  # Confirmation that the Node is running
        
        qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Create publishers for wheel velocities and joint positions
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.position_publisher = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
        self.imu_subscription  # Prevent unused variable warning

        timer_frequency = 0.1  # Publish frequency
        self.timer = self.create_timer(timer_frequency, self.timer_callback)  # Initialize Timer Callback

        self.step_counter = 0  # Initialize Step Counter
        self.max_steps = 1200  # Set a limit for movement

    def imu_callback(self, msg):  # Define the Callback Function
        imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec) / 1e9  # Grab the time of record
        current_yaw = msg.orientation.z  # Grab the robot's current yaw

        self.timestamp = imu_timestamp  # Update the node's timestamp
        self.current_yaw = current_yaw  # Update the robot's current heading

    def timer_callback(self):  # Defining the Timer Callback Function
        K = 5  # Proportional Controller Gain

        # Desired heading
        desired_yaw = 0.3  # Adjusted for IMU Errors

        linear_velocity = 3.0  # Constant velocity profile
        steering_angle = 0.0  # Initialize steer angle to 0 rad

        # Calculate the error between the current heading and desired heading
        yaw_error = desired_yaw - self.current_yaw
        steering_angle = -K * yaw_error  # Generate Control Input

        # Implement joint limits for steering angle
        steering_angle = max(min(steering_angle, 1.0), -1.0)

        # Initialize Messages
        velocity_commands = Float64MultiArray()
        position_commands = Float64MultiArray()

        # Implement stop command based on counter limit for movement
        if self.step_counter < self.max_steps:
            velocity_commands.data = [linear_velocity, -linear_velocity, linear_velocity, -linear_velocity, linear_velocity, -linear_velocity, linear_velocity, -linear_velocity]
            position_commands.data = [steering_angle, steering_angle, steering_angle]
        else:
            # Stop the robot after reaching the desired trajectory
            velocity_commands.data = [0.0] * 8
            position_commands.data = [0.0, 0.0, 0.0]

        # Publish joint and velocity commands
        self.position_publisher.publish(position_commands)
        self.velocity_publisher.publish(velocity_commands)

        # Log data for plotting
        time_log.append(self.timestamp)
        steering_commands.append(steering_angle)
        heading_errors.append(yaw_error)

        # Track positions
        if self.step_counter < self.max_steps:
            # Simulate movement along a straight line
            x_position = (self.step_counter / self.max_steps) * 10  # Move from 0 to 10
            y_position = (self.step_counter / self.max_steps) * 10  # Move from 0 to 10
            x_positions.append(x_position)
            y_positions.append(y_position)

        self.step_counter += 1  # Increase Counter

def main(args=None):  # Defining the 'Main' Function
    print('Moving from (0,0) ---> (10,10)...')  # Confirmation of movement
    rclpy.init(args=args)

    controller = ProportionalController()  # Establish the Publisher Node
    try:
        rclpy.spin(controller)  # Spin the Publisher Node
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()  # Shut the Node Down

    # After Ctrl+C, Plot the Results
    print('Movement completed. Plotting results...')

    # Plot the trajectory from (0, 0) to (10, 10)
    plt.figure()
    plt.title('Trajectory from (0,0) to (10,10)')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.plot(x_positions, y_positions, 'bo-', markersize=2)  # Plot the trajectory
    plt.grid()
    plt.axis('equal')  # Equal aspect ratio for clarity
    plt.show()


if __name__ == '__main__':
    main()

