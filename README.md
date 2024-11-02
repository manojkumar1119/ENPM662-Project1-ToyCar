# ENPM662: Introduction to Robot Modeling

## Project 1: CAD Modeling & Simulation using Gazebo

### Instructions for Running the Simulation

### Setup

1. **Clone the repository:** Clone the project repository into your chosen workspace directory.
   ```bash
   git clone git@github.com:manojkumar1119/ENPM662-Project1-ToyCar.git
   ```
2. **Make a ROS workspace:**
   ```bash
   mkdir -p ~/ros_ws/src
   ```
3. **Copy the package from the repo into the workspace**
   ```bash
   cd ENPM662-Project1-ToyCar/
   mv package/ ~/ros_ws/src
   mv package/ final/
   ```
4. **Build the package:** Build the ROS 2 package using the following command:
   ```bash
   colcon build --build-type ament_cmake final
   ```
   
5. **Source the setup:** Source the workspace setup script to make ROS 2 packages and tools available in your current terminal session:
   ```bash
   source install/setup.bash
   ```

### Spawning the Robot in Competition World
From the workspace directory, run the following command to launch the competition world in Gazebo and spawn your robot within it:
```bash
ros2 launch final competition.launch.py
```
### Running Teleoperation
After spawning the robot in Gazebo, open a new terminal window and run the following command to launch the teleoperation controller for your robot:
```bash
ros2 run final teleop.py
```
### Running Proportional Controller
After spawning the robot in Gazebo, open another new terminal window and run the following command to launch the proportional controller for your robot:
```bash
ros2 run final prop_controller.py
```


