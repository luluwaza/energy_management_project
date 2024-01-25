## Prerequisites

To run this code, you'll need:

- ROS2
- Colcon
- Python 3
- `rclpy` library for ROS2 in Python. Install it using:

    ```
    pip install -U rclpy
    ```

## Instructions

1. **Clone the Repository**

    Clone or download the repository to your local machine.

2. **Building and Sourcing the Workspace**
    Get into your ros2_ws/robotics_project_final_work and run:
    ```bash
    colcon build
    ```
    Get into your install folder and run:
    ```bash
    source install/setup.bash
    ```

3. **Launch the ROS2 Nodes**

    A runnable launch file exists for future development, however, the launch file doesn't ask for inputs and users only run one node which can be ran directly using run:

    ```bash
    ros2 run final_project_energy show_energy
    ```

4. **Incase you'd like to copy paste everything at once**

    ```bash
    cd ~/ros2_ws/robotics_project_final_work
    colcon build
    cd ../install
    source install/setup.bash
    ros2 run final_project_energy show_energy
    ```