# RoboMaster Dual Robot Controller and Track Runner

This project provides launch and run commands for different simulation scenarios including single robot tracks (circuit, diagonal, Monza, Silverstone, straight line) and dual robot control for Y-shape paths.

---

## Running Single Robot Scenarios (Circuit, Diagonal, Monza, Silverstone, Straight Line)

1. **Start CoppeliaSim Simulation**
    ```bash
    pixi run coppelia
    ```

2. **Launch ROS2 Environment**
    ```bash
    ros2 launch robomaster_ros s1.launch
    ```

3. **Run Standard Track Controller**
    ```bash
    ros2 launch robomaster_example standard.launch name:=/rm0
    ```

---

## Running Dual Robot Scenario

1. **Run Controller for Robot 1**
    ```bash
    ros2 run robomaster_example controller_node --ros-args \
      -r /cmd_vel:=/rm1/cmd_vel \
      -r /odom:=/rm1/odom \
      -r /left_vision:=/rm1/left_vision \
      -r /middle_vision:=/rm1/middle_vision \
      -r /right_vision:=/rm1/right_vision
    ```

2. **Run Controller for Robot 2**
    ```bash
    ros2 run robomaster_example controller_node --ros-args \
      -r /cmd_vel:=/rm2/cmd_vel \
      -r /odom:=/rm2/odom \
      -r /left_vision:=/rm2/left_vision \
      -r /middle_vision:=/rm2/middle_vision \
      -r /right_vision:=/rm2/right_vision
    ```

---

## Running Dual Robot Y-Shape Path

Run the Dual Robot Controller:
```bash
ros2 run robomaster_example dual_robot_controller
