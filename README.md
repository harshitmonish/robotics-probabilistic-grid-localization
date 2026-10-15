# Probabilistic Grid Localization

This ROS 1 project maintains a 20 × 20 probability grid and updates it from motion-prediction or observation commands. The implementation uses fixed transition and observation probability arrays, renders map images with Matplotlib, and displays them with OpenCV.

## Contents

- The Python node in `src/` implements the probability-grid updates.
- The launch file in `launch/` starts the ROS node.
- `package.xml` and `CMakeLists.txt`: catkin package metadata.

## Requirements

ROS 1 with `rospy` and `std_msgs`, plus Python packages NumPy, Matplotlib, and OpenCV. The display code opens GUI windows, so it needs a graphical desktop. The program writes `map_plot.png` in its current working directory.

## Build and run

Place this repository, with `package.xml` at its root, in a catkin workspace's `src/` directory, then build and launch:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch <package-name> <launch-file>
```

The node subscribes to the `robot` topic (`std_msgs/String`). Commands are a direction letter followed by one digit: `R`, `L`, `U`, or `D` selects a motion prediction; `X` or `Y` selects an observation update. For example:

```bash
rostopic pub -1 /robot std_msgs/String "data: 'R1'"
rostopic pub -1 /robot std_msgs/String "data: 'X1'"
```

The included node also advertises `robot` but does not publish commands itself.

## Notes

This is an academic prototype and has not been verified for correctness or runtime stability. The node mutates probability arrays in place and starts a display thread; inspect these behaviors before using the output as a reliable localization estimate.
