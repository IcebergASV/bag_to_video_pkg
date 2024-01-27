# bag_to_video_pkg
## Description
The `bag_to_video_pkg` is a ROS (Robot Operating System) package designed to convert camera data from a ROS bag file into a video file. A ROS bag is a file format in ROS for storing ROS message data. These bags are primarily used for logging sensor data and messages transmitted over ROS topics and are very useful for capturing a snapshot of the system's state for later analysis and visualization.

This package specifically listens to a camera topic within a ROS bag file and saves the image data as a video file. It's useful for situations where you need to extract and view camera footage from a robotic system for analysis or record-keeping.

## Prerequisites

- ROS Melodic (or compatible version)
- Python 2.7
- OpenCV
- cv_bridge

## Installation and Building

1. Navigate to your ROS workspace (e.g., `~/catkin_ws/src`).
2. Clone or copy the `bag_to_video_pkg` into the `src` directory.
3. Navigate back to the root of your workspace (e.g., `~/catkin_ws`) and run `catkin_make` or `catkin build` depending on how you iniralized your catkin_ws.
4. Source the environment: `source devel/setup.bash`.


## RUnning the Package
1. To launch the package, use the following command:
   ```bash
   roslaunch bag_to_video_pkg bag_to_video.launch
2. In a separaete terminal, play the ROS bag file you want to process:
   ```bash
   rosbag play <path_to_your_bag_file.bag>
   
Ensure that the ROS environment is sourced in this terminal as well.

## Topics
The package subscribes to the following topic to receive camera data:
- **/camera/color/image_raw**: This topic is expected to provide images from the camera in a raw format.

## Output
- The script saves the output video in the root directory of the package as output.avi. You can modify the script to change the format, filename, or saving directory.

## Customization
To customize the package for different topics or different video settings, edit the **bag_to_video.py** script located in the **scripts** directory of the package.
