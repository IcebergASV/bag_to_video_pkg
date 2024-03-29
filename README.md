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


## Running the Package
1. To launch the package, use the following command:
   ```bash
   roslaunch bag_to_video_pkg bag_to_video.launch
2. In a separaete terminal, play the ROS bag file you want to process:
   ```bash
   rosbag play <path_to_your_bag_file.bag>
3. A new window will pop open and you can visualize the ros bag contents at the same time
4. To verify everything is working and the images are being converted you can run the following command in a new window and visualize the ros logs of the script.
   ```bash
   rostopic echo /rosout

   
Ensure that the ROS environment is sourced in this terminal as well.

## Topics
The package subscribes to the following topic to receive camera data:
- **/camera/color/image_raw**: This topic is expected to provide images from the camera in a raw format.

## Output
- The script saves the output video in the root directory of the package as output.avi. You can modify the script to change the format, filename, or saving directory.

## Customization
To customize the package for different topics or different video settings, edit the **bag_to_video.py** script located in the **scripts** directory of the package. Modify where you would like the *output_directory* variable point at. 

## Ubuntu Video Editors
### Kdenlive for Editing ROS Bag Videos
After converting ROS bag files to video format using the **bag_to_video_pkg**, you might need to edit these videos for various purposes, such as trimming unnecessary footage or highlighting specific segments. For Ubuntu users, a highly recommended video editing tool is **Kdenlive**.

#### Why Kdenlive?
- **Compatibility**: Kdenlive is well-suited for Ubuntu and integrates seamlessly with its desktop environment.
- **Features**: It offers a wide range of editing features which are perfect for handling videos converted from ROS bag files.
- **Ease of Use**: Kdenlive has a user-friendly interface that is suitable for both beginners and professional editors.

#### Installing Kdenlive
1. Kdenlive can be easily installed from the Ubuntu Software Store. Follow these steps:
2. Open the Ubuntu Software Store.
Search for "Kdenlive".
3. Click on the Kdenlive application and press "Install".

#### Editing ROS Bag Videos
Once you have Kdenlive installed, you can use it to edit the videos created by the **bag_to_video_pkg**. This can be particularly useful for:
- Removing unnecessary footage from the video.
- Enhancing video quality by adjusting color, contrast, and brightness.

#### Steps to Edit Videos
1. Launch Kdenlive.
2. Import your converted video file (now in MP4 format) into Kdenlive.
3. Use Kdenlive's timeline and tools to trim or adjust the video as needed.
4. Export the edited video in your desired format. Click this link for an [mp4](https://www.youtube.com/watch?v=u-wAiKWEhTE) format.


## Next Steps?
If you want to create a custom dataset from the mp4 video you created and need to take many screenshots, check out this [repository](https://github.com/DavidGuamanDavila/auto_screenshot_util/tree/main), which automates screenshot-taking. 
