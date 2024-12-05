##      📔 HOMEWORK 3
This repository has been created in order to fulfill the third homework of RoboticsLab 2024/2025 class. 

## 💻 Usage 
###      🔨 Build
First of all, clone this repository in your ros2_ws/src folder
```
git clone https://github.com/PasFar/Homework-3.git
```
Then, build the packages using the following command inside the ros2_ws folder and source the setup.bash file 
```
colcon build
. install/setup.bash
```
### ✅ How to run
Now you can run the nodes. You have different options.

To test the object detection of the spherical object: 
   ```
ros2 launch iiwa_bringup iiwa.launch.py use_vision:="true" blob_detection:="true"
   ```
This will spawn the manipulator and the spherical object in the gazebo simulation.
Then, open another terminal, connect to the same docker container and run:
   ```
   . install/setup.bash
   ros2 run ros2_opencv ros2_opencv_node
   ```
This will start the object detection. To see the results you can open rqt and check the /processed_image topic.
You should see a pink circle around all the detected objects as shown in the report. 

For the second part of the project you can test the vision based tasks.
In order to run the positioning task with velocity commands you have to run:
   ```
ros2 launch iiwa_bringup iiwa.launch.py use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller"
   ```
Then, open another terminal, connect to the same docker container and run:
   ```
   . install/setup.bash
   ros2 launch aruco_ros single.launch.py marker_id:=201 marker_size:=0.1
   ```
This will start the aruco marker detection (you can check it on rqt from the topic /aruco_single/result).

Again, open another terminal, connect to the same docker container and run:
   ```
   . install/setup.bash
   ros2 launch ros2_kdl_package kdl_vision.launch.py
   ```
The manipulator will move until facing the aruco marker and reaching the marker position with a default offset of: [x_off=0.3 , y_off=0.1 , z_off=0.0], as shown in the report.
The position and orientation offset can be set in the ros2_kdl_package/config/params.yaml by modifying the params called "_offset"

In order to change the task you need to modify the _task_ param from the .yaml file and switch to from "positioning" to "look_at_point".
Then build and run again the three previous commands.

In this case the manipulator will face the center of the marker and move accordingly to it (example shown in the attatched video).

If you want to use the positioning or look-at-point task with an effort controller you need to change the _cmd_interface_ param from the .yaml and switch to "effort_j" if you want to use a joint space controller or "effort_o" if you want to use the operational space one.
Once you did this changes you can run:
```
colcon build
. install/setup.bash
ros2 launch iiwa_bringup iiwa.launch.py use_vision:=true command_interface:="effort" robot_controller:="effort_controller"
```
And then open and connect to the docker other two terminals and run the same commands we saw above for starting aruco detection and kdl_vision.

In order to use a merged controller for a look-at-point vision-based task, you can:
 
 - Use an inverse dynamics controller in the joint space or in the Cartesian space by changing the _cmd_interface_ as previously explained 

 - Accordingly change the _task_ parameter in the paramas.yaml file in "merge_j" if you selected the effort_j as _cmd_interface_, otherwise "merge_o" if you are using effort_o

Once you did all the desired changes you have to run:
   ```
   colcon build
   ```
And then again:
   ```
   . install/setup.bash
   
   ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
   ```
In a different terminal, connected to the same docker container, you run:
   ```
   . install/setup.bash
   ros2 launch aruco_ros single.launch.py marker_id:=201 marker_size:=0.1
   ```
And finally, in a third terminal:
   ```
   . install/setup.bash
   ros2 launch ros2_kdl_package kdl_vision.launch.py
   ```
The robot should complete a linear trajectory while properly adjusting its orientation to face the marker.
