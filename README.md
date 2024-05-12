# nav2_accountability_explainability_vlms
This repository includes a ROS 2 accountability and explainability solution based on the use of its topics, Rosbag files and VLMs. This approach aims to identify the causes that have triggered a set of specific events, providing to the final user a non-expert explanation.


# Software artifacts
ROS 2 Humble

RB1 simulator for ROS 2

[AWS RoboMaker Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world) World ROS package available [here](https://github.com/jmguerreroh/aws-robomaker-hospital-world/tree/ros2).

The following images show the floor plan of the scenario used in the development of this work, including in the second one an obstacle in the route initially calculated to reach the goal destination. This obstacle will force the change of the pre-computed path.

![imagen](https://user-images.githubusercontent.com/13176052/227868761-7df42f3d-9043-4b07-af27-2b843806be0e.png)

![imagen](https://user-images.githubusercontent.com/13176052/227868841-21b6f0e0-1017-4136-94aa-396ba1205a6b.png)


# Usage
## Accountability solution
After starting the simulation, black-box recording information service can be started with the command
```
ros2 launch bag_recorder bag_recorder.launch.py 
```
The previous recording service can be started and stopped by running
```
ros2 service call /bag_recorder std_srvs/srv/SetBool data:\ true
ros2 service call /bag_recorder std_srvs/srv/SetBool data:\ false
```
To start the recording process only when it would be an obstacle, it must be run
```
ros2 run bag_recorder bag_recorder_scan_client
```
To start the recording process only when a nav2 behavior tree node would be in FAILURE status, it must be run
```
ros2 run bag_recorder bag_recorder_btstatus_client
```
The navigation goal pose, can be sent by running
```
ros2 run goal_sender nav_to_pose_action_client 
```
The reproducibility of the recorded information can be tested with the command
```
ros2 bag play <path to rosbag file>
```
A sample of the recorded data can be found [here](https://drive.google.com/drive/folders/19dJu8P5nJbYrA8s7bAF5fEMyJEJk0LYS?usp=sharing).

## Explainability solution
```
ros2 launch explicability_bringup explicability_ros.launch.py
```
```
$ ros2 service call /question explicability_msgs/srv/Question "{'question': 'What is happening?'}"
```
