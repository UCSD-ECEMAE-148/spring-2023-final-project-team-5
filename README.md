<div id="top"></div>

<h1 align="center">Obstacle Avoidance and Pedestrian Detection with ROS2</h1>

<br />
<div align="center">
    <img src="images\UCSDLogo.png" alt="Logo" width="400" height="100">
    <h3>ECE-MAE 148 Final Project</h3>
    <p>
    Team 5 Spring 2023
    </p>
</div>

<div align="center">
    <img src="images\ourRobot.PNG" width="500" height="400">
</div>
<br>
<hr>
<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#final-project-videos">Final Project Videos</a></li>
    <li><a href="#software">Software</a></li>
        <ul>
        <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        <li><a href="#pedestrian-detection">Pedestrian Detection</a></li>
      </ul>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
  </ol>
</details>
<hr>

## Team Members
Shasta Subramanian (ECE) - [LinkedIn](https://www.linkedin.com/in/shasta-subramanian/)

Armond Greenberg (MAE)

Jacob Cortez (MAE)

Zixu Hao (ECE - UPS Student)
<hr>

## Abstract
The baseline goals of our team's final project were implementing obstacle avoidance and pedestrian detection on top of the lane following program. Using the Lidar and OakD Lite Camera, the robot is able to closely follow the yellow-dotted line on the track, make optimal turning decisions to avoid any objects in the pathway, and stop if a pedestrian is detected.
<hr>

## What We Promised
* Obstacle avoidance where if the obstacle is not a person the robot swerves around and continues on its path using LIDAR
* If a pedestrian is detected the robot will stop until the person is no longer detected using the OakD Lite Camera
* The previous two objectives would also be achieved while line following and within ROS2
<hr>

## Accomplishments
* Achieved all of our promised goals successfully 
* Completed project within ROS2
* Refined obstacle avoidance algorithm
* Extremely accurate person detection (almost too good)
  * Works on real humans and print out images
<hr>

## Challenges
* Combining our obstacle avoidance program on the track with the pedestrian detection proved to be more complicated than initially expected
* Adapting the various nodes, creating unique publishers/subscribers, and implementing all of our code within ROS2
<hr>

## Final Project Videos
**Final Clips**

Everything Together (3rd Person)

[<img src="images\everything.PNG" width="300">](https://drive.google.com/file/d/1p2i5tRBihoJaQEioC826KT2X-poMZBU0/view?usp=drive_link)

Everything Together (POV)

[<img src="images\pov.PNG" width="300">](https://drive.google.com/file/d/1Jt7cPzaUdMSlrA__SZ71dYU6QvQGNySb/view?usp=drive_link)

Obstacle Avoidance

[<img src="images\earlyAvoid.PNG" width="300">](https://drive.google.com/file/d/1Pd8OklPu9tDEfgzJNNkNkR_zGN-SYbMb/view?usp=drive_link)

Pedestrian Detection

[<img src="images\personDetected.PNG" width="300">](https://drive.google.com/file/d/11qNUHqOkSNa8mivNA_k-UFk8U7LfdoAp/view?usp=drive_link)

**Progress Clips**

Early Obstacle Avoidance

[<img src="images\earlierAvoid.PNG" width="300">](https://drive.google.com/file/d/14KP8B8-IEhhGi5ObEtc7LC3ndSPOTO66/view?usp=drive_link)

Early Pedestrian Detection

[<img src="images\earlyPed.PNG" width="300">](https://drive.google.com/file/d/1O3riZQaE1dmO9sFID_FuAqzC17s5VZ9x/view?usp=drive_link)

<hr>

## Software

### Overall Architecture
Our project was completed entirely with ROS2 navigation in python. The 'rclpy' package is being used to control the robot and our primary control logic consists of the Calibration, Person Detection, Lane Detection, Lane Guidance, and nodes.

- The **Calibration Node** was adapted from Spring 2022 Team 1 and updated for our use case. We strictly needed the gold mask to follow the yellow lines and implemented our own lane following code.
  
- The **Person Detection Node** was full created for our team's project implementation. We create a new oakd node that sends the same image used by the depthai package for person detection to our guidance node so that the pedestrian detection can run concurrently with the line following. 

- The **Lane Detection Node** is used to control the robot within the track. We adapt the PID function to calculate the error and set new values to determine the optimal motion of the car to continue following the yellow lines in the lane. This is done by taking the raw camera image, using callibrated color values to detect yellow, and ultimately using the processed image to publish the control values that are are subscribed by the lane guidance node.

- Ultimately, the "magic" happens within the **Lane Guidance Node** which is responsible for directly controlling car's movement. We have adapted the Lidar subscription from Spring 2022 Team 1 to detect obstacles within a particular viewing range in front of our car. The lane guidance node subscribes to lane detection node and our Person Detection nodes to correctly traverse the path. If no obstacles are detected, the car will simply continue its line following program, sticking to the yellow lines in the middle of the lane. If an obstacle is detected by the lidar, the car will correspondingly make a turn based on the object's angle and distance. As it routes around the object, the car continues to check for obstacles to avoid any collision and come back to the path. Additionally, if the subscription to the person_detected node is triggered as active, then the car knows there is a pedestrian in view and will stop.

### Obstacle Avoidance
We used the LD06 Lidar to implement obstacle avoidance within ROS2. The program logic is quite simple in that we are constantly scanning the 60 degrees in front of the robot. If an object is detected within our distace threshold, the robot will accordingly make a turn to avoid it. Our logic for selecting which direction to turn in is quite simple in that if the object is on the left side, we first turn right and otherwise we turn left. Both turning directions include a corrective turn to bring the robot back to the centerline of the track and continue lane following.

### Pedestrian Detection
We used the DepthAI package to implement the pedestrian detection within ROS2. We took advantage of the Tiny YOLO neural network setup found within the examples. We filter through the detections to check strictly for a "person" with adjustable confidence levels. We found that a 60% confidence level worked pretty well for our project's use cases. Surprisingly, we found better results with real humans walking in front of the robot (it would detect their feet and be able to classify them as "person" objects). We were also able to successfully scan various printout images of people with high accuracy and success. The programming logic for the pedestrian detection is very simple in that if a "person" has been detected in the image passed through by the camera, the VESC throttles are set to 0, stopping the car, until the person has moved out of the field of view. 
<hr>

## Hardware 

* __3D Printing:__ Camera Mount, Jetson Nano Case
* __Laser Cutting:__ Base plate to mount electronics and other components.

__Parts List__

* Traxxas Chassis with steering servo and sensored brushless DC motor
* Jetson Nano
* WiFi adapter
* 64 GB Micro SD Card
* Adapter/reader for Micro SD Card
* Logitech F710 controller
* OAK-D Lite Camera
* LD06 Lidar
* VESC
* Anti-spark switch with power switch
* DC-DC Converter
* 4-cell LiPo battery
* Battery voltage checker/alarm
* DC Barrel Connector
* XT60, XT30, MR60 connectors

*Additional Parts used for testing/debugging*

* Car stand
* USB-C to USB-A cable
* Micro USB to USB cable
* 5V, 4A power supply for Jetson Nano

__Baseplate__

<img src="images\baseplate.PNG" height="300">
<!--![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/e6602891-79b6-4dc2-9041-1f3944957bf0)-->

__Jetson Nano Case__

<img src="images\jetsonCase.PNG" height="300">
<!--![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/63896700-373c-46b1-9988-9b9c2bb5c747)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/9ef2de13-563a-4913-a153-63b0cf8d14c7)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/17c07153-c1a1-49b8-80e3-6d8203097c39)-->

Credit: https://www.thingiverse.com/thing:3778338

__Camera Mount__

<img src="images\cameraMount.PNG" height="300">

Credit: https://www.thingiverse.com/thing:5336496
<!--![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/5c09cbd4-6bd3-4c49-9e59-29f13cd2f1fd)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/dfee34c8-abdf-4bed-a303-4a3d9dcf1e5c)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/979798ce-7823-4885-bd02-62e6fe5d953e)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/fd08ccc0-b16f-4b23-b3a5-b96e650e1717)-->

__Circuit Diagram__

<img src="images\circuitDiagram.PNG" height="300">

<hr>

## Gantt Chart

<img src="images\projectGantt.PNG" height="300">
<hr>

## Course Deliverables
Here are our autonomous laps as part of our class deliverables and preparation for the final project:

* Lane detection using OpenCV + ROS2: https://youtu.be/ensYDWS0fc4
* Inner lane: https://drive.google.com/file/d/1SGNKMuTuL6o_IKrJB7Sfbhk6654geIMa/view?usp=drive_link
* Outer lane: https://drive.google.com/file/d/1BtGmwQEgpboFYSuyDKBHTyqufXaYvAOl/view?usp=drive_link
* GPS: https://youtu.be/IlAwR1aKfdU

Here are our presentation slides for the weekly project updates and final presentation:
[Team 5 Presentation](https://docs.google.com/presentation/d/1sPPWAnGMisoc15QvhqSP7sxDoCSJ5Mn_mOzy9liQnEs/edit?usp=sharing)
<hr>

## Project Reproduction
If you are interested in reproducing our project, here are a few steps to get you started with our repo:

<ol>
  <li>Clone this repository</li>
  <li>Replace the <i>ucsd_robocar_sensor2_pkg</i> and <i>ucsd_robocar_lane_detection2_pkg</i> in the default <i>ucsd_robocar_hub2</i> directory</li>
  <li> Calibrate Your Robot
    <ol>
      <li>Toggle <i>camera_nav_calibration</i> to 1 and <i>camera_nav</i> to 0 within <i>node_config.yaml</i></li>
      <li>Run <i>source_ros2</i>, <i>build_ros2</i>, and then <i>ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py</i> </li>
      <li>Adjust sliders within GUI to ensure gold mask is clear with <b>NO</b> noise </li>
      <li>Toggle <i>camera_nav_calibration</i> to 0 and <i>camera_nav</i> to 1 within <i>node_config.yaml</i></li>
      <li>Update your PID and throttle values in <i>ros_racer_calibration.yaml</i></li>
    </ol>
  </li>
  <li>Run on Track</li>
    <ol>
        <li>Run <i>source_ros2</i>, <i>build_ros2</i>, and then <i>ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py</i> </li>
    </ol>
</ol>

Alternatively you can refer to the `lane_guidance_node.py` and `lane_detection_node.py` programs in `ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg` to adapt our code as needed for your project. We have extensive comments through the code explaining what is happening. Additionally, if you search for <i>(Edit as Wanted)</i> in our code, we have listed the primary areas where one would want to adjust parameters to adapt the lidar usage, pedestrian detection logic, and more.

<hr>

## Acknowledgements
Special thanks to Professor Jack Silberman and TAs (Kishore Nukala & Moises Lopez) for all the support!

**Programs Referenced:**
* [Spring 2022 Team 1](https://guitar.ucsd.edu/maeece148/index.php/2022SpringTeam1)
* [DepthAI](https://github.com/luxonis/depthai-python)
* [UCSD Robocar Framework](https://gitlab.com/ucsd_robocar2)

<hr>

## Contacts

* Shasta Subramanian - s1subram@ucsd.edu | shasta.subramanian@gmail.com
* Armond Greenberg - argreenb@ucsd.edu
* Zixu Hao - ax008324@ucsd.edu
* Jacob Chandler - j1cortez@ucsd.edu


