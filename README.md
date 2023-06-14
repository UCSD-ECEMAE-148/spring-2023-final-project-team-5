<div id="top"></div>

<h1 align="center">Obstacle Avoidance and Pedestrian Detection with ROS2</h1>

<br />
<div align="center">
    <img src="images\UCSDLogo.png" alt="Logo" width="400" height="100">
  </a>
<h3>ECE-MAE 148 Final Project</h3>
<p>
Team 5 Spring 2023
</p>
</div>

**insert image of robot**

## Team Members
Shasta Subramanian (ECE)

Armond Greenberg (MAE)

Jacob Cortez (MAE)

Zixu Hao (ECE - UPS Student)

## Abstract
The goals of our team's final project was implementing obstacle avoidance and pedestrian detection on top of the lane following program. Using the Lidar and OakD Lite Camera, the robot is able to closely follow the yellow-dotted line on the track, make optimal turning decisions to avoid any objects in the pathway, and stop if a pedestrian is detected.

## What We Promised
* Obstacle avoidance where if the obstacle is not a person the robot swerves around and continues on its path using LIDAR
* If a pedestrian is detected the robot will stop until the person is no longer detected using the OakD Lite Camera
* The previous two objectives would also be achieved while line following and within ROS2

## Challenges
* Combining our obstacle avoidance program on the track with the pedestrian detection proved to be more complicated than initially expected
* Adapting the various nodes, creating unique publishers/subscribers, and implementing all of our code within ROS2

## Accomplishments
* Achieved all of our promised goals successfully 
* Completed project within ROS2
* Refined obstacle avoidance algorithm
* Extremely accurate person detection (almost too good)
* * Works on real humans and print out images

## Gantt Chart
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/ac5c8ead-56f3-41d3-8ed3-f4c0331c360c)

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
## Baseplate
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/26fd8e2c-5acf-48f4-9e47-6c7ec2a96f17)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/e6602891-79b6-4dc2-9041-1f3944957bf0)
### *Team 5's baseplate used for mounting electronics and other components (Isometric and Top View)*
## Jetson Nano Case
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/2102a021-ed98-42e6-920e-41a2522bfe3f)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/63896700-373c-46b1-9988-9b9c2bb5c747)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/9ef2de13-563a-4913-a153-63b0cf8d14c7)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/17c07153-c1a1-49b8-80e3-6d8203097c39)
### *Team 5's Jetson Nano Case from top to bottom (Isometric, Top, Right, and Front View). Credit to yun3d at https://www.thingiverse.com/thing:3778338*

## Camera Mount
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/5c09cbd4-6bd3-4c49-9e59-29f13cd2f1fd)
### *Credit for this portion of the camera mount to Matstic at https://www.thingiverse.com/thing:5336496*
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/8272498f-241f-45ee-80ac-fcd6efa82b10)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/dfee34c8-abdf-4bed-a303-4a3d9dcf1e5c)

![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/979798ce-7823-4885-bd02-62e6fe5d953e)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/fd08ccc0-b16f-4b23-b3a5-b96e650e1717)
### *Team 5's Camera Mount from top to bottom (Isometric, Right, Front, and Top View).*
## Our Robot
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/4e3ba3bd-a8e5-476e-af54-031f0d7d162b)
### *Team 5's fully assembled robot that utilizes an angled down Oak D Lite and LIDAR*

## Circuit Diagram
<img width="689" alt="image" src="https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/114700732/3ea9eb5f-e87d-42db-82ef-71abc0a1a276">

### * Team 5 followed the circuit diagram given by the instructors in the documentation provided *

## Software Documentation

## Autonomous Laps
### Here are our autonomous laps as part of our class deliverables and preparation for the final project:

* Lane detection using OpenCV + ROS2: https://youtu.be/ensYDWS0fc4
* Inner lane: https://drive.google.com/file/d/1SGNKMuTuL6o_IKrJB7Sfbhk6654geIMa/view?usp=drive_link
* Outer lane: https://drive.google.com/file/d/1BtGmwQEgpboFYSuyDKBHTyqufXaYvAOl/view?usp=drive_link
* GPS: https://youtu.be/IlAwR1aKfdU

## Acknowledgements
Special thanks to Professor Jack Silberman and TAs (Kishore Nukala & Moises Lopez) and  for all of your support!


