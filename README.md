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
  </ol>
</details>

## Team Members
Shasta Subramanian (ECE) - [LinkedIn](https://www.linkedin.com/in/shasta-subramanian/)

Armond Greenberg (MAE)

Jacob Cortez (MAE)

Zixu Hao (ECE - UPS Student)

## Abstract
The baseline goals of our team's final project were implementing obstacle avoidance and pedestrian detection on top of the lane following program. Using the Lidar and OakD Lite Camera, the robot is able to closely follow the yellow-dotted line on the track, make optimal turning decisions to avoid any objects in the pathway, and stop if a pedestrian is detected.

## What We Promised
* Obstacle avoidance where if the obstacle is not a person the robot swerves around and continues on its path using LIDAR
* If a pedestrian is detected the robot will stop until the person is no longer detected using the OakD Lite Camera
* The previous two objectives would also be achieved while line following and within ROS2

## Accomplishments
* Achieved all of our promised goals successfully 
* Completed project within ROS2
* Refined obstacle avoidance algorithm
* Extremely accurate person detection (almost too good)
  * Works on real humans and print out images

## Challenges
* Combining our obstacle avoidance program on the track with the pedestrian detection proved to be more complicated than initially expected
* Adapting the various nodes, creating unique publishers/subscribers, and implementing all of our code within ROS2

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


## Software
### Obstacle Avoidance
We used the LD06 Lidar to implement obstacle avoidance within ROS2. The program logic is quite simple in that we are constantly scanning the 60 degrees in front of the robot. If an object is detected within our distace threshold, the robot will accordingly make a turn to avoid it. Our logic for selecting which direction to turn in is quite simple in that if the object is on the left side, we first turn right and otherwise we turn left. Both turning directions include a corrective turn to bring the robot back to the centerline of the track and continue lane following.

### Pedestrian Detection
We used the DepthAI package to implement the pedestrian detection within ROS2. We took advantage of the Tiny YOLO neural network setup found within the examples. We filter through the detections to check strictly for a "person" with adjustable confidence levels. We found that a 60% confidence level worked pretty well for our project's use cases. Surprisingly, we found better results with real humans walking in front of the robot (it would detect their feet and be able to classify them as "person" objects). We were also able to successfully scan various printout images of people with high accuracy and success. The programming logic for the pedestrian detection is very simple in that if a "person" has been detected in the image passed through by the camera, the VESC throttles are set to 0, stopping the car, until the person has moved out of the field of view. 

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
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/26fd8e2c-5acf-48f4-9e47-6c7ec2a96f17)
<!--![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/e6602891-79b6-4dc2-9041-1f3944957bf0)-->

__Jetson Nano Case__
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/2102a021-ed98-42e6-920e-41a2522bfe3f)
<!--![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/63896700-373c-46b1-9988-9b9c2bb5c747)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/9ef2de13-563a-4913-a153-63b0cf8d14c7)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/17c07153-c1a1-49b8-80e3-6d8203097c39)-->
**Credit to https://www.thingiverse.com/thing:3778338**

__Camera Mount__
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/8272498f-241f-45ee-80ac-fcd6efa82b10)
**Credit to Matstic at https://www.thingiverse.com/thing:5336496**
<!--![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/5c09cbd4-6bd3-4c49-9e59-29f13cd2f1fd)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/dfee34c8-abdf-4bed-a303-4a3d9dcf1e5c)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/979798ce-7823-4885-bd02-62e6fe5d953e)
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/fd08ccc0-b16f-4b23-b3a5-b96e650e1717)-->

__Circuit Diagram__
<img width="689" alt="image" src="https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/114700732/3ea9eb5f-e87d-42db-82ef-71abc0a1a276">

## Gantt Chart
![image](https://github.com/UCSD-ECEMAE-148/spring-2023-final-project-team-5/assets/110933991/ac5c8ead-56f3-41d3-8ed3-f4c0331c360c)

## Course Deliverables
Here are our autonomous laps as part of our class deliverables and preparation for the final project:

* Lane detection using OpenCV + ROS2: https://youtu.be/ensYDWS0fc4
* Inner lane: https://drive.google.com/file/d/1SGNKMuTuL6o_IKrJB7Sfbhk6654geIMa/view?usp=drive_link
* Outer lane: https://drive.google.com/file/d/1BtGmwQEgpboFYSuyDKBHTyqufXaYvAOl/view?usp=drive_link
* GPS: https://youtu.be/IlAwR1aKfdU

Here are our presentation slides for the weekly project updates and final presentation:
[Team 5 Presentation](https://docs.google.com/presentation/d/1sPPWAnGMisoc15QvhqSP7sxDoCSJ5Mn_mOzy9liQnEs/edit?usp=sharing)

## Acknowledgements
Special thanks to Professor Jack Silberman and TAs (Kishore Nukala & Moises Lopez) and  for all of your support!

**Credited Programs Referenced:**
* [Spring 2022 Team 1](https://guitar.ucsd.edu/maeece148/index.php/2022SpringTeam1)
* [DepthAI](https://github.com/luxonis/depthai-python)
* [UCSD Robocar Framework](https://gitlab.com/ucsd_robocar2)

## Contacts

* Shasta Subramanian - shasta.subramanian@gmail.com | s1subram@ucsd.edu
* Armond Greenberg -
* Zixu Hao - 
* Jacob Chandler - 


