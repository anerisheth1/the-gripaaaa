---
title: Design
permalink: /design/
---

# Design Criteria and Functionality

![design](/assets/design.png)

The primary objective of this project was to create an autonomous mobile manipulation system capable of localized object retrieval. The specific design criteria and desired functionalities included:
* **Autonomous Detection**: The system must use computer vision to identify and locate specific targets (wooden cubes) and the robot’s own position relative to them using ArUco markers.
* **Navigation**: The TurtleBot must navigate to a precise stop distance from the target to facilitate a successful grasp.
* **Precision Actuation**: A custom-built robotic arm must execute a multi-step pick-and-place sequence, including preparing for the grab, grasping, lifting, and depositing the object into an onboard container.
* **Loop Closure**: The robot must be able to return to its home base after successfully retrieving or placing a block

# Design

## Arm Design Physical Structure

![arm design 1](/assets/arm_design1.png)
![arm design 2](/assets/arm_design2.png)

We developed a 4-DOF (Degree of Freedom) robotic arm constructed from basswood featuring four revolute servos for positioning and one additional servo for the gripper. The arm has a vertical reach of approximately 11.8 cm and a horizontal reach of 14.5 cm to the gripper. The gripper opens at 140 degrees and closes at 180 degrees.

## Perception and Localization

We utilized an Intel RealSense camera to detect 25 mm ArUco markers pasted on the blocks. This data is processed through an Aruco Node in ROS2 to update the TF Tree allowing the TurtleBot Controller to compute precise transforms for navigation.


## Control System

The high-level planning is handled in ROS, which communicates velocity commands to the TurtleBot base. For the arm, commands are bridged from a Raspberry Pi to an Arduino via USB Serial to actuate the servos.


# Design Choices and Trade-Offs

During the development process, several design choices were made to balance performance with available resources:
* **Material Selection**: We transitioned from a cardboard and balsa wood prototype to basswood for the final design. While basswood increased the weight, it provided the structural rigidity necessary to maintain consistent joint angles during heavy lifts.
* **Actuator**: We swapped standard micro-servos for MG995 servos to increase torque.
* **Gripper Surface**: We added foam grips to the wooden gripper. This was a response to the "wood on wood" slippage encountered in early testing, sacrificing a small amount of precision for a significantly higher grasp success rate.
* **Communication Bridge**: We ideally wanted for a ROS → RPi → Arduino serial bridge. This allowed us to autonomously trigger the Arduino to start moving the gripper while maintaining high-level control in ROS though it added complexity to the software stack and wiring with the Turtlebot.


# Engineering Applications

Our design choices directly impacted the system's viability in professional engineering contexts:
* **Robustness**: The use of ArUco markers provides a reliable localization method, but the small size (25 mm) proved difficult to detect at a distance limiting the distance the camera could operate at.
* **Durability**: The move to basswood improved the arm's strength but it still lacks the durability of metal or carbon-fiber components found in industrial Trash Pickers or Industrial Sorters.
* **Efficiency**: The flow from detection to deposit demonstrates high operational efficiency . However, the current manual calibration required for joint limits and operation would require automation.
