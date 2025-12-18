---
layout: page
title: Introduction
permalink: /intro/
nav_order: 2
---
# Introduction
## Project End Goal

The goal of this project is to design and implement an autonomous robotic system in which a TurtleBot detects wooden cubes in its environment, navigates to them, picks them up using a custom-built gripper, and transports them to a designated home base.

## Interesting Project
This project is interesting because it requires solving several core robotics challenges while also addressing the unique design challenge of building and integrating a custom gripper.

### Perception
Our robot must reliably detect and localize objects in real time using camera sensors and ArUco tags. This requires accurate detection and  coordinate frame transformations with  robust handling of sensor noise to ensure strong readings.

### Planning and Control
Our robot must convert perception outputs into navigation targets, compute appropriate stopping distances and execute stable motion toward objects while avoiding overshoot or oscillations. The system must also handle deciding when navigation is complete and when to trigger manipulation actions.

### Actuation
The manipulation component of our custom gripper introduces additional complexity due to the mechanical design and coordinate motion controlled via an external arduino. The gripper was specifically designed to securely grasp items while remaining lightweight enough to be moved by the TurtleBot. Additionally, reliably sending commands to the arm for discrete actions (grasp, rotate, and place) are challenges that require careful design.

Together, these components form a complete end-to-end robotic system that tightly couples perception, planning, control, and hardware actuation.

## Real world applications
* Warehouse and logistics robots that locate, pick up, and transport inventory items in structured and semi-structured environments
* Service robots that assist with household organization, delivery, or campus navigation tasks
* Manufacturing and industrial mobile manipulators used for material handling, assembly, and part transfer
* Autonomous laboratory and wet-lab robotics systems for drug discovery and chemical experimentation where robots must safely identify, grasp, and transport samples, containers, or labware
* Healthcare support robots that assist with supply transport or object handling in clinical environments
* Agricultural harvesting robots that detect, approach, and pick/grasp crops in outdoor settings
* Search-and-recovery robots that locate and retrieve objects in unknown or dynamic environments
