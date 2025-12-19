---
nav_order: 1
---

![team name](/assets/team_name.png)

# EECS 106A Final Project Report

Autonomous TurtleBot system for detecting, navigating to, and collecting wooden cubes using ArUco-based perception and a custom 4-DOF gripper.

---

## Overview
This project presents an end-to-end mobile manipulation system in which a TurtleBot autonomously detects wooden cubes in its environment, plans and executes navigation toward them, and performs pickup and drop-off using a custom-built gripper.

---

## System Pipeline
- **Perception:** RealSense RGB camera + ArUco tags for robust cube localization
- **Planning & Control:** TF-based navigation to a tuned stopping distance
- **Actuation:** Arduino-controlled 4-DOF gripper for grasping and deposit

---

## Results
- Reliable detection and pose estimation of cubes
- Approach and stop by cube detected via ArUco tags
- Successful pickup and placement into onboard basket

---

## Team
Andrea • Annabelle • Aneri • Arun

---

📄 Use the navigation bar above to explore our system design, implementation details, and conclusions.
