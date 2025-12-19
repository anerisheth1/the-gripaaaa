---
layout: page
title: Conclusion
permalink: /conclusion/
nav_order: 4
---

## Results

The final implementation of The GripAAAA successfully demonstrated the autonomous retrieval and transport of wooden cubes. The system met several key design criteria:
* **Autonomous Retrieval**: The robot consistently completed the "Search and Retrieve" flow transitioning from AR tag detection to navigation to pick-and-place .
* **Precision Actuation**: The custom gripper arm successfully executed pick-and-place sequences, including lifting and rotating objects to the onboard storage cup .
* **Task Complexity**: We successfully demonstrated multi-tag courier missions where the robot navigated between distinct AR tag locations to collect and place items .

## Challenges

Throughout the development process, we encountered some technical hurdles:
* **Communication Bottlenecks**: Establishing a reliable command link between ROS, the Raspberry Pi, and the Arduino was a major hurdle . We faced specific motor control challenges where the desired autonomous arm commands were occasionally blocked by the communication overhead between the RPi and the Arduino .
* **Vision Constraints**: Very small ArUco markers (25 mm) proved difficult for the RealSense camera to detect reliably from a distance, leading to localization errors.
* **Custom Arm**: The arm design required balancing motor strength against material weight, manually calibrating joint angles to respect physical limits, and adding foam to the gripper to prevent wood-on-wood slippage.


## Hacks and Future Improvements
While functional, the current system includes a few "hacks" that we would address with more development time:
* **The "Large AR Tag" Hack**: To compensate for the difficulty in detecting small 25 mm markers on our wooden blocks, we utilized a larger ArUco tag for initial navigation to ensure the TurtleBot reached the target area before switching to finer localization.
* **Laptop communication and Motor Control Conflict**: Connecting the Arduino to the onboard Raspberry Pi caused motor commands to not be sent due to serial conflicts. To avoid the "blocking" issue, we manually connected the Arduino to a laptop to trigger arm sequences instead of sending "START" commands autonomously via ROS.

If given more time, we would implement the following improvements:
* **Advanced Navigation**: Incorporate SLAM nodes for dynamic obstacle avoidance and Inverse Kinematics (IK) for more less hard coded arm trajectories.
Robust Perception: Move beyond simple AR tags by using Machine Learning and Computer Vision for object recognition and position estimation.
* **Integrated Communication**: Streamline the TurtleBot-arm communication to eliminate the command blocking we experienced with the current serial bridge.
* **Scaling**: Enhance the arm's structural strength to support heavier loads for real-world applications like trash picking or industrial box sorting.