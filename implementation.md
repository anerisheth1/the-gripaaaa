![arm protoype to final](/assets/arm_prototype_to_final.png)

### Components Used

- **Mobile Base:** TurtleBot platform equipped with a Raspberry Pi.
- **Perception:** RealSense Camera for depth and ArUco marker detection.
- **The Arm:** A custom 4-DOF robotic arm.
- **Servos:** Four MG995 high-torque revolute servos for the base, shoulder, and elbow joints, plus one micro-servo for the gripper.
- **Materials:** Basswood links for arm structure and foam padding on the gripper to prevent slippage.
- **Electronics:**  
  - **Arduino:** Used to control the five servos.
- **Battery Pack:** Dedicated power supply for the high-torque servos.
- **Containers:** An onboard basket (paper cup) for storing retrieved cubes.

### What parts did you use to build your solution?

- **Prototype:** Cardboard, Basswood  
- **Gripper design modified from:**  
  https://www.instructables.com/  
  Robotic-Gripper-1

### Software Implementation
![ros1](/assets/ros1.png)
![ros2](/assets/ros2.png)

Our software stack is built on ROS2 coordinating perception, navigation, and control.

#### 1. ROS Architecture

High-level control is managed by a central TurtleBot Controller node that interfaces with several sub-systems:

- **Vision Node:** Processes `image_raw` and `camera_info` from the RealSense camera using the Aruco Node to output a `PoseArray` and `ArucoMarkers`.
- **TF Tree:** Dynamically calculates the transform from the `camera_frame` to the `ar_marker_`.
- **Serial Bridge:** A dedicated `arduino_serial_bridge` running on the Raspberry Pi sends movement commands via USB Serial to the Arduino.

#### 2. Motion Planning & Control

- **Navigation:** The controller uses TF Lookups to compute the distance to the target ArUco tag. It publishes to the `/cmd_vel` topic to drive the TurtleBot until the "stop distance" is reached.
- **Arm Sequences:** The arm operates via pre-defined sequences triggered manually by wired connection from laptop to arduino.

### Complete System

The system follows a sequential state machine to complete a **Search and Retrieve** mission:

1. **Target Detection:** The robot identifies an ArUco Tag place next to a cube.
2. **Approach:** The controller computes the transform to the tag and navigates the TurtleBot to the precise stop distance.
3. **Grasping Sequence:**  
   - The arm moves to a start pose.  
   - The gripper opens to 140° and then closes to 180° to secure the block.  
   - The arm lifts the block
4. **Deposit:** The base or arm rotates to align with the onboard cup and the gripper releases the block.
5. **Return/Next Task:** The robot either returns to the start location or computes the distance to a second tag for a follow-up courier task.
