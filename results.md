---
title: Results
permalink: /results/
---

## Demos

This section documents all implemented system demos. Each demo includes a short description of the behavior being demonstrated and a placeholder link where the corresponding demo video can be added.

---

### Arm Demo 1: Manual Command Angles

**Description:**  
This demo showcases manual control of the 4-DOF arm by sending joint angle commands in a fixed chronological sequence. The demo was filmed prior to improving the gripper design.

**Sequence:**
- Startup
- Prepare for grab
- Grab
- Lift and hold
- Sweep and return
- Drop
- Startup

**Notes:**
- Filmed before improving grip

**Demo Video:**  
[Link to Arm Demo 1 – Manual Command Angles](https://drive.google.com/drive/u/1/folders/1_Ye309f8WYFzGMTFFgnmvAcjIRsoTZuX)

---

### Arm Demo 2: Pick and Place

**Description:**  
This demo demonstrates an automated pick-and-place routine triggered by a single ROS message. The arm executes the full sequence without manual intervention.

**Sequence:**
- Startup
- Prepare for grab
- Grab
- Lift and hold
- Return to grab
- Release
- Startup

**Notes:**
- Fully automated arm sequence

**Demo Video:**  
[Link to Arm Demo 2 – Pick and Place](https://drive.google.com/drive/u/1/folders/1_Ye309f8WYFzGMTFFgnmvAcjIRsoTZuX)

---

### Arm Demo 3: TurtleBot Grab and Drop

**Description:**  
This demo integrates the arm with the TurtleBot base. The system performs an automated grab and deposits the object into the onboard cup.

**Sequence:**
- Startup
- Prepare for grab
- Grab
- Lift and hold
- Rotate to cup
- Drop
- Startup

**Notes:**
- Automated flow from a single published message

**Demo Video:**  
[Link to Arm Demo 3 – TurtleBot Grab and Drop](https://drive.google.com/drive/u/1/folders/1_Ye309f8WYFzGMTFFgnmvAcjIRsoTZuX)

---

### Project Demo 1: Search and Retrieve

**Description:**  
This demo highlights the full mobile manipulation pipeline, where the TurtleBot detects an ArUco tag, navigates to the cube, and retrieves it.

**Automated Flow:**
- Detect AR Tag
- Compute transform
- Navigate to large AR tag
- Trigger arm start (manual)
- Collect block
- Rotate and drop block into onboard cup
- Return to start location

**Notes:**
- Partial automation (manual arm trigger)

**Demo Video:**  
[Link to Project Demo 1 – Search and Retrieve](https://drive.google.com/file/d/1aWHdKR1awoMA9FBK_THKndS1D5RHgUGQ/view?usp=drive_link)

---

### Project Demo 2: Search & Retrieve, Place & Store

**Description:**  
This demo extends the search-and-retrieve task by handling multiple ArUco tags and sequential navigation goals.

**Automated Flow:**
- Detect all AR tags
- Compute distance to Tag 1
- Navigate to Tag 1 at stop distance
- Trigger arm
- Collect block
- Compute distance to Tag 2
- Navigate to Tag 2 at stop distance
- Trigger arm
- Place block
- Return block to TurtleBot holder

**Notes:**
- Multi-goal navigation
- Video recorded at 2× speed

**Demo Video:**  
[Link to Project Demo 2 – Search, Retrieve, Place & Store](https://drive.google.com/drive/u/1/folders/1_Ye309f8WYFzGMTFFgnmvAcjIRsoTZuX)

---

### Project Demo 3: Courier Task

**Description:**  
This demo demonstrates a courier-style task in which the robot transports objects between multiple locations defined by ArUco tags.

**Automated Flow:**
- Detect all AR tags
- Trigger arm
- Collect block
- Compute distance to Tag 1
- Navigate to Tag 1 at stop distance
- Place block
- Collect block
- Compute distance to Tag 2
- Navigate to Tag 2 at stop distance
- Trigger arm
- Place block
- Return block to TurtleBot holder

**Notes:**
- Fully sequential courier behavior

**Demo Video:**  
[Link to Project Demo 3 – Courier](https://drive.google.com/drive/u/1/folders/1_Ye309f8WYFzGMTFFgnmvAcjIRsoTZuX)

---

### Live Demo

**Description:**  
Our live, in-person demonstration of the full robotic system presented during Fall 2025 EECS 106A Final Showcase.

**Demo Video:**  
[Link to Live Demo](https://www.youtube.com/watch?v=JPJU4CurZMg&list=PLnocShPlK-FtQelPANvCp9eF9PmZfOWTA&index=5&t=4539s)
