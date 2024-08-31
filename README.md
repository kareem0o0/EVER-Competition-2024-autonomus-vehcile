# EVER Autonomous Vehicle Competition

## 🏆 Second Place Overall!

Welcome to our repository documenting our journey through the EVER (Electric Vehicle Rally) Autonomous Vehicle Competition. Over 6 months, our team worked tirelessly on various milestones, ultimately securing second place overall! This repository serves as a record of our work, challenges, and achievements throughout the competition.

## 🚗 Competition Overview

The EVER competition is designed to challenge teams in developing cutting-edge autonomous driving capabilities. It simulates real-world scenarios and pushes participants to integrate various aspects of autonomous vehicle technology. The competition was structured into three main milestones, with a special award for top-performing teams:

1. Open Loop Controller
2. Environmental Perception
3. System Integration
4. Special Award: Vehicle Real-time Demo (Top 5 Teams)

Each milestone is built upon the previous one, gradually increasing in complexity and requiring a more sophisticated integration of autonomous driving technologies.

## 📅 Milestones

### Milestone 1: Open Loop Controller

- **Objective**: Develop and test an open loop control system for a vehicle in a simulation environment.
- **Key Concepts**: Trajectory planning, vehicle kinematics, open loop control
- **Tasks**:
  1. Move in a straight line (75m): Test basic forward movement control
  2. Draw a circle (6m radius): Implement constant curvature motion
  3. Lane change maneuver: Combine straight line motion with lateral movement
  4. Draw an infinity shape: Master complex curved trajectories
- **Tools**: 
  - CoppeliaSim V4.5.1 rev4 simulator
  - ROS1 for communication between our control algorithms and the simulator
- **Challenges**:
  - Achieving accurate path following without sensor feedback
  - Compensating for vehicle dynamics in an open loop system

### Milestone 2: Environmental Perception

This milestone was divided into two crucial parts:

#### Part 1: Sensor Data Processing
- **Objective**: Improve the reliability of sensor data and implement closed-loop control
- **Tasks**:
  1. Add Gaussian noise to Odometry readings: Simulate real-world sensor imperfections
  2. Apply filtering methods: Implement Kalman Filter or Extended Kalman Filter
  3. Develop a closed-loop controller: Use filtered data for more accurate navigation
- **Key Concepts**: Sensor fusion, state estimation, noise filtering

#### Part 2: Object Detection
- **Objective**: Develop a computer vision system for identifying objects in the vehicle's environment
- **Tasks**:
  1. Identify objects (human, car, cone) using the car's camera
  2. Implement bounding box drawing with labels and accuracy scores
- **Technologies Used**: 
  - YOLO (You Only Look Once) for real-time object detection
  - OpenCV for image processing
- **Challenges Overcome**:
  - Real-time processing of high-resolution (960x480) camera feed
  - Achieving high accuracy in varied lighting conditions

### Milestone 3: System Integration

- **Objective**: Create a comprehensive autonomous driving system by integrating perception, control, and sensor filtering
- **Key Features Implemented**:
  1. Obstacle detection and classification: Identify and categorize objects in real-time
  2. Adaptive Cruise Control: Maintain safe distances from vehicles ahead
  3. Lane Change Maneuvers: Safely navigate around slower vehicles or obstacles
  4. Emergency Stops: Rapidly respond to sudden obstacles or hazards
- **Tracks Developed**:
  1. Straight line with pedestrian crossing: Test basic obstacle avoidance
  2. Lane change with obstacles: Evaluate complex decision-making
  3. Circular track with mixed obstacles: Challenge the system with continuous curvature and varied obstacles
  4. Custom track design: (Describe your unique track and its specific challenges)
- **Integration Challenges**:
  - Ensuring real-time performance with multiple subsystems running concurrently
  - Balancing safety, efficiency, and comfort in the control algorithms
  - Handling edge cases and unexpected scenarios

### Special Award: Vehicle Real-time Demo

- **Achievement**: Qualified as one of the top 5 teams!
- **Challenge**: Control a real-life car to move in an infinity shape for three laps
- **Track Details**:
  - Bounded by traffic cones (yellow on right, blue on left)
  - Infinity shape requiring precise steering and speed control
- **Scoring Criteria**:
  1. Lap time: Emphasis on completing laps quickly
  2. Lateral oscillation: Smooth trajectory following
  3. Avoiding cone collisions: Precision in navigation

## 🛠 Technologies Used

Our solution integrated a wide range of technologies and tools:

- **ROS1 (Robot Operating System)**: Used for inter-process communication and as the backbone of our software architecture
- **Python**: Primary programming language for implementing algorithms and data processing
- **C++**: Used for performance-critical components
- **MATLAB**: 
  - Data visualization and analysis
  - Controller tuning and simulation
  - prototyping of algorithms
- **Computer Vision Libraries**:
  - YOLO (You Only Look Once) for real-time object detection
  - OpenCV for image processing and feature detection
- **Sensor Fusion Algorithms**:
  - Extended Kalman Filter for state estimation
- **Control Theory**:
  - PID controllers for low-level control
  - Pure Pursuit for trajectory following
- **Simulation Environments**:
  - CoppeliaSim for initial testing and development


## 📁 Repository Structure

Our repository is organized to reflect the progression through the competition milestones:

```
/
├── Milestone1/
│   ├── code/
│   ├── Videos/
│   ├── CSVs/
│   └── report.pdf
│   └── Rules.pdf

├── Milestone2/
│   ├── Vision/
│   │   ├── Models/
│   │   ├── Videos/
│   ├── Noise and control/
│   │   ├── Matlab Code/
│   │   ├── ROS Code/
│   └── Report.pdf
│   └── Rules.pdf
├── Milestone3/
│   ├── CSVs/
│   ├── Code/
│   ├── videos/
│   ├── Vision Models/
│   ├── Plots/
│   ├── Objects/
│   └── Report.pdf
│   └── Rules.pdf
├── RealTimeDemo/
│   ├── Code/
│   ├── CSVs/
│   └── Scene_Real_time.ttt
├── Additionals/
│   ├── Archives/
│   ├── Business and Cost/
├── EVER V_Autonomous Track Rules.pdf
└── license.md
└── README.md
```


## 🏅 Achievements

Our team's hard work and innovation led to significant achievements in the EVER Autonomous Vehicle Competition:

- **Second Place Overall**: A testament to our consistent performance across all milestones
- **Honors**: Received recognition and encouragement from mentors and judges as one of the best teams they have worked with.
- **Top 5 Team Qualification**: Selected for the Vehicle Real-time Demo, showcasing the real-world applicability of our solutions

These achievements reflect our team's dedication, technical skills, and ability to integrate complex systems effectively.

## 👥 Team Members

Our success was made possible by the diverse skills and tireless efforts of our team members:

- [Omar Emad]: Noise-Filtration -  Computer-Vision model design - 
- [Kareem Salah]: computer vision & LIDAR fusion - PID control & tuning
- [Islam Wael]: Noise-Filtration - PID control & tuning - Depth Camera 
- [Mohamed Emad]: Pure pursuit control - Debugging   
- [Omar Abbas]: Pure-Pursuit control and tuning - Computer-Vision model design - Real-time milestone control 

Each team member brought unique expertise and perspective, contributing to our well-rounded and effective approach to autonomous vehicle development.

## 🙏 Acknowledgements

We extend our heartfelt gratitude to:

- The organizers of the EVER competition for creating this challenging and educational experience
- Our mentors, who provided invaluable guidance and support throughout the competition
- Eng, Ibrahim Abdelghfar (Mechatronics Engineer)
- Eng, Ziad Osama (Robotics Software Engineer)
- Open-source communities behind ROS, OpenCV, and other tools that were crucial to our project


## 📚 Further Reading

For those interested in diving deeper into our work:

- Detailed reports for each milestone can be found in their respective directories
---

We hope this repository serves as both a record of our achievement and a resource for future teams tackling similar challenges in autonomous vehicle development. For any questions or collaborations, please open an issue or contact us directly.

Thank you for your interest in our EVER Autonomous Vehicle Competition journey!
