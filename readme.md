# Subwoofer

Subwoofer is a quadrupedal robot modelled after robots like Boston Dynamics' Spot and the Unitree go.
The version made has been given the name `spotmicro`, and the original designs can be found [here](https://gitlab.com/public-open-source/spotmicroai).


## Usage
The Aber Robotics Club's subwoofer is currently ran on a raspberry pi running ROS2 Humble.
Login for this pi is (ip: `192.168.4.1`, username: `ubuntu`, password: `rootroot`).
The ros2 workspace is located in the ubuntu home directory.


## Setup
### Installation
1. Create a ROS2 Humble workspace.
2. Clone this git repository into the `src` directory.
3. Build and source the `subwoofer_interfaces` package.
4. Build the remaining packages.

### Face detection setup
In `face_detection_launch.xml`, make sure the cascade file is properly linked for face detection.


## Servos

| Leg         | Joint | ID | Standing |
| ----------- | ----- | -- | :------: |
| Front Left  | Hip   | 05 | 140      |
| Front Left  | Upper | 02 | 130      |
| Front Left  | Lower | 00 | 070      |
| Front Right | Hip   | 04 | 105      |
| Front Right | Upper | 03 | 078      |
| Front Right | Lower | 01 | 132      |
| Back Left   | Hip   | 07 | 090      |
| Back Left   | Upper | 13 | 100      |
| Back Left   | Lower | 15 | 048      |
| Back Right  | Hip   | 06 | 110      |
| Back Right  | Upper | 12 | 179      |
| Back Right  | Lower | 14 | 110      |




## Robot model
The robot model meshes are borrowed from the microspot project, and recoloured to match the team's printed version.
A license for the original files can be found in `subwoofer_model/meshes`.

