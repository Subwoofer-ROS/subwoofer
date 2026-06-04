# Subwoofer

Subwoofer is a robotic quadruped developed from the microspot project for ROS2 Humble.

## Startup sequence
Boot sequence:
1. Before starting any nodes, set the legs so each leg is fully bent and flush with the ground. The hips should be at 0.
2. Next, launch `behaviour.launch.py`, which will start the behaviour manager and all behaviour nodes.
3. Once the state manager reports online, run the `subwoofer` node with `simulated:=false` (likely to be merged with `behaviour.launch.py` in the future).
4. Finally, launch `remote_controller.launch.py` (currently tested on the Switch 1 joycons, may need remapping for other controllers for correct button labelling).

The robot pose can be visualised in RViz2 by launching `viewer.launch.py`.



## States
The robot currently have three states.
The behaviour for each state is listed below:

### STORAGE
The initial state which the robot expects on boot.
Each leg is tucked under itself, causing the main body to be lifted above the ground.

### STANCE
The robot will have some bend to each leg, but will stand stably.
Using the controller joysticks, the height (left y), pitch (right y) and roll (right x) of the robot can be adjusted.
This stance is used as a "hub state", acting as an in-between state for other states (currently storage and trot).
While in any other state, they will only be able to go to this state. 

### TROT
The robot will trot forwards at a fixed pace.
No controls other than mode switching is available, though several parameters can be tuned through the cli.



## Controls
The controls available depends on what state the robot is in.
The state switcher is however always available:

| Button | Requested state |
|:------:|:---------------:|
| A      | Stance          |
| B      | Trot            |
| X      | Storage         |

### Controls in *STANCE* state
When in the *STANCE* state, the joysticks enable to contorl height, pitch and roll.

| Joystick | Axis | Parameter | Direction                       |
|:--------:|:----:|:---------:|:-------------------------------:|
| Left     | X    | Unused    |                                 |
| Left     | Y    | Height    | Up = increased height           |
| Right    | X    | Roll      | Left = lower left, higher right |
| Right    | Y    | Pitch     | Inverted controls               |
