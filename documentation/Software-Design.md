# Software Design

This section is aimed at providing a template of what the software on the system will be. Since the main board for this work will be an Nvidia Jetson nano, any diagrams will likely aim to show the many microservices running on this small board.

## Design

### Overview

The overview of design can be seen in the following diagram.

<p align="center">
  <picture>
    <img src="https://github.com/gotbordom/rc-racer/blob/main/documentation/diagrams/RC%20Car%20Design%20Diagram.png?raw=true" alt="Overview" style="width:100%; max-width:600px; height:auto;">
  </picture>
</p>

Here it canbe seen that a Red node means that this section is not intended to be implemented for this RC car. However I am aiming to write this code to be able to handle new sensors / hardware as needed with as few changes to the code as possible. So I am noting that they could exist but don't today.

Many of these nodes may already exist in the ROS2 ecosystem today. As they get implemented the diagrams may be updated to handle these adjustments.

The Green sections really are just indicating that my hdarware is already done and the sensors are installed.

### State Estimation

Similar to previously mentioned in the overview, I am aiming to make this as modular as I can. We will see how well that goes. This includes not only changes to which sensors are added to the system, but also what frame is used. Knowing the frame used is important, especially for state estiamtion (and later localization) because a two wheel differential robot will drive differently than an ackerman steering system.

The below diagram is aimed at using a simple Bridge Adapter pattern to more easily handle these varying robot frames.

<p align="center">
  <picture>
    <img src="https://github.com/gotbordom/rc-racer/blob/main/documentation/diagrams/State%20Estimation%20Node.png?raw=true" alt="Overview" style="width:100%; max-width:600px; height:auto;">
  </picture>
</p>

Here I am extending the rcl::Node class with another simple interface to then add the RobotFrame interface. This creates the bridge adapter and allows each node I create for this robot to have a robot frame associated with it, all shareing the same methods for obtaining a new state estimate.

## Data requirements

Since this is a real time system that will be merging many sensors with the aim of doing localization and obstacle tracking/avoidance, there will be a lot of data being produced and consumed. This section aims to mention all forms of that will be expected in order to create the best solution for handling this data in real time.

### Data produced

- Camera
  - Image streams
  - Disparity maps
  - Position estimates
  - Detected static and/or dynamic obstacles
  - Decteded mjor / minor objectives
- GPS
  - Position estimates
- Accelerometer
  - Position Estimates
  - Velocity Estimates
  - Acceleration Estimates
- Wheel encoders ( longer term goal )
  - Position Estimates
    - NOTE: Wheel encoders are one of the few ways to better track slipage.
- System Statistics
  - Battery lifetime estimates
  - Tempurature measurements
- General logs

### Data use cases

- Position Estimates
  - Aggregated together to get a more accurate position of the car (localization)
  - Choosing correct steering angle and speed to continue along the current path
  - Visualizations for UI and debugging, etc.
- Detected obstacles and/or objectives
  - Update the konwn map
  - Adjust current path as needed
  - Visualizations for UI and debugging, etc.
- System Stats
  - Mainly as a way to track health of the system

### Backend Software Requirements

This secion aims to paint a better picture of what we will likely need from software to manage the data.

This is a real time solution, with lots of data. I envision that in order to do all the tasks that an autonomous RC car will need to do, we will need a time effiecient way to:

- Maintain / Update our current state
- Access any previous states

## Third Party Utilities

In this section we will talk about utilities that have been found to be very helpful when working whith this hardware.

- Jtop monitor: https://jetsonhacks.com/2023/02/07/jtop-the-ultimate-tool-for-monitoring-nvidia-jetson-devices/
- PID Fan Controller for Jetson nano: https://github.com/Pyrestone/jetson-fan-ctl
- Husarnet: https://husarnet.com/

## Important references

- A good source for a ubuntu 20.04 image for this specific board: https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image
- A good source for a ubuntu 20.04 ROS2 image for this specific board: https://github.com/CollaborativeRoboticsLab/JetsonNano-ROS2/blob/main/docs/u20-humble-Docker-Desktop.md
