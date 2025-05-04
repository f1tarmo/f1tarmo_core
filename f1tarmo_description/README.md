# f1tarmo_description
This package houses the f1tarmo's URDF description(s). There is a base f1tarmo
description with just the f1tarmo model, as well as a few additional provided
models that include sensors on sensor mounts (RealSense, Luxonis, ZED stereo
cameras).

## URDF Files


## Learning
### What is a URDF file?
Put simply, a URDF file is a physical descrpition of a robot. I.e., how wide it
is, how tall it is, how far apart the wheels are from each other, where the
camera is with respect to the wheels, etc.

URDF, or Unified Robot Description Format, is a standard format used to define
those kinds of physical attributes about a robot. You can find more great
information and examples for URDF from the [ROS documentation](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html).

### Why do we need a physical description of my robot?
To motivate why you might need a physical description of your robot, it helps to
consider a few different scenarios:

- Your robot's path planner is trying to figure out if it can fit through a
  doorway. How can it figure this out without knowing how wide your robot is? It
  needs some kind of physical measurements of your robot.
- You have a GPS antenna mounted at the front of your robot. The GPS
  measurements will tell you where the GPS antenna is, but not necessarily where
  the robot is. How do you know where the GPS antenna is with respect to the
  wheels of your robot, or with respect to the footprint of your robot?
- You want to simulate your robot in a virtual environment for offline testing.
  However, how can a simulator (like Gazebo) figure out how your robot will move
  if it doesn't know how big your robot's wheels are?

These are just toy examples, but they help you to begin thinking about why this
physical/mechanical information is so vital.

### How do I make a URDF file for my robot?
To make a physical description of your robot, you have a few options these days:

1. Create a URDF file by hand
  - This is probably the most common method for creating a basic physical robot
    description for getting started.
  - Tutorial from the official ROS documentation
    [here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html).
  - Tutorial (and a great explanation!) from the ROS Nav2 documentation
    [here](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html).
  - Another fantastic tutorial and explanation from Articulated Robotics
    [here](https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf/).
2. Create a URDF file visually
  - These kinds of tools have just popped up recently, and allow you to
    model your robot using a CAD-like interface.
  - [RoboEverything](https://lever-robotics.github.io/URDF_creator/) is a recent
    example of one of these visual tools.
3. Automatically generate URDF from a CAD design of your robot
  - This is by and large the most *scalable* way to generate a URDF for your
    robot, but it requires that you already have a mechanical model for your
    robot in some CAD software.
  - The official ROS docs have a page dedicated to the tools available to go
    from a CAD model of your robot to a URDF file [here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Exporting-an-URDF-File.html).


## Generating URDF from Onshape
### Why?
Why generate your robot's URDF from a CAD model instead of making it by hand?
For almost any project, there are a few key factors that come to mind:
1. **Allows for quick iterations and changes to the mechanical design**: Often
   times, robots will be designed in CAD, but then some poor soul will have to
   go and craft a URDF from the mechanical drawings given to them. That takes a
   lot of time, especially if the robot's design is physically changing a lot.
2. **Accurate sensor extrinsics**: So often, URDF tutorials will talk about
   including sensors to your robot's design, but don't talk about _where those
   sensors are actually mounted on the robot_. Unless you are just duct-taping
   your LiDAR or camera to your robot, there's a good chance you have designed
   or purchased some kind of mounting bracket to mount the sensor on your robot.
   This mounting bracket can be included in your robot's CAD drawings, and a
   link can be greated for the exact point the sensor is mounted to. _Yes,
   sensor extrinsics are usually computed anyway, but not all systems have this
   capability, and many of them that do still benefit from a good initial
   guess._
3. **Accurate dynamics and inertia**: If you have a detailed CAD model of your
   robot (if you are Unitree or Bostom Dynamics, for example), then that model
   probably estimates the mass of each component pretty accurately. If you want
   to do any dynamic simulation with your robot, the dynamics properties of each
   piece of your robot (mass and rotation inertial) need to be present. While
   you can fill in these properties by hand, many of the toolboxes can extract
   these properties and translate them directly into your robot's URDF.

For the f1tarmo specifically: The Tarmo 5 (the fully 3D printed RC car that the
f1tarmo is derived from) was originally designed from the ground up in Onshape
(Thanks Engineering Nonsense). This means there was already a comprehensive
mechanical model of the vehicle. To make a URDF from this design by hand seemed
silly given all the work that was already put into the mechanical model.
Additionally, sensor extrinsics are always a pain. For those reasons, the
f1tarmo's URDF is primarily generated automatically from Onshape.

### How?


## TODO
The README for this package should detail:
- The purpose and origin of each URDF/xacro file it contains, a small
  explanation for each.
- A brief explanation of the rationale behind the model and resulting tf tree,
  woes with onshape-to-robot, etc. Basically, explain to them why the base link
  isn't the root (if I end up doing that). This would also be a good place to
  describe REP 105...
- Additionally, there should be a description for EACH URDF / model file in
  here. For now, there is just the model with a realsense and a standalone model
  with no sensor.

For the URDF generated from CAD via onshape-to-robot, I'm thinking that I should
just drop a link in there to the document in the doc folder that describes the
workflow for generating this from the onshape design. Either that, or that
document lives in here. Not sure which makes more sense.

If you're just building a "standard" f1tarmo, you don't care, and don't need
to carry out those steps. For that reason, I think it might make the most sense
to just document that process here, as that is really specific to generating the
contents of this package, or modifying/extending it if you are making a
different version of the f1tarmo / extending it. Then, in the main docs, I can
just link to this document.

## Changelog

- 2025.05.04: Updated README with tutorial on using `onshape-to-robot` to
  generate base URDF from onshape design.