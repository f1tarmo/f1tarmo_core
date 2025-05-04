# f1tarmo_description
This package houses the f1tarmo's URDF description(s). There is a base f1tarmo
description with just the f1tarmo model, as well as a few additional provided
models that include sensors on sensor mounts (RealSense, Luxonis, ZED stereo
cameras).

## URDF Files

### asdf.urdf

### Notes
- Make a note about the unconventional base link frame?
- Make a note about the weird changes that had to be made to the base model to
  support onshape-to-robot (really, all the design time considerations).

## Learning
TODO: Consider moving this to the f1tarmo wiki/docs.
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

For the f1tarmo specifically: The [Tarmo 5](https://www.reddit.com/r/EngineeringNS/comments/zvellk/tarmo5/) (the fully 3D printed RC car that the
f1tarmo is derived from) was originally designed from the ground up in Onshape
(Thanks Engineering Nonsense). This means there was already a comprehensive
mechanical model of the vehicle. To make a URDF from this design by hand seemed
silly given all the work that was already put into the mechanical model.
Additionally, sensor extrinsics are always a pain. For those reasons, the
f1tarmo's URDF is primarily generated automatically from Onshape.

### How?
#### Find an conversion tool
To generate a URDF file from a CAD design, you will need some kind of tool that
can talk take the CAD model from the software you use and translate it into
URDF. There is [a whole
page](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Exporting-an-URDF-File.html)
under the official ROS URDF documentation page dedicated to some well-known
toolboxes for doing exactly this.

In the case of the f1tarmo (which is designed in Onshape), the
`onshape-to-robot` tool ([link to their
github](https://github.com/Rhoban/onshape-to-robot)) is the weapon of choice for
generating the URDF.

#### Generate f1tarmo URDF from Onshape design
To actually generate the f1tarmo's URDF from an Onshape design, you can follow
the [getting started steps](https://onshape-to-robot.readthedocs.io/en/latest/getting_started.html#getting-started) on the `onshape-to-robot` documentation pretty much verbatim. For this
project specifically:

1. Clone the f1tarmo repo containing the `f1tarmo_description` package if you
   have not already done so.
   ```
   git clone --recurse-submodules git@github.com:nlitz88/f1tarmo.git
   ```
2. Enter the `f1tarmo_description` package directory
   ```
   cd f1tarmo/f1tarmo_description
   ```
3. From here, the steps install the `onshape-to-robot` Python package using pip.
   ```
   sudo pip install onshape-to-robot
   ```
   Note: You may prefer to install the `onshape-to-robot` package in a virtual
   environment, instead of installing it on your host as directed above.

4. Create a `.env` file with an Onshape access key.
   ```
   touch .env
   ```
   Open this file with your preferred editor and paste in the boilerplate from
   the `onshape-to-robot` docs. There should be a field for the onshape api url,
   an access key, and a secret key.
5. Log into Onshape and head to the [Onshape developer
   portal](https://dev-portal.onshape.com/keys). Here, you can create an API key
   that will allow your computer to access documents on Onshape via their API.

   Once logged in, click on the `API Keys` tab, and then click on the `Create new API key`
   button at the top right. Check the box that gives the key permission to read
   your documents.

   Click "Create API key." A popup will appear with your access key and
   corresponding secret key. Copy these keys into the .env file you created
   earlier into their respective fields.

6. At this point, you should be able to use the `onshape-to-robot` tool to
   generate the f1tarmo URDF from the design. The `f1tarmo_description`
   package's `model` directory already contains a `config.json` file with the
   options needed for the base f1tarmo design, but you may still want to read
   the `onshape-to-robot` docs to familiarize yourself with those options.

   Before regenerating the design, delete the `assets` and `robot.urdf` files
   from the package, as these will be recreated when you regenerate the URDF.

   Run the following command to generate the f1tarmo URDF:
   ```
   onshape-to-robot model/
   ```

The tool should run and create an `assets` directory containing the stl mesh
files for each part and the `robot.urdf` file.

#### Why are there other URDF/Xacro files in the f1tarmo_description package?
The onshape to robot tool creates a URDF model with all of the most important
pieces of the f1tarmo--it's steering joints, wheel links, etc. However, there
are usually a few extra links and joints that need to be added to make the model
play nice with the rest of the ROS ecosystem.

A good example of this is when a sensor's URDF needs to be included on top of a
URDF that has just been generated from Onshape that includes a mountpoint for
that sensor. The overarching xacro file will include both the `robot.urdf` file
generated by `onshape-to-robot`, as well as the xacro file for the sensor. It
will create an instance of the sensor, with a joint between the sensor and
sensor mount point.

### Do I need to generate a URDF for the f1tarmo to use it for my project?
TL;DR: This package already comes with a few complete URDF files for common
configurations of the f1tarmo, which should be enough to get you started.
However, if your project requires substantial hardware changes and additional
sensor mounts, only then will you likely need to generate the URDF from Onshape.

For example:

- You are working on a project where you need to attach both an IMU and Camera
  to the f1tarmo chassis, and want to model these sensor mounts in Onshape.
- The base design doesn't include these mounts for these custom sensors, so you
  design your own in a copy of the f1tarmo Onshape document.
- The base URDF files already in this package will not contain your custom
  sensor mounts. Therefore, you will have to generate a new URDF from your copy
  document.
- Because this hardware configuration is probably specific to your project, you
  will likely be served best by forking the f1tarmo repository containing the
  `f1tarmo_description` package and adding the URDF in your forked repository.
  If you feel it is a configuration that the community could benefit from, you
  could create a Pull Request with your URDF and linked document.

This is just one scenario to give you an idea as to how you might extend the
f1tarmo platform for your own project's needs. However, there are all kinds of
ways to do this--and there's no single correct approach! :)

## Changelog

- 2025.05.04: Updated README with tutorial on using `onshape-to-robot` to
  generate base URDF from onshape design.