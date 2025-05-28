
Whether you are manually driving your car around to map out a
new area or just need a quick way to take over while debugging a new
planning algorithm, you will need some way to control your f1tarmo.

You can control your f1tarmo through any device that has a ROS interface for
translating the controller's physical buttons/rockers to  
[`geometry_msgs/TWist`](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)
messages, as this is the interface the f1tarmo's low-level control stack
supports/implements.

While a keyboard can absolutely do the trick with the
`ros-teleop-twist-keybaord` package, a console-style controller will often be
more familiar and intuitive. 


What kind of controller do I need?
- Any controller that implements the [Simple DirectMedia Layer
  (SDL)](https://www.libsdl.org/release/SDL-1.2.15/docs/html/joystick.html).
  Most controllers are "SDL" controllers.

Logitech F710 Controller
- Switch the top rocker to the "X" position. This puts the controller in
  "X-Input" mode, which will essentially make it behave as if it were an xBox
  controller (which is what we want). The cross-platform SDL2 library that most
  programs use to interact with controllers supports XInput devices, so this is
  a good bet.
- If not already, make sure you have inserted charged batteries into your
  controller.
- Plug the USB receiver into one of the Jetson's USB ports.
- Run `sudo dmesg` to see if your controller was detected. You should see
  something like this:
  ```
  [344540.149568] usb 1-2.3: new full-speed USB device number 9 using tegra-xusb
  [344540.273128] hid-generic 0003:046D:C22F.0051: hidraw0: USB HID v1.11 Device [Logitech Logitech Cordless RumblePad 2] on usb-3610000.usb-2.3/input0
  [344540.277035] hid-generic 0003:046D:C22F.0052: hidraw1: USB HID v1.11 Device [Logitech Logitech Cordless RumblePad 2] on usb-3610000.usb-2.3/input1
  ```

  Additionally, if you run `lsusb`, you should find your controller somewhere in that list, like:

  ```
  Bus 002 Device 009: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
  Bus 002 Device 002: ID 0bda:0489 Realtek Semiconductor Corp. 4-Port USB 3.0 Hub
  Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
  Bus 001 Device 003: ID 13d3:3549 IMC Networks Bluetooth Radio
  Bus 001 Device 010: ID 046d:c21f Logitech, Inc. F710 Wireless Gamepad [XInput Mode]
  Bus 001 Device 004: ID 0483:5740 STMicroelectronics Virtual COM Port
  Bus 001 Device 002: ID 0bda:5489 Realtek Semiconductor Corp. 4-Port USB 2.0 Hub
  Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
  ```
- Install `jstest`. `jstest` is a convenient tool we can use to make sure our
  Jetson is actually receiving signals / control inputs from the controller we
  just connected.
  ```
  sudo apt-get install joystick
  ```
  

The linux kernel shipped with Jetpack 6 (Jetson Linux 36.3) does not have the
necessary kernel modules/drivers to support game controllers like the Logitech
F710. Thus, we need to add these modules to the kernel ourselves. There are two
easy ways to do this as of right now:

1. Install `xpad` and use dkms to insert the necessary kernel modules via the
  Dynamic Kernel Module System (dkms) (thanks to [Genozen's solution](https://github.com/jetsonhacks/logitech-f710-module/issues/7#issue-2441586565)).
2. Rebuild the linux kernel with the necessary kernel modules added with a [guide
  from
  Jetsonhacks](https://github.com/jetsonhacks/logitech-f710-module/tree/master/JetPack6).
  
While both of these approaches work, solution #1 is a bit harder to screw up, so
it is recommended that you try this approach first.

Method1: xpad and dkms

- Install the dkms command line tool:
  ```
  sudo apt update && sudo apt install -y dkms
  ```
- Remove any existing versions of the xpad driver just in case:
  ```
  sudo dkms remove xpad/0.4 --all
  ```
- Install xpad by following the instructions from their [github
  readme](https://github.com/paroj/xpad?tab=readme-ov-file#installing).
  ```
  sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4
  sudo dkms install -m xpad -v 0.4
  ```
- Now, run `sudo dmesg --follow`. Switch your Logitech controller's rocker from
  `D` to `X`, and you should see something similar to the following in your
  dmesg log:
  ```
  [346396.759371] usb 1-2.3: new full-speed USB device number 12 using tegra-xusb
  [346396.912751] xpad: module verification failed: signature and/or required key missing - tainting kernel
  [346396.913905] input: Logitech Gamepad F710 as /devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.3/1-2.3:1.0/input/input943
  [346396.914201] usbcore: registered new interface driver xpad
  ```
- Xpad should have created a new file descriptor for the controller device at
  `/dev/input/js0`. Run jstest with this to check its outputs:
  ```
  jstest /dev/input/js0
  ```
  You should see the values change for each column as you press different
  buttons.

Now that we know the Jetson can see the inputs from the controller itself, it is
time to pipe those controller inputs into our vehicle's ROS control interface.
This is typically done with a combination of two off-the-shelf nodes:
1. A node to take the raw X-Input controller commands and encode them into a
   [`sensor_msgs/Joy`
   message](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Joy.html).
   This node will likely come from one of the packages in the
   [`joystick_drivers` metapackage](http://wiki.ros.org/joystick_drivers),
   depending on what kind of controller you have:
     - For any SDL controller (most common), you'll use the `joy` node from the
       [`joy` package](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy).
     - When the `joy` package doesn't work (because you are missing some
       essential Linux driver, usually), the [`joy_linux`
       package](https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy_linux)
       has a node that will probably do the trick.
     - There are other packages like `ps3joy` and `wiimote` in the
       `joystick_drivers` package that provide support for those other types of
       controllers.
2. A node to take the `Joy` messages and translate them into [`geometry_msgs/TWist`](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/Twist.html)
messages (what the f1tarmo's control interface is expecting). This is typically
the `teleop_node` from the [`teleop_twist_joy`
package](http://wiki.ros.org/teleop_twist_joy).

**joy** package
For our Logitech F710 controller, we will first attempt to get this working with
the `joy_node` from the `joy` package listed above, as it not only supports
translating joystick commands to Joy messages, but also subscribes to a feedback
topic for setting controller LED and vibration. While these functions are not
strictly  necessary, they might be nice to have.

- Install the `joy` package:
  ```
  sudo apt update && sudo apt install -y ros-humble-joy
  ```
- Source your installation workspace:
  ```
  source /opt/ros/humble.setup.bash
  ```
- To see if the joy node will be able to talk to your controller, run the
  following utility node from the `joy` package:
  ```
  ros2 run joy joy_enumerate_devices
  ```
  You should see an output something like this:
  ```
  ID : GUID                             : GamePad : Mapped : Joystick Device Name
  -------------------------------------------------------------------------------
  0 : 030000006d0400001fc2000005030000 :    true :   true : Logitech F710 Gamepad (XInput)
  ```

- If that was successful, you should be good to run the `joy` node:
  ```
  ros2 run joy joy_node
  ```

**joy_linux** package
If the `joy_node` from the `joy` package didn't work out, there's a good chance
you'll still have hope with the `joy_linux_node` from the `joy_linux` package.

- Install the `joy_linux` package:
  ```
  sudo apt update && sudo apt install -y ros-humble-joy-linux
  ```
- Source your installation workspace:
  ```
  source /opt/ros/humble.setup.bash
  ```
- Run the `joy_linux_node`
  ```
  ros2 run joy_linux joy_linux_node
  ```

Remapping joystick buttons
- In its current state, pressing the `RB` button will cause the steering servo
  to fly over to one side and start spinning the wheels slowly. This is a good
  sign of life, but it means we need to remap the buttons of the controller so
  that the steering is controlled by one of the joysticks and throttle by one of
  the triggers. 
- This can be done with a node provided by the [`joystick_remapper`
  package](http://wiki.ros.org/joystick_remapper). Basically, this node just
  takes the `joy` message published by whichever joystick node you used above
  and changes the order of the commanded values within the message.

Remapping teleop_twist_joy node

TODO:
- Angular axis is 0 from the controller
- Axis 5 for speed.
- Enable button for button 5
- Need to set deadzone for axis 5 or something. 1.0 needs to be 0, -1 needs to
  be 100% throttle.
- Maybe make the other trigger for reverse? 
- The f1tenth stack just uses the joysticks--maybe I should just do this too.
