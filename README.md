# sdl-application

Repository for files and applications of the sdl mobile robotic scientist at Argonne National Laboratory and University of Chicago. Should be used with the [sdl-packages](https://github.com/dsquez/sdl-packages.git) repository.

## Install the AR library:

`sudo apt-get install ros-melodic-ar-track-alvar`

Move Gazebo models:
```
cp sdl-packages/sdl_gazebo/models/ar_box ~/.gazebo/models -r
```

## Running

### Launching the Robot
Run the launch file:

`roslaunch sdl_application single_robot.launch`

The gazebo window is launched in a paused state to give the controllers enough time to initialize, when you see the message:

`[ WARN] [1627157134.932762011]: service '/get_planning_scene' not advertised yet. Continue waiting...
[ INFO] [1627157134.933636210]: waitForService: Service [/get_planning_scene] has not been advertised, waiting... `

**Press the play button in Gazebo.** You will see the arm oscillate slightly. This can be mitigated in the future with controller gain tuning.

When you see the message:

`You can start planning now!`

`[ INFO] [1627157159.619626329, 18.202000000]: Ready to take commands for planning group ur_arm.`

You are ready to proceed. You can run the following python script or publish to the `/ur_arm/moveit/goal_pose` topic

### Demo Picking Up Can
`python sdl_application/src/can_demo.py`

This Python file moves the robot to the location of the AR Tag cube and finds the exact location of the cube using information from the AR Tag topic. It then grabs the cube with the arm and moves it to another table.

### Limb Test
As of 6 August 2021, the `set_joints_position()` function works to publish joint commands to gazebo. This is demonstrated by the following launch command:

`roslaunch sdl_application test_limb_class.launch`

You will get errors that the velocity is not specified. This does not seem to effect functionality.

## Details
`single_robot.launch` starts a robot by running the necessary ROS nodes provided by the repository `sdl-packages`. It loads a URDF of the robot, starts the controllers, etc.

In `src/can_demo.py`, a [plugin](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins) is used to attach the gripper to the cube. To enable the plugin, this line must be in the world file: `<plugin name='ros_link_attacher_plugin' filename='libgazebo_ros_link_attacher.so'/>`.

## Issues
Robot teleports (likely due to the plugin that attaches the gripper to the cube)
MoveIt fails because the arm position is not in the expected spot
The tables are empty beneath, which prevents the MIR from being able to detect them