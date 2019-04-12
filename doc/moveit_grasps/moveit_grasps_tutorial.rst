MoveIt Grasps
=======================

MoveIt Grasps is a grasp generator for objects such as blocks or cylinders and can be used as a replacement for the MoveIt pick and place pipeline. MoveIt Grasps provides functionality for filtering grasps based on reachability and Cartesian planning of approach, lift and retreat motions. The grasp generation algorithm is based on simple cuboid shapes and does not consider friction cones or other grasp dynamics.

MoveIt Grasps can be used with both parallel finger grippers and suction grippers.
Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

.. image:: https://raw.githubusercontent.com/ros-planning/moveit_grasps/kinetic-devel/resources/demo.png
   :width: 500pt

Installing MoveIt Grasps
^^^^^^^^^^^^^^^^^^^^^^^^^

Install From Source
--------------------

Clone the `moveit_grasps <https://github.com/ros-planning/moveit_grasps>`_ repository into a `catkin workspace <https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace>`_. For this tutorial we use Franka Emika's Panda robot setup from `panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_::

    cd ~/ws_moveit/src
    git clone https://github.com/ros-planning/moveit_grasps.git

Use the rosdep tool to automatically install its dependencies::

    rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO

Build the workspace::

    catkin build

Install From Debian
--------------------

**Note:** this package has not been released as of 4/11/19::

    sudo apt-get install ros-$ROS_DISTRO-moveit-grasps

Setup
^^^^^^^^^^^^^^^^

Robot-Agnostic Configuration
----------------------------

Before you can use MoveIt Grasps with your robot and end effector, you will need a configuration file that described your robot's end effector geometry.

An example can be seen in the MoveIt Grasps repository at `config_robot/panda_grasp_data.yaml <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/config_robot/panda_grasp_data.yaml>`_.

See the comments within that file for explanations.

To load that file at launch, you copy the example in the file `launch/load_panda.launch <https://github.com/ros-planning/moveit_grasps/blob/kinetic-devel/launch/load_panda.launch>`_ where you should see the line::

    <rosparam command="load" file="$(find moveit_grasps)/config_robot/panda_grasp_data.yaml"/>

Within that file you will find all of the gripper specific parameters necessary for customizing MoveIt Grasps with any suction or parallel finger gripper.

Notes on Some Important Parameters:
-------------------------------------

**grasp_pose_to_eef_transform**

The ``grasp_pose_to_eef_transform`` represents the transform from the wrist to the end-effector.
This parameter is provided to allow different URDF end effectors to all work together without recompiling code.

In MoveIt the EE always has a parent link, typically the wrist link or palm link.
That parent link should have its Z-axis pointing towards the object you want to grasp i.e. where your pointer finger is pointing.

This is the convention laid out in "Robotics" by John Craig in 1955.
However, a lot of URDFs do not follow this convention, so this transform allows you to fix it.

Additionally, the x-axis should be pointing up along the grasped object, i.e. the circular axis of a (beer) bottle if you were holding it.

The y-axis should be point towards one of the fingers.

**Switch from Bin to Shelf Picking**

The ``setIdealGraspPoseRPY()`` and ``setIdealGraspPose()`` methods in GraspGenerator can be used to select an ideal grasp orientation for picking.

These methods is used to score grasp candidates favoring grasps that are closer to the desired orientation.

This is useful in applications such as bin and shelf picking where you would want to pick the objects from a bin with a grasp that is vertically alligned and you would want to pick obejects from a shelf with a grasp that is horozontally alligned.

Demo Scripts
^^^^^^^^^^^^

We have provided demo scripts showcasing MoveIt Grasps, and for visualizing MoveIt Grasps configuration parameters.

.. image:: https://raw.githubusercontent.com/ros-planning/moveit_grasps/kinetic-devel/resources/moveit_grasps_poses.jpeg
   :width: 500pt


There are four demo scripts in this package. To view the tests, first start Rviz with::

    roslaunch moveit_grasps rviz.launch

To see the entire MoveIt Grasps pipeline in action::

    roslaunch moveit_grasps grasp_pipeline_demo.launch

To visualize gripper specific parameters::

    roslaunch moveit_grasps grasp_poses_visualizer_demo.launch

To test just grasp generation for randomly placed blocks::

    roslaunch moveit_grasps demo_grasp_generator.launch

To test the grasp filtering::

    roslaunch moveit_grasps demo_filter.launch

Grasp Filter
------------

When filtered, the colors represent the following:

* RED - grasp filtered by ik
* PINK - grasp filtered by collision
* MAGENTA - grasp filtered by cutting plane
* YELLOW - grasp filtered by orientation
* BLUE - pregrasp filtered by ik
* CYAN - pregrasp filtered by collision
* GREEN - valid

Tested Robots
-------------

* UR5
* Jaco2
* Baxter
* `REEM <http://wiki.ros.org/Robots/REEM>`_
* Panda
