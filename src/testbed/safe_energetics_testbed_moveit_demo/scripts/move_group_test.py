#!/usr/bin/env python3

# Edited file for Kinova robot from MoveIt tutorials, Move Group Python Interface (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html )

##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy

import rospy
import tf2_ros

import moveit_commander

import moveit_msgs.msg
from shape_msgs.msg import SolidPrimitive
from moveit_commander.conversions import pose_to_list

import geometry_msgs.msg
from std_msgs.msg import String

from math import pi
import numpy as np


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    self.group_name = "arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.move_group.get_planning_frame()
    print ("============ Planning frame: %s" % self.planning_frame)

    # We can also print the name of the end-effector link for this group:
    self.eef_link = self.move_group.get_end_effector_link()
    print ("============ End effector link: %s" % self.eef_link)

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    print ("============ Available Planning Groups:", self.robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (self.robot.get_current_state())
    print ("")

    # Misc variables
    self.box_name = "box"
    self.cylinder_name = "cylinder"
    self.pouring_box_tip_name = "pouring_box_tipp"
    self.co = None

    # TF2 listener
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    self.tf_pouringpoint_frame_name = "pouring_point"
    self.T_world2pouringpoint = None


  def go_to_joint_state(self):
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## the first thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    initial_angles = np.array([249.81617737, 175.453125, 59.50367737, 238.97727966, 84.47727203, 94.90909576]) # deg
    initial_angles = np.deg2rad(initial_angles) # rad

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = initial_angles[0]
    joint_goal[1] = initial_angles[1]
    joint_goal[2] = initial_angles[2]
    joint_goal[3] = initial_angles[3]
    joint_goal[4] = initial_angles[4]
    joint_goal[5] = initial_angles[5]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # print ("============ Printing robot state AGAIN")
    # print (self.move_group.get_current_state())
    print ("============ Printing robot link names")
    print (self.robot.get_link_names())

    # Let's set the end effector frame different than the default one
    # self.move_group.set_end_effector_link(self.eef_link)
    self.move_group.set_end_effector_link(self.cylinder_name + "/" + self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.eef_link + "/" + self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.eef_link + "/" + self.cylinder_name)
    # self.move_group.set_end_effector_link(self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.cylinder_name)


    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = self.T_world2pouringpoint.transform.rotation.w
    pose_goal.orientation.x = self.T_world2pouringpoint.transform.rotation.x
    pose_goal.orientation.y = self.T_world2pouringpoint.transform.rotation.y
    pose_goal.orientation.z = self.T_world2pouringpoint.transform.rotation.z
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z

    self.move_group.set_pose_target(pose_goal)
    # self.move_group.set_pose_target(pose_goal, end_effector_link=self.cylinder_name + "/" + self.pouring_box_tip_name)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction




  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);




  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        for key, value in attached_objects.items():
          print(key, ' : ', value)
          print(type(value))
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_box(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the end effector:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.eef_link
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.01 # slightly above the end effector
    self.scene.add_box(self.box_name, box_pose, size=(0.1, 0.1, 0.1))

    return self.wait_for_state_update(self.box_name, box_is_known=True, timeout=timeout)

  def add_cylinder(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a cylinder in the planning scene at the location of the end effector:
    height = 0.129
    radius = 0.039 

    cylinder_pose = geometry_msgs.msg.PoseStamped()
    cylinder_pose.header.frame_id = self.eef_link
    cylinder_pose.pose.orientation.x = 0.5
    cylinder_pose.pose.orientation.y = 0.5
    cylinder_pose.pose.orientation.z = 0.5
    cylinder_pose.pose.orientation.w = -0.5

    cylinder_pose.pose.position.x = 0.0 
    cylinder_pose.pose.position.y = height/2.0 # slightly below the end effector
    cylinder_pose.pose.position.z = -0.006 # slightly below the end effector
    
    
    self.scene.add_cylinder(self.cylinder_name, cylinder_pose, height, radius)

    return self.wait_for_state_update(self.cylinder_name, box_is_known=True, timeout=timeout)

  def add_cylinder_with_subframes(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a cylinder in the planning scene at the location of the end effector:
    height = 0.129
    radius = 0.039 

    co = moveit_msgs.msg.CollisionObject()
    co.operation = moveit_msgs.msg.CollisionObject().ADD # co.ADD
    co.id = self.cylinder_name
    co.header.frame_id = self.eef_link

    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = [height, radius]
    co.primitives = [cylinder]

    cylinder_pose = geometry_msgs.msg.PoseStamped()
    cylinder_pose.pose.orientation.x = 0.5
    cylinder_pose.pose.orientation.y = 0.5
    cylinder_pose.pose.orientation.z = 0.5
    cylinder_pose.pose.orientation.w = -0.5

    cylinder_pose.pose.position.x = 0.0 
    cylinder_pose.pose.position.y = height/2.0 # slightly below the end effector
    cylinder_pose.pose.position.z = -0.006 # slightly below the end effector
    co.primitive_poses = [cylinder_pose.pose]

    co.subframe_names = [self.pouring_box_tip_name]

    subframe_pose = geometry_msgs.msg.PoseStamped()
    subframe_pose.pose.orientation.x = 0.5
    subframe_pose.pose.orientation.y = 0.5
    subframe_pose.pose.orientation.z = 0.5
    subframe_pose.pose.orientation.w = -0.5

    subframe_pose.pose.position.x = 0.055
    subframe_pose.pose.position.y = height
    subframe_pose.pose.position.z = -0.006 # slightly below the end effector
    co.subframe_poses = [subframe_pose.pose]

    self.co = co

    self.scene.add_object(co)

    return self.wait_for_state_update(self.cylinder_name, box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Kinova end effector. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Kinova
    ## robot, we set ``grasping_group = 'gripper'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    # grasping_group = 'gripper'
    # touch_links = self.robot.get_link_names(group=grasping_group)
    touch_links = self.robot.get_link_names()
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.box_name, box_is_attached=True, box_is_known=False, timeout=timeout)

  def attach_cylinder(self, timeout=4):
    ## Attaching CylinderObjects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the cylinder box to the Kinova end effector. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    # grasping_group = 'gripper'
    # touch_links = self.robot.get_link_names(group=grasping_group)
    touch_links = self.robot.get_link_names()
    
    # self.move_group.attach_object(self.cylinder_name,self.eef_link,touch_links=touch_links)
    # self.scene.attach_box(self.eef_link, self.cylinder_name, touch_links=touch_links)

    # create attached collision object
    aco = moveit_msgs.msg.AttachedCollisionObject()
    # aco.object.id = self.cylinder_name
    # aco.object.operation = moveit_msgs.msg.AttachedCollisionObject().object.ADD
    aco.object = self.co
    aco.link_name = self.eef_link
    aco.touch_links = touch_links
    self.scene.attach_object(aco)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.cylinder_name, box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.box_name, box_is_known=True, box_is_attached=False, timeout=timeout)

  def detach_cylinder(self, timeout=4):
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    self.scene.remove_attached_object(self.eef_link, name=self.cylinder_name)
    # self.move_group.detach_object(name=self.cylinder_name)

    # Do not forget to reset your end_effector_link to a robot link when you detach your object
    self.move_group.set_end_effector_link(self.eef_link)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.cylinder_name, box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    self.scene.remove_world_object(self.box_name)
    ## **Note:** The object must be detached before we can remove it from the world

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.box_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_cylinder(self, timeout=4):
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the cylinder from the world.
    self.scene.remove_world_object(self.cylinder_name)
    ## **Note:** The object must be detached before we can remove it from the world

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.cylinder_name, box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print ("")
    print ("----------------------------------------------------------")
    print ("Welcome to the MoveIt MoveGroup Python Interface Test for Kinova")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    input()
    result = tutorial.go_to_joint_state()
    print("Is go_to_joint_state result sufficiently close to goal: " + str(result))

    # print ("============ Adding a Cylinder to the planning scene ...")
    input("============ Press `Enter` to add a cylinder/box to the planning scene ...")
    # tutorial.add_cylinder()
    result =tutorial.add_cylinder_with_subframes()
    # tutorial.add_box()
    print("Is add a cylinder/box to the planning scene successful: " + str(result))

    # print ("============ Attaching a Cylinder to the Kinova robot ...")
    input("============ Press `Enter` to attach a Cylinder/box to the Kinova robot ...")
    result = tutorial.attach_cylinder()
    # tutorial.attach_box()
    print("Is attach a Cylinder/box to the Kinova robot successful: " + str(result))

    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    result = tutorial.go_to_pose_goal()
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    # print ("============ Press `Enter` to plan and display a Cartesian path ...")
    # input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print ("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    # input()
    # tutorial.display_trajectory(cartesian_plan)

    # print ("============ Press `Enter` to execute a saved path ...")
    # input()
    # tutorial.execute_plan(cartesian_plan)

    # print ("============ Press `Enter` to add a box to the planning scene ...")
    # input()
    # tutorial.add_box()

  

    # print ("============ Press `Enter` to plan and execute a path with an attached collision object ...")
    # input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

    
    print ("============ Press `Enter` to detach the cylinder/box from the Kinova robot ...")
    input()
    # tutorial.detach_box()
    tutorial.detach_cylinder()

    print ("============ Press `Enter` to remove the cylinder/box from the planning scene ...")
    input()
    # tutorial.remove_box()
    tutorial.remove_cylinder()


    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/