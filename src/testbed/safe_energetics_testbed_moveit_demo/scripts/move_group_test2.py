#!/usr/bin/env python3

# Edited file for Kinova robot from MoveIt tutorials, Move Group Python Interface (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html )

##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

from ossaudiodev import control_labels
import sys
import copy

import rospy
import tf2_ros
import tf.transformations
import tf_conversions

import moveit_commander

import moveit_msgs.msg
from shape_msgs.msg import SolidPrimitive
from moveit_commander.conversions import pose_to_list

import geometry_msgs.msg
from std_msgs.msg import String

from math import pi, cos, sin, tan, fabs
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

    # self.move_group.set_planner_id("SBLkConfigDefault")
    # self.move_group.set_planner_id("ESTkConfigDefault")
    # self.move_group.set_planner_id("LBKPIECEkConfigDefault")
    # self.move_group.set_planner_id("BKPIECEkConfigDefault")
    # self.move_group.set_planner_id("KPIECEkConfigDefault")
    # self.move_group.set_planner_id("RRTkConfigDefault")
    # self.move_group.set_planner_id("RRTConnectkConfigDefault") # default
    # self.move_group.set_planner_id("RRTstarkConfigDefault") # maybe
    # self.move_group.set_planner_id("TRRTkConfigDefault") # maybe 2
    # self.move_group.set_planner_id("PRMkConfigDefault") # maybe 1 fails on revers
    # self.move_group.set_planner_id("PRMstarkConfigDefault") # yes fails on reverse

    self.group_name_gripper = "gripper"
    self.move_group_gripper = moveit_commander.MoveGroupCommander(self.group_name_gripper)

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
    # print ("============ Printing robot state")
    # print (self.robot.get_current_state())

    # print ("============ Printing Goal Tolerances")
    # self.print_tolerances()
    print ("")

    

    # Misc variables
    self.ground_name = "ground_plane"
    self.block1_name = "block1"
    self.block2_name = "block2"
    self.ramp_name = "ramp"
    self.desired_pouring_height =  0.15
    self.pouring_offset_y = 0.05

    self.cylinder_name = "cylinder"
    self.pouring_box_tip_name = "pouring_box_tipp"
    self.keep_orient = True
    self.keep_position = True

    self.plan = None # main plan from home to pouring point
    self.plan2 = None # plan for rotation of the pouring box
    self.plan3 = None # plan for rotation of the pouring box (reverse)
    self.rot_angle = 90.0 # deg , angle for amount of robot motion during pouring
    
    self.ramp_angle = 75.0 # deg # choose one: 15. 30. 45. 60. 75. 


    # TF2 listener
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    self.tf_pouringpoint_frame_name = "pouring_point"
    self.tf_pouringboxtip_frame_name = "pouring_box_tip"
    self.tf_testbedorigin_frame_name = "testbed_origin"


    self.T_world2pouringpoint = None # fixed
    self.T_world2pouringboxtip = None # not fixed
    self.T_pouringboxtip2eef = None # fixed
    # self.T_pouringboxtip2pouringpoint = None

  def close_gripper(self):
    target = "HalfClose"
    gripper_goal = self.move_group_gripper.get_named_target_values(target)

    self.move_group_gripper.set_named_target(target)
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group_gripper.go(wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group_gripper.stop()

    # For testing:
    current_joints = self.move_group_gripper.get_current_joint_values()
    return all_close(gripper_goal, current_joints, 0.01)

  def open_gripper(self):
    target = "Open"
    gripper_goal = self.move_group_gripper.get_named_target_values(target)

    self.move_group_gripper.set_named_target(target)
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group_gripper.go(wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group_gripper.stop()

    # For testing:
    current_joints = self.move_group_gripper.get_current_joint_values()
    return all_close(gripper_goal, current_joints, 0.01)

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
    self.keep_orient = True
    self.keep_position = True
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # Let's set the end effector frame different than the default one
    self.move_group.set_end_effector_link(self.eef_link)
    # self.move_group.set_end_effector_link(self.cylinder_name + "/" + self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.eef_link + "/" + self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.eef_link + "/" + self.cylinder_name)
    # self.move_group.set_end_effector_link(self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.cylinder_name)


    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    self.T_world2pouringboxtip = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringboxtip_frame_name,  rospy.Time())
    self.T_pouringboxtip2eef = self.tfBuffer.lookup_transform(self.tf_pouringboxtip_frame_name, self.eef_link, rospy.Time())

    current_position = self.move_group.get_current_pose().pose.position

    R0 = self.tf2R(self.T_world2pouringpoint)
    R1 = self.tf2R(self.T_pouringboxtip2eef)
    Rgoal = np.dot(R0,R1)
    qgoal = tf_conversions.transformations.quaternion_from_matrix(Rgoal) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = qgoal[0]
    pose_goal.orientation.y = qgoal[1]
    pose_goal.orientation.z = qgoal[2]
    pose_goal.orientation.w = qgoal[3]
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x + (current_position.x - self.T_world2pouringboxtip.transform.translation.x)
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y + (current_position.y - self.T_world2pouringboxtip.transform.translation.y)
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z + (current_position.z - self.T_world2pouringboxtip.transform.translation.z)

    self.move_group.set_pose_target(pose_goal)
    # self.move_group.set_pose_target(pose_goal, end_effector_link=self.cylinder_name + "/" + self.pouring_box_tip_name)

    #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      orien_const.link_name = self.eef_link
      orien_const.header.frame_id = self.planning_frame

      #constrain it to be the same as my goal state.  Seems reasonable.
      orien_const.orientation.x = pose_goal.orientation.x
      orien_const.orientation.y = pose_goal.orientation.y
      orien_const.orientation.z = pose_goal.orientation.z
      orien_const.orientation.w = pose_goal.orientation.w
      orien_const.absolute_x_axis_tolerance = 0.1 # 0.1
      orien_const.absolute_y_axis_tolerance = 2.0*pi # 0.1
      orien_const.absolute_z_axis_tolerance = 0.1 # 0.1
      orien_const.weight = 0.5

      # orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().XYZ_EULER_ANGLES # Default
      orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().ROTATION_VECTOR

      consts.orientation_constraints.append(orien_const)
      
    if self.keep_position:
      # Define position constrain box based on current pose and the target pose
      primitive = SolidPrimitive()
      primitive.type = primitive.BOX

      constrain_box_scale = 2.0
      x_dim = constrain_box_scale * fabs(pose_goal.position.x - current_position.x)
      y_dim = constrain_box_scale * fabs(pose_goal.position.y - current_position.y)
      z_dim = constrain_box_scale * fabs(pose_goal.position.z - current_position.z)
      primitive.dimensions = [x_dim,y_dim,z_dim]

      box_pose = geometry_msgs.msg.Pose()
      box_pose.orientation.w = 1.0
      # place btw start point and goal point
      box_pose.position.x = (pose_goal.position.x + current_position.x)/2.0
      box_pose.position.y = (pose_goal.position.y + current_position.y)/2.0
      box_pose.position.z = (pose_goal.position.z + current_position.z)/2.0

      pos_const = moveit_msgs.msg.PositionConstraint()
      pos_const.link_name = self.eef_link
      pos_const.header.frame_id = self.planning_frame
      pos_const.constraint_region.primitives.append(primitive)
      pos_const.constraint_region.primitive_poses.append(box_pose)
      pos_const.weight = 0.5

      consts.position_constraints.append(pos_const)

    if self.keep_orient or self.keep_position:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END

    ## Now, we call the planner to compute the plan and execute it.
    # self.move_group.go(wait=True)
    user_confirmed = False
    while not user_confirmed:
      (is_planned, plan, plan_time, err_code) = self.move_group.plan()

      if (is_planned == False):
        print("PLANNING FAILED")
        wanna_plan_again = self.single_yes_or_no_question("Would you like to try planning again?", default_no=False)
        if wanna_plan_again:
          continue
        else:
          print("planning failed, continuing without execution")
          break
      else:
        self.display_trajectory(plan) 
        user_confirmed = self.single_yes_or_no_question("Do you accept the suggested plan?")
    
    if is_planned:
      # save the plan for re use later
      self.plan = plan
      # Execute the plan
      self.move_group.execute(self.plan, wait=True)
      

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # Also clear path constraints
    if self.keep_orient or self.keep_position:
      self.move_group.clear_path_constraints()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def go_to_pose_goal2(self):
    self.keep_orient = True
    self.keep_position = True
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # Let's set the end effector frame different than the default one
    self.move_group.set_end_effector_link(self.eef_link)

    theta = np.deg2rad(self.rot_angle)

    Rnew = np.eye(4)
    Rnew[1,1] = cos(theta)
    Rnew[1,2] = sin(theta)
    Rnew[2,1] = -sin(theta)
    Rnew[2,2] = cos(theta)

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    self.T_world2pouringboxtip = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringboxtip_frame_name,  rospy.Time())
    self.T_pouringboxtip2eef = self.tfBuffer.lookup_transform(self.tf_pouringboxtip_frame_name, self.eef_link, rospy.Time())

    current_position = self.move_group.get_current_pose().pose.position

    R0 = self.tf2R(self.T_world2pouringpoint)
    R1 = self.tf2R(self.T_pouringboxtip2eef)
    R0new = np.dot(R0,Rnew)
    Rgoal = np.dot(R0new,R1)
    qgoal = tf_conversions.transformations.quaternion_from_matrix(Rgoal) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = qgoal[0]
    pose_goal.orientation.y = qgoal[1]
    pose_goal.orientation.z = qgoal[2]
    pose_goal.orientation.w = qgoal[3]
    
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x + (self.T_pouringboxtip2eef.transform.translation.x)
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + sin(theta)*(self.T_pouringboxtip2eef.transform.translation.z)
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z - sin(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.z)

    self.move_group.set_pose_target(pose_goal)
    # self.move_group.set_pose_target(pose_goal, end_effector_link=self.cylinder_name + "/" + self.pouring_box_tip_name)

    # #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      orien_const.link_name = self.eef_link
      orien_const.header.frame_id = self.planning_frame

      #constrain it to be the same as my goal state.  Seems reasonable.
      orien_const.orientation.x = pose_goal.orientation.x
      orien_const.orientation.y = pose_goal.orientation.y
      orien_const.orientation.z = pose_goal.orientation.z
      orien_const.orientation.w = pose_goal.orientation.w
      orien_const.absolute_x_axis_tolerance = 0.1 # 0.1
      orien_const.absolute_y_axis_tolerance = 0.1 # pi # 0.1
      orien_const.absolute_z_axis_tolerance = 2*pi # 0.1
      orien_const.weight = 0.5

      # orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().XYZ_EULER_ANGLES # Default
      orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().ROTATION_VECTOR

      consts.orientation_constraints.append(orien_const)
      
    if self.keep_position:
      # Define position constrain box based on current pose and the target pose
      primitive = SolidPrimitive()
      primitive.type = primitive.BOX

      constrain_box_scale = 2.0
      x_dim = constrain_box_scale * fabs(pose_goal.position.x - current_position.x)
      y_dim = constrain_box_scale * fabs(pose_goal.position.y - current_position.y)
      z_dim = constrain_box_scale * fabs(pose_goal.position.z - current_position.z)
      primitive.dimensions = [x_dim,y_dim,z_dim]

      box_pose = geometry_msgs.msg.Pose()
      box_pose.orientation.w = 1.0
      # place btw start point and goal point
      box_pose.position.x = (pose_goal.position.x + current_position.x)/2.0
      box_pose.position.y = (pose_goal.position.y + current_position.y)/2.0
      box_pose.position.z = (pose_goal.position.z + current_position.z)/2.0

      pos_const = moveit_msgs.msg.PositionConstraint()
      pos_const.link_name = self.eef_link
      pos_const.header.frame_id = self.planning_frame
      pos_const.constraint_region.primitives.append(primitive)
      pos_const.constraint_region.primitive_poses.append(box_pose)
      pos_const.weight = 0.5

      consts.position_constraints.append(pos_const)

    if self.keep_orient or self.keep_position:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END


    ## Now, we call the planner to compute the plan and execute it.
    # self.move_group.go(wait=True)
    user_confirmed = False
    while not user_confirmed:
      (is_planned, plan, plan_time, err_code) = self.move_group.plan()

      if (is_planned == False):
        print("PLANNING FAILED")
        wanna_plan_again = self.single_yes_or_no_question("Would you like to try planning again?", default_no=False)
        if wanna_plan_again:
          continue
        else:
          print("planning failed, continuing without execution")
          break
      else:
        self.display_trajectory(plan)
        user_confirmed = self.single_yes_or_no_question("Do you accept the suggested plan?")
    
    if is_planned:
      # save the plan for re use later
      self.plan2 = plan
      # Execute the plan
      self.move_group.execute(self.plan2, wait=True)
      

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # # Also clear path constraints
    if self.keep_orient or self.keep_position:
      self.move_group.clear_path_constraints()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal3(self):
    self.keep_orient = True
    self.keep_position = True
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # Let's set the end effector frame different than the default one
    self.move_group.set_end_effector_link(self.eef_link)

    theta = np.deg2rad(0.0)

    Rnew = np.eye(4)
    Rnew[1,1] = cos(theta)
    Rnew[1,2] = sin(theta)
    Rnew[2,1] = -sin(theta)
    Rnew[2,2] = cos(theta)

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    self.T_world2pouringboxtip = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringboxtip_frame_name,  rospy.Time())
    self.T_pouringboxtip2eef = self.tfBuffer.lookup_transform(self.tf_pouringboxtip_frame_name, self.eef_link, rospy.Time())

    current_position = self.move_group.get_current_pose().pose.position

    R0 = self.tf2R(self.T_world2pouringpoint)
    R1 = self.tf2R(self.T_pouringboxtip2eef)
    R0new = np.dot(R0,Rnew)
    Rgoal = np.dot(R0new,R1)
    qgoal = tf_conversions.transformations.quaternion_from_matrix(Rgoal) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = qgoal[0]
    pose_goal.orientation.y = qgoal[1]
    pose_goal.orientation.z = qgoal[2]
    pose_goal.orientation.w = qgoal[3]
    
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x + (self.T_pouringboxtip2eef.transform.translation.x)
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + sin(theta)*(self.T_pouringboxtip2eef.transform.translation.z)
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z - sin(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.z)

    self.move_group.set_pose_target(pose_goal)
    # self.move_group.set_pose_target(pose_goal, end_effector_link=self.cylinder_name + "/" + self.pouring_box_tip_name)

    #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      orien_const.link_name = self.eef_link
      orien_const.header.frame_id = self.planning_frame

      #constrain it to be the same as my goal state.  Seems reasonable.
      orien_const.orientation.x = pose_goal.orientation.x
      orien_const.orientation.y = pose_goal.orientation.y
      orien_const.orientation.z = pose_goal.orientation.z
      orien_const.orientation.w = pose_goal.orientation.w
      orien_const.absolute_x_axis_tolerance = 0.1 # 0.1
      orien_const.absolute_y_axis_tolerance = 0.1 # pi # 0.1
      orien_const.absolute_z_axis_tolerance = 2*pi # 0.1
      orien_const.weight = 0.5

      # orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().XYZ_EULER_ANGLES # Default
      orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().ROTATION_VECTOR

      consts.orientation_constraints.append(orien_const)
      
    if self.keep_position:
      # Define position constrain box based on current pose and the target pose
      primitive = SolidPrimitive()
      primitive.type = primitive.BOX

      constrain_box_scale = 2.0
      x_dim = constrain_box_scale * fabs(pose_goal.position.x - current_position.x)
      y_dim = constrain_box_scale * fabs(pose_goal.position.y - current_position.y)
      z_dim = constrain_box_scale * fabs(pose_goal.position.z - current_position.z)
      primitive.dimensions = [x_dim,y_dim,z_dim]

      box_pose = geometry_msgs.msg.Pose()
      box_pose.orientation.w = 1.0
      # place btw start point and goal point
      box_pose.position.x = (pose_goal.position.x + current_position.x)/2.0
      box_pose.position.y = (pose_goal.position.y + current_position.y)/2.0
      box_pose.position.z = (pose_goal.position.z + current_position.z)/2.0

      pos_const = moveit_msgs.msg.PositionConstraint()
      pos_const.link_name = self.eef_link
      pos_const.header.frame_id = self.planning_frame
      pos_const.constraint_region.primitives.append(primitive)
      pos_const.constraint_region.primitive_poses.append(box_pose)
      pos_const.weight = 0.5

      consts.position_constraints.append(pos_const)

    if self.keep_orient or self.keep_position:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END

    ## Now, we call the planner to compute the plan and execute it.
    # self.move_group.go(wait=True)
    user_confirmed = False
    while not user_confirmed:
      (is_planned, plan, plan_time, err_code) = self.move_group.plan()

      if (is_planned == False):
        print("PLANNING FAILED")
        wanna_plan_again = self.single_yes_or_no_question("Would you like to try planning again?", default_no=False)
        if wanna_plan_again:
          continue
        else:
          print("planning failed, continuing without execution")
          break
      else:
        self.display_trajectory(plan)
        user_confirmed = self.single_yes_or_no_question("Do you accept the suggested plan?")
    
    if is_planned:
      # save the plan for re use later
      self.plan3 = plan
      # Execute the plan
      self.move_group.execute(self.plan3, wait=True)
      

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # # Also clear path constraints
    if self.keep_orient or self.keep_position:
      self.move_group.clear_path_constraints()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, scale=1):
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = self.move_group.get_current_pose().pose
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
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def go_to_pose_goal_cartesian(self):
    ## Planning to a Pose Goal Cartesian
    waypoints = []
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # Let's set the end effector frame different than the default one
    self.move_group.set_end_effector_link(self.eef_link)
    # self.move_group.set_end_effector_link(self.cylinder_name + "/" + self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.eef_link + "/" + self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.eef_link + "/" + self.cylinder_name)
    # self.move_group.set_end_effector_link(self.pouring_box_tip_name)
    # self.move_group.set_end_effector_link(self.cylinder_name)


    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    self.T_world2pouringboxtip = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringboxtip_frame_name,  rospy.Time())
    self.T_pouringboxtip2eef = self.tfBuffer.lookup_transform(self.tf_pouringboxtip_frame_name, self.eef_link, rospy.Time())

    current_position = self.move_group.get_current_pose().pose.position

    R0 = self.tf2R(self.T_world2pouringpoint)
    R1 = self.tf2R(self.T_pouringboxtip2eef)
    Rgoal = np.dot(R0,R1)
    qgoal = tf_conversions.transformations.quaternion_from_matrix(Rgoal) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = qgoal[0]
    pose_goal.orientation.y = qgoal[1]
    pose_goal.orientation.z = qgoal[2]
    pose_goal.orientation.w = qgoal[3]
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x + (current_position.x - self.T_world2pouringboxtip.transform.translation.x)
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y + (current_position.y - self.T_world2pouringboxtip.transform.translation.y)
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z + (current_position.z - self.T_world2pouringboxtip.transform.translation.z)

    # self.move_group.set_pose_target(pose_goal)

    #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      orien_const.link_name = self.eef_link
      orien_const.header.frame_id = self.planning_frame

      #constrain it to be the same as my goal state.  Seems reasonable.
      orien_const.orientation.x = pose_goal.orientation.x
      orien_const.orientation.y = pose_goal.orientation.y
      orien_const.orientation.z = pose_goal.orientation.z
      orien_const.orientation.w = pose_goal.orientation.w
      orien_const.absolute_x_axis_tolerance = 0.1 # 0.1
      orien_const.absolute_y_axis_tolerance = 2.0*pi # 0.1
      orien_const.absolute_z_axis_tolerance = 0.1 # 0.1
      orien_const.weight = 0.5

      # orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().XYZ_EULER_ANGLES # Default
      orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().ROTATION_VECTOR

      consts.orientation_constraints.append(orien_const)
      
    if self.keep_position:
      # Define position constrain box based on current pose and the target pose
      primitive = SolidPrimitive()
      primitive.type = primitive.BOX

      constrain_box_scale = 2.0
      x_dim = constrain_box_scale * fabs(pose_goal.position.x - current_position.x)
      y_dim = constrain_box_scale * fabs(pose_goal.position.y - current_position.y)
      z_dim = constrain_box_scale * fabs(pose_goal.position.z - current_position.z)
      primitive.dimensions = [x_dim,y_dim,z_dim]

      box_pose = geometry_msgs.msg.Pose()
      box_pose.orientation.w = 1.0
      # place btw start point and goal point
      box_pose.position.x = (pose_goal.position.x + current_position.x)/2.0
      box_pose.position.y = (pose_goal.position.y + current_position.y)/2.0
      box_pose.position.z = (pose_goal.position.z + current_position.z)/2.0

      pos_const = moveit_msgs.msg.PositionConstraint()
      pos_const.link_name = self.eef_link
      pos_const.header.frame_id = self.planning_frame
      pos_const.constraint_region.primitives.append(primitive)
      pos_const.constraint_region.primitive_poses.append(box_pose)
      pos_const.weight = 0.5

      consts.position_constraints.append(pos_const)

    if self.keep_orient or self.keep_position:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END

    waypoints.append(pose_goal)
    jump_threshold=0.0
    eef_step = 0.01


    ## Now, we call the planner to compute the plan and execute it.
    # self.move_group.go(wait=True)
    user_confirmed = False
    while not user_confirmed:
      # (is_planned, plan, plan_time, err_code) = self.move_group.plan()
      is_planned = True
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold) 

      if (is_planned == False):
        print("PLANNING FAILED")
        wanna_plan_again = self.single_yes_or_no_question("Would you like to try planning again?", default_no=False)
        if wanna_plan_again:
          continue
        else:
          print("planning failed, continuing without execution")
          break
      else:
        self.display_trajectory(plan) 
        user_confirmed = self.single_yes_or_no_question("Do you accept the suggested plan?")
    
    if is_planned:
      # save the plan for re use later
      self.plan = plan
      # Execute the plan
      self.move_group.execute(self.plan, wait=True)
      

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # Also clear path constraints
    if self.keep_orient or self.keep_position:
      self.move_group.clear_path_constraints()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal2_cartesian(self):
    ## Planning to a Pose Goal with cartesian planning
    waypoints = []
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # Let's set the end effector frame different than the default one
    self.move_group.set_end_effector_link(self.eef_link)

    theta = np.deg2rad(self.rot_angle)

    Rnew = np.eye(4)
    Rnew[1,1] = cos(theta)
    Rnew[1,2] = sin(theta)
    Rnew[2,1] = -sin(theta)
    Rnew[2,2] = cos(theta)

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    self.T_world2pouringboxtip = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringboxtip_frame_name,  rospy.Time())
    self.T_pouringboxtip2eef = self.tfBuffer.lookup_transform(self.tf_pouringboxtip_frame_name, self.eef_link, rospy.Time())

    current_position = self.move_group.get_current_pose().pose.position

    R0 = self.tf2R(self.T_world2pouringpoint)
    R1 = self.tf2R(self.T_pouringboxtip2eef)
    R0new = np.dot(R0,Rnew)
    Rgoal = np.dot(R0new,R1)
    qgoal = tf_conversions.transformations.quaternion_from_matrix(Rgoal) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = qgoal[0]
    pose_goal.orientation.y = qgoal[1]
    pose_goal.orientation.z = qgoal[2]
    pose_goal.orientation.w = qgoal[3]
    
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x + (self.T_pouringboxtip2eef.transform.translation.x)
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + sin(theta)*(self.T_pouringboxtip2eef.transform.translation.z)
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z - sin(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.z)

    # self.move_group.set_pose_target(pose_goal)

    # #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      orien_const.link_name = self.eef_link
      orien_const.header.frame_id = self.planning_frame

      #constrain it to be the same as my goal state.  Seems reasonable.
      orien_const.orientation.x = pose_goal.orientation.x
      orien_const.orientation.y = pose_goal.orientation.y
      orien_const.orientation.z = pose_goal.orientation.z
      orien_const.orientation.w = pose_goal.orientation.w
      orien_const.absolute_x_axis_tolerance = 0.1 # 0.1
      orien_const.absolute_y_axis_tolerance = 0.1 # pi # 0.1
      orien_const.absolute_z_axis_tolerance = 2*pi # 0.1
      orien_const.weight = 0.5

      # orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().XYZ_EULER_ANGLES # Default
      orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().ROTATION_VECTOR

      consts.orientation_constraints.append(orien_const)
      
    if self.keep_position:
      # Define position constrain box based on current pose and the target pose
      primitive = SolidPrimitive()
      primitive.type = primitive.BOX

      constrain_box_scale = 2.0
      x_dim = constrain_box_scale * fabs(pose_goal.position.x - current_position.x)
      y_dim = constrain_box_scale * fabs(pose_goal.position.y - current_position.y)
      z_dim = constrain_box_scale * fabs(pose_goal.position.z - current_position.z)
      primitive.dimensions = [x_dim,y_dim,z_dim]

      box_pose = geometry_msgs.msg.Pose()
      box_pose.orientation.w = 1.0
      # place btw start point and goal point
      box_pose.position.x = (pose_goal.position.x + current_position.x)/2.0
      box_pose.position.y = (pose_goal.position.y + current_position.y)/2.0
      box_pose.position.z = (pose_goal.position.z + current_position.z)/2.0

      pos_const = moveit_msgs.msg.PositionConstraint()
      pos_const.link_name = self.eef_link
      pos_const.header.frame_id = self.planning_frame
      pos_const.constraint_region.primitives.append(primitive)
      pos_const.constraint_region.primitive_poses.append(box_pose)
      pos_const.weight = 0.5

      consts.position_constraints.append(pos_const)

    if self.keep_orient or self.keep_position:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END

    waypoints.append(pose_goal)
    jump_threshold=0.0
    eef_step = 0.02


    ## Now, we call the planner to compute the plan and execute it.
    # self.move_group.go(wait=True)
    user_confirmed = False
    while not user_confirmed:
      # (is_planned, plan, plan_time, err_code) = self.move_group.plan()
      is_planned = True
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold) 

      if (is_planned == False):
        print("PLANNING FAILED")
        wanna_plan_again = self.single_yes_or_no_question("Would you like to try planning again?", default_no=False)
        if wanna_plan_again:
          continue
        else:
          print("planning failed, continuing without execution")
          break
      else:
        self.display_trajectory(plan)
        user_confirmed = self.single_yes_or_no_question("Do you accept the suggested plan?")
    
    if is_planned:
      # save the plan for re use later
      self.plan2 = plan
      # Execute the plan
      self.move_group.execute(self.plan2, wait=True)
      
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # # Also clear path constraints
    if self.keep_orient or self.keep_position:
      self.move_group.clear_path_constraints()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal3_cartesian(self):
    ## Planning to a Pose Goal cartesian
    waypoints = []
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()

    self.move_group.set_start_state_to_current_state()

    # Let's set the end effector frame different than the default one
    self.move_group.set_end_effector_link(self.eef_link)

    theta = np.deg2rad(0.0)

    Rnew = np.eye(4)
    Rnew[1,1] = cos(theta)
    Rnew[1,2] = sin(theta)
    Rnew[2,1] = -sin(theta)
    Rnew[2,2] = cos(theta)

    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    self.T_world2pouringpoint = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringpoint_frame_name,  rospy.Time())
    self.T_world2pouringboxtip = self.tfBuffer.lookup_transform(self.planning_frame, self.tf_pouringboxtip_frame_name,  rospy.Time())
    self.T_pouringboxtip2eef = self.tfBuffer.lookup_transform(self.tf_pouringboxtip_frame_name, self.eef_link, rospy.Time())

    current_position = self.move_group.get_current_pose().pose.position

    R0 = self.tf2R(self.T_world2pouringpoint)
    R1 = self.tf2R(self.T_pouringboxtip2eef)
    R0new = np.dot(R0,Rnew)
    Rgoal = np.dot(R0new,R1)
    qgoal = tf_conversions.transformations.quaternion_from_matrix(Rgoal) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = qgoal[0]
    pose_goal.orientation.y = qgoal[1]
    pose_goal.orientation.z = qgoal[2]
    pose_goal.orientation.w = qgoal[3]
    
    pose_goal.position.x = self.T_world2pouringpoint.transform.translation.x + (self.T_pouringboxtip2eef.transform.translation.x)
    pose_goal.position.y = self.T_world2pouringpoint.transform.translation.y + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + sin(theta)*(self.T_pouringboxtip2eef.transform.translation.z)
    pose_goal.position.z = self.T_world2pouringpoint.transform.translation.z - sin(theta)*(self.T_pouringboxtip2eef.transform.translation.y) + cos(theta)*(self.T_pouringboxtip2eef.transform.translation.z)

    # self.move_group.set_pose_target(pose_goal)

    #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      orien_const.link_name = self.eef_link
      orien_const.header.frame_id = self.planning_frame

      #constrain it to be the same as my goal state.  Seems reasonable.
      orien_const.orientation.x = pose_goal.orientation.x
      orien_const.orientation.y = pose_goal.orientation.y
      orien_const.orientation.z = pose_goal.orientation.z
      orien_const.orientation.w = pose_goal.orientation.w
      orien_const.absolute_x_axis_tolerance = 0.1 # 0.1
      orien_const.absolute_y_axis_tolerance = 0.1 # pi # 0.1
      orien_const.absolute_z_axis_tolerance = 2*pi # 0.1
      orien_const.weight = 0.5

      # orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().XYZ_EULER_ANGLES # Default
      orien_const.parameterization = moveit_msgs.msg.OrientationConstraint().ROTATION_VECTOR

      consts.orientation_constraints.append(orien_const)
      
    if self.keep_position:
      # Define position constrain box based on current pose and the target pose
      primitive = SolidPrimitive()
      primitive.type = primitive.BOX

      constrain_box_scale = 2.0
      x_dim = constrain_box_scale * fabs(pose_goal.position.x - current_position.x)
      y_dim = constrain_box_scale * fabs(pose_goal.position.y - current_position.y)
      z_dim = constrain_box_scale * fabs(pose_goal.position.z - current_position.z)
      primitive.dimensions = [x_dim,y_dim,z_dim]

      box_pose = geometry_msgs.msg.Pose()
      box_pose.orientation.w = 1.0
      # place btw start point and goal point
      box_pose.position.x = (pose_goal.position.x + current_position.x)/2.0
      box_pose.position.y = (pose_goal.position.y + current_position.y)/2.0
      box_pose.position.z = (pose_goal.position.z + current_position.z)/2.0

      pos_const = moveit_msgs.msg.PositionConstraint()
      pos_const.link_name = self.eef_link
      pos_const.header.frame_id = self.planning_frame
      pos_const.constraint_region.primitives.append(primitive)
      pos_const.constraint_region.primitive_poses.append(box_pose)
      pos_const.weight = 0.5

      consts.position_constraints.append(pos_const)

    if self.keep_orient or self.keep_position:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END

    waypoints.append(pose_goal)
    jump_threshold=0.0
    eef_step = 0.02

    ## Now, we call the planner to compute the plan and execute it.
    # self.move_group.go(wait=True)
    user_confirmed = False
    while not user_confirmed:
      # (is_planned, plan, plan_time, err_code) = self.move_group.plan()
      is_planned = True
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold) 

      if (is_planned == False):
        print("PLANNING FAILED")
        wanna_plan_again = self.single_yes_or_no_question("Would you like to try planning again?", default_no=False)
        if wanna_plan_again:
          continue
        else:
          print("planning failed, continuing without execution")
          break
      else:
        self.display_trajectory(plan)
        user_confirmed = self.single_yes_or_no_question("Do you accept the suggested plan?")
    
    if is_planned:
      # save the plan for re use later
      self.plan3 = plan
      # Execute the plan
      self.move_group.execute(self.plan3, wait=True)
      

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # # Also clear path constraints
    if self.keep_orient or self.keep_position:
      self.move_group.clear_path_constraints()

    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
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
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)


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
        # for key, value in attached_objects.items():
        #   print(key, ' : ', value)
        #   print(type(value))
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def add_ground(self, timeout=8):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the end effector:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id =  self.tf_testbedorigin_frame_name # self.planning_frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.305
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = -0.005 
    self.scene.add_box(self.ground_name, box_pose, size=(0.9, 1.1, 0.01))

    return self.wait_for_state_update(self.ground_name, box_is_known=True, timeout=timeout)

  def add_block1(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the end effector:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id =  self.tf_testbedorigin_frame_name # self.planning_frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.305
    box_pose.pose.position.y = 0.305/2.0
    box_pose.pose.position.z = 0.02 
    self.scene.add_box(self.block1_name, box_pose, size=(0.08, 0.44, 0.04))

    return self.wait_for_state_update(self.block1_name, box_is_known=True, timeout=timeout)

  def add_block2(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the end effector:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id =  self.tf_testbedorigin_frame_name # self.planning_frame
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.305
    box_pose.pose.position.y = 0.33
    box_pose.pose.position.z = 0.04 + 0.2
    self.scene.add_box(self.block2_name, box_pose, size=(0.08, 0.04, 0.4))

    return self.wait_for_state_update(self.block2_name, box_is_known=True, timeout=timeout)

  def add_ramp(self, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    

    theta = np.deg2rad(self.ramp_angle)

    Rnew = np.eye(4)
    Rnew[1,1] = cos(theta)
    Rnew[1,2] = -sin(theta)
    Rnew[2,1] = sin(theta)
    Rnew[2,2] = cos(theta)
    qnew = tf_conversions.transformations.quaternion_from_matrix(Rnew) 

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id =  self.tf_pouringpoint_frame_name # self.tf_testbedorigin_frame_name # self.planning_frame

    box_pose.pose.orientation.x = qnew[0]
    box_pose.pose.orientation.y = qnew[1]
    box_pose.pose.orientation.z = qnew[2]
    box_pose.pose.orientation.w = qnew[3]
    
    l = 0.4 # lenght of the ramp
    w = 0.16 # width of the ramp
    thickness = 0.002 # thickness of the ramp
    s = self.pouring_offset_y
    h = self.desired_pouring_height
    temp = l*0.5*sin(theta) - s*tan(theta)

    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = -(temp/tan(theta))
    box_pose.pose.position.z = -(h + temp)
    self.scene.add_box(self.ramp_name, box_pose, size=(w, l, thickness))

    return self.wait_for_state_update(self.ramp_name, box_is_known=True, timeout=timeout)


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

    self.scene.add_object(co)

    return self.wait_for_state_update(self.cylinder_name, box_is_known=True, timeout=timeout)


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
    self.scene.attach_box(self.eef_link, self.cylinder_name, touch_links=touch_links)

    # # create attached collision object
    # aco = moveit_msgs.msg.AttachedCollisionObject()
    # aco.object.id = self.cylinder_name
    # aco.object.operation = moveit_msgs.msg.AttachedCollisionObject().object.ADD
    # aco.link_name = self.eef_link
    # aco.touch_links = touch_links
    # self.scene.attach_object(aco)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.cylinder_name, box_is_attached=True, box_is_known=False, timeout=timeout)


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

  def remove_ground(self, timeout=4):
    self.scene.remove_world_object(self.ground_name)
    return self.wait_for_state_update(self.ground_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_block1(self, timeout=4):
    self.scene.remove_world_object(self.block1_name)
    return self.wait_for_state_update(self.block1_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_block2(self, timeout=4):
    self.scene.remove_world_object(self.block2_name)
    return self.wait_for_state_update(self.block2_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_ramp(self, timeout=4):
    self.scene.remove_world_object(self.ramp_name)
    return self.wait_for_state_update(self.ramp_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def remove_cylinder(self, timeout=4):
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the cylinder from the world.
    self.scene.remove_world_object(self.cylinder_name)
    ## **Note:** The object must be detached before we can remove it from the world

    # We wait for the planning scene to update.
    return self.wait_for_state_update(self.cylinder_name, box_is_attached=False, box_is_known=False, timeout=timeout)

  def print_tolerances(self):
    tol_joints, tol_pos, tol_orient =  self.move_group.get_goal_tolerance()

    print("Joint tolerances: " + str(tol_joints) )
    print("Position tolerances: " + str(tol_pos) )
    print("Orientation tolerances (roll pitch yaw): " + str(tol_orient) )



  def tf2R(self, T):
        # T: transform
        # returns 4x4 rotation matrix (effective R is 3x3 part)

        qw = T.transform.rotation.w # Scalar part of quaternion
        qx = T.transform.rotation.x
        qy = T.transform.rotation.y
        qz = T.transform.rotation.z
        q = [qx,qy,qz,qw]
        R = tf.transformations.quaternion_matrix(q)
        return R # R[:3,:3]

  def single_yes_or_no_question(self, question, default_no=True):
      choices = ' [y/N]: ' if default_no else ' [Y/n]: '
      default_answer = 'n' if default_no else 'y'
      reply = str(input(question + choices)).lower().strip() or default_answer
      if reply[0] == 'y':
          return True
      if reply[0] == 'n':
          return False
      else:
          return False if default_no else True


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

    

    print ("============ Adding Ground plane to the planning scene ...")
    input("============ Press `Enter` to add a ground plane to the planning scene ...")
    result =tutorial.add_ground()
    print("Is add ground plane to the planning scene successful: " + str(result))

    print ("============ Adding block1 to the planning scene ...")
    input("============ Press `Enter` to add block1 to the planning scene ...")
    result =tutorial.add_block1()
    print("Is add block1 to the planning scene successful: " + str(result))

    print ("============ Adding block2 to the planning scene ...")
    input("============ Press `Enter` to add block2 to the planning scene ...")
    result =tutorial.add_block2()
    print("Is add block2 to the planning scene successful: " + str(result))

    print ("============ Adding ramp to the planning scene ...")
    input("============ Press `Enter` to add ramp to the planning scene ...")
    result =tutorial.add_ramp()
    print("Is add ramp to the planning scene successful: " + str(result))

    print ("============ Adding a Cylinder to the planning scene ...")
    input("============ Press `Enter` to add a cylinder/box to the planning scene ...")
    # tutorial.add_cylinder()
    result =tutorial.add_cylinder_with_subframes()
    print("Is add a cylinder/box to the planning scene successful: " + str(result))

    print ("============ Attaching a Cylinder to the Kinova robot ...")
    input("============ Press `Enter` to attach a Cylinder/box to the Kinova robot ...")
    result = tutorial.attach_cylinder()
    print("Is attach a Cylinder/box to the Kinova robot successful: " + str(result))




    print ("============ Press `Enter` to close the gripper ...")
    input()
    result = tutorial.close_gripper()
    print("Is closing gripper result sufficiently close to goal: " + str(result))

    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    input()
    result = tutorial.go_to_joint_state()
    print("Is go_to_joint_state result sufficiently close to goal: " + str(result))



    print ("============ Press `Enter` to execute a movement using a pose goal ...")
    input()
    result = tutorial.go_to_pose_goal()
    # result = tutorial.go_to_pose_goal_cartesian()
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    print ("============ Press `Enter` to execute a movement using a pose goal 2...")
    input()
    result = tutorial.go_to_pose_goal2()
    # result = tutorial.go_to_pose_goal2_cartesian()
    print("Is go_to_pose_goal result sufficiently close to goal 2: " + str(result))

    print ("============ Press `Enter` to execute a movement using a pose goal 3(REVERSE) ...")
    input()
    result = tutorial.go_to_pose_goal3()
    # result = tutorial.go_to_pose_goal3_cartesian()
    print("Is go_to_pose_goal result sufficiently close to goal 3 (REVERSE): " + str(result))






    print ("============ Press `Enter` to execute a movement using a joint state goal AGAIN ...")
    input()
    result = tutorial.go_to_joint_state()
    print("Is go_to_joint_state result sufficiently close to goal: " + str(result))

    print ("============ Press `Enter` to detach the cylinder/box from the Kinova robot ...")
    input()
    tutorial.detach_cylinder()

    # print ("============ Press `Enter` to remove the cylinder/box from the planning scene ...")
    # input()
    tutorial.remove_cylinder()

    # print ("============ Press `Enter` to remove the ramp from the planning scene ...")
    # input()
    tutorial.remove_ramp()
    tutorial.remove_block2()
    tutorial.remove_block1()
    tutorial.remove_ground()


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