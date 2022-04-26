#!/usr/bin/env python3

# Edited file for Safe Energetics Fanuc robot from MoveIt tutorials, Move Group Python Interface (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html )

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
import tf2_geometry_msgs

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
    self.group_name = "fanuc_cage_arm"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    # Define workspace limits
    minX = -8.0
    minY = -8.0
    minZ = -4.0
    maxX = 8.0
    maxY = 8.0
    maxZ = 4.0
    self.move_group.set_workspace([minX, minY, minZ, maxX, maxY, maxZ])

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

    # self.group_name_gripper = "gripper"
    # self.move_group_gripper = moveit_commander.MoveGroupCommander(self.group_name_gripper)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    self.planning_frame = self.move_group.get_planning_frame()
    print ("============ Planning frame: %s" % self.planning_frame)

    self.pose_reference_frame = self.move_group.get_pose_reference_frame()
    print ("============ Pose Reference frame: %s" % self.pose_reference_frame)

    current_pose = self.move_group.get_current_pose()
    print ("============ current_pose: %s" % current_pose)

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
    self.keep_orient = True
    self.keep_position = True

    self.home_pose = None

    self.plan = None # main plan from home to pouring point
    self.plan2 = None # plan for rotation of the pouring box
    self.plan3 = None # plan for rotation of the pouring box (reverse)
    self.rot_angle = 90.0 # deg , angle for amount of robot motion during pouring
    

    # TF2 listener
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    self.pouring_box_tip_name = "fanuc_pouring_box_tip"
    self.pouring_box_bottom_name = "fanuc_pouring_box_bottom"

    self.tf_hopper_long_pouringpoint_frame_name = "hopper_long_pouring_point"
    self.tf_hopper_short_pouringpoint_frame_name = "hopper_short_pouring_point"

    self.tf_cabinet_pickpoint_frame_name_l1p1 = "cabinet_picking_level_1_point_1" # 1
    self.tf_cabinet_pickpoint_frame_name_l1p2 = "cabinet_picking_level_1_point_2" # 2
    self.tf_cabinet_pickpoint_frame_name_l2p1 = "cabinet_picking_level_2_point_1" # 3
    self.tf_cabinet_pickpoint_frame_name_l2p2 = "cabinet_picking_level_2_point_2" # 4

    self.cylinder_name_l1p1 = "cylinder1"
    self.cylinder_name_l1p2 = "cylinder2"
    self.cylinder_name_l2p1 = "cylinder3"
    self.cylinder_name_l2p2 = "cylinder4"

    self.T_world2pouringpoint_long = None # fixed
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
    self.move_group.set_start_state_to_current_state()

    ## the first thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    initial_angles = np.array([-10, -10, 10, -10, -10, -10]) # deg
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

  def go_to_joint_state_zeros(self):
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    self.move_group.set_start_state_to_current_state()

    ## the first thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    initial_angles = np.array([0, 0, 0, 0, 0, 0]) # deg
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

    # Set the home pose ot reach later with constraints
    self.home_pose = self.move_group.get_current_pose()

    print("Home pose: " + str(self.home_pose))
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_tf_pose_goal(self, tf_goal_frame_name, eef_frame_name, is_goal_home=False):
    self.keep_orient = True
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()


    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    if is_goal_home:
      # Let's set the end effector frame different than the default one
      self.move_group.set_end_effector_link(self.eef_link)
      pose_goal_stamped = self.home_pose # in default planning frame (ie. world)
      starting_frame = tf_goal_frame_name
      pose_goal_stamped = self.transform_pose(pose_goal_stamped, starting_frame) # in user specified starting frame

      pose_goal = pose_goal_stamped.pose
      
    else:
      # Let's set the end effector frame different than the default one
      # self.move_group.set_end_effector_link(self.eef_link)
      # self.move_group.set_end_effector_link(self.cylinder_name + "/" + self.pouring_box_tip_name)
      self.move_group.set_end_effector_link(eef_frame_name)

      pose_goal_stamped = geometry_msgs.msg.PoseStamped()
      pose_goal_stamped.header.frame_id = tf_goal_frame_name

      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = 0.0
      pose_goal.orientation.y = 0.0
      pose_goal.orientation.z = 0.0
      pose_goal.orientation.w = 1.0
      pose_goal.position.x = 0.0
      pose_goal.position.y = 0.0
      pose_goal.position.z = 0.0

      pose_goal_stamped.pose = pose_goal

    self.move_group.set_pose_target(pose_goal_stamped)
    # self.move_group.set_pose_target(pose_goal_stamped, end_effector_link=self.cylinder_name + "/" + self.pouring_box_tip_name)

    #~~~~~~~~~~~~ Setup constraints - BEGIN
    consts = moveit_msgs.msg.Constraints()

    if self.keep_orient:
      # create a path constraint
      orien_const = moveit_msgs.msg.OrientationConstraint()
      if is_goal_home:
        orien_const.link_name = self.eef_link
        orien_const.header.frame_id = starting_frame
      else:
        orien_const.link_name = eef_frame_name # self.eef_link
        orien_const.header.frame_id = tf_goal_frame_name
      

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
      

    if self.keep_orient:
      # print(consts)
      self.move_group.set_path_constraints(consts)

      # Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      self.move_group.set_planning_time(15.0)
    #~~~~~~~~~~~~ Setup constraints - END

    self.move_group.set_planning_time(15.0)
    # Before planning make sure you set the start state as the current state
    self.move_group.set_start_state_to_current_state()

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
    if self.keep_orient:
      self.move_group.clear_path_constraints()

    # For testing:
    # print ("============ End effector link: %s" % self.move_group.get_end_effector_link())

    current_pose = self.move_group.get_current_pose().pose # in planning frame
    # current_pose = self.move_group.get_current_pose(end_effector_link=self.cylinder_name + "/" + self.pouring_box_tip_name).pose # in planning frame
    # print("current_pose: " + str(current_pose))
    pose_goal_in_planning_frame = self.transform_pose(pose_goal_stamped, self.planning_frame).pose
    # print("pose_goal_in_planning_frame: " + str(pose_goal_in_planning_frame))
    return all_close(pose_goal_in_planning_frame, current_pose, 0.01)


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
    cylinder_pose.pose.position.y = height/2.0 # 
    cylinder_pose.pose.position.z = -0.006 # 
    
    
    self.scene.add_cylinder(self.cylinder_name, cylinder_pose, height, radius)

    return self.wait_for_state_update(self.cylinder_name, box_is_known=True, timeout=timeout)

  def add_cylinder_with_subframes(self, frame_to_add, obj_name_to_add, timeout=4):
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a cylinder in the planning scene at the location of the end effector:
    height = 0.129
    radius = 0.039 

    co = moveit_msgs.msg.CollisionObject()
    co.operation = moveit_msgs.msg.CollisionObject().ADD # co.ADD
    co.id = obj_name_to_add
    co.header.frame_id = frame_to_add

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
    cylinder_pose.pose.position.y = height/2.0 # 
    cylinder_pose.pose.position.z = 0.0 # -0.006 # 
    co.primitive_poses = [cylinder_pose.pose]

    # By default, the defined subframe poses are wrt. the frame of the attached objects (e.g the cylinder's)
    # some some thinking is be needed for the subframe poses to set them correctly
    co.subframe_names = [self.pouring_box_tip_name]

    subframe_pose = geometry_msgs.msg.PoseStamped()
    subframe_pose.pose.orientation.x = 0.5
    subframe_pose.pose.orientation.y = 0.5
    subframe_pose.pose.orientation.z = 0.5
    subframe_pose.pose.orientation.w = 0.5

    subframe_pose.pose.position.x = 0.0
    subframe_pose.pose.position.y = -0.055
    subframe_pose.pose.position.z = height/2.0
    co.subframe_poses = [subframe_pose.pose]

    self.scene.add_object(co)

    return self.wait_for_state_update(obj_name_to_add, box_is_known=True, timeout=timeout)


  def attach_cylinder(self, obj_name_to_attach, timeout=4):
    ## Attaching CylinderObjects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the cylinder box to the Fanuc end effector. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    # grasping_group = 'gripper'
    # touch_links = self.robot.get_link_names(group=grasping_group)
    touch_links = self.robot.get_link_names()
    
    # self.move_group.attach_object(self.cylinder_name,self.eef_link,touch_links=touch_links)
    self.scene.attach_box(self.eef_link, obj_name_to_attach, touch_links=touch_links)

    # # create attached collision object
    # aco = moveit_msgs.msg.AttachedCollisionObject()
    # aco.object.id = self.cylinder_name
    # aco.object.operation = moveit_msgs.msg.AttachedCollisionObject().object.ADD
    # aco.link_name = self.eef_link
    # aco.touch_links = touch_links
    # self.scene.attach_object(aco)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(obj_name_to_attach, box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_cylinder(self, obj_name_to_detach, timeout=4):
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    self.scene.remove_attached_object(self.eef_link, name=obj_name_to_detach)
    # self.move_group.detach_object(name=self.cylinder_name)

    # Do not forget to reset your end_effector_link to a robot link when you detach your object
    self.move_group.set_end_effector_link(self.eef_link)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(obj_name_to_detach, box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_cylinder(self, obj_name_to_remove, timeout=4):
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the cylinder from the world.
    self.scene.remove_world_object(obj_name_to_remove)
    ## **Note:** The object must be detached before we can remove it from the world

    # We wait for the planning scene to update.
    return self.wait_for_state_update(obj_name_to_remove, box_is_attached=False, box_is_known=False, timeout=timeout)

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

  def transform_pose(self, input_pose_stamped, to_frame): 
    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = self.tfBuffer.transform(input_pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

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
    print ("Welcome to the MoveIt MoveGroup Python Interface Test for Safe Energetics Testbed")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    
    ########################### ADD COLLISION BOXES TO THE SCENE #######################
    print ("============ Adding a Cylinder to the planning scene ...")
    input("============ Press `Enter` to add a cylinder/box 1 to the planning scene ...")
    # tutorial.add_cylinder()
    result =tutorial.add_cylinder_with_subframes(tutorial.tf_cabinet_pickpoint_frame_name_l1p1, tutorial.cylinder_name_l1p1)
    print("Is add a cylinder/box to the planning scene successful: " + str(result))

    input("============ Press `Enter` to add a cylinder/box 2 to the planning scene ...")
    # tutorial.add_cylinder()
    result =tutorial.add_cylinder_with_subframes(tutorial.tf_cabinet_pickpoint_frame_name_l1p2, tutorial.cylinder_name_l1p2)
    print("Is add a cylinder/box to the planning scene successful: " + str(result))

    input("============ Press `Enter` to add a cylinder/box 3 to the planning scene ...")
    # tutorial.add_cylinder()
    result =tutorial.add_cylinder_with_subframes(tutorial.tf_cabinet_pickpoint_frame_name_l2p1, tutorial.cylinder_name_l2p1)
    print("Is add a cylinder/box to the planning scene successful: " + str(result))

    input("============ Press `Enter` to add a cylinder/box 4 to the planning scene ...")
    # tutorial.add_cylinder()
    result =tutorial.add_cylinder_with_subframes(tutorial.tf_cabinet_pickpoint_frame_name_l2p2, tutorial.cylinder_name_l2p2)
    print("Is add a cylinder/box to the planning scene successful: " + str(result))

    ########################### MOVE ROBOT TO HOME POSITION #######################
    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    input()
    result = tutorial.go_to_joint_state()
    print("Is go_to_joint_state result sufficiently close to goal: " + str(result))

    print ("============ Press `Enter` to execute a movement using a joint state goal (ZEROS) ...")
    input()
    result = tutorial.go_to_joint_state_zeros()
    print("Is go_to_joint_state (ZEROS result sufficiently close to goal: " + str(result))

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ POINT 1 - LONG HOPPER: BEGIN~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ########################### MOVE ROBOT TO PICK UP POSITION 1 #######################
    # TODO
    print ("============ Press `Enter` to execute a movement to: '"+ tutorial.tf_cabinet_pickpoint_frame_name_l1p1  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p1, tutorial.pouring_box_bottom_name)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### ATTACH BOX 1 TO THE ROBOT #######################
    print ("============ Attaching a Cylinder to the Fanuc robot ...")
    input("============ Press `Enter` to attach a Cylinder/box 1 to the Fanuc robot ...")
    result = tutorial.attach_cylinder(tutorial.cylinder_name_l1p1)
    print("Is attach a Cylinder/box to the Fanuc robot successful: " + str(result))

    ########################### MOVE THE ROBOT TO HOME POSE WITH BOX 1 #######################
    print ("============ Press `Enter` to execute a movement to: '"+ "HOME"  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p1, tutorial.eef_link, is_goal_home=True)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### MOVE THE ROBOT TO LONG HOPPER WITH BOX 1 #######################
    print ("============ Press `Enter` to execute a movement to: '"+ tutorial.tf_hopper_long_pouringpoint_frame_name  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_hopper_long_pouringpoint_frame_name, tutorial.pouring_box_tip_name)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### POUR TO LONG HOPPER WITH BOX 1 #######################
    # TODO

    ########################### REVERT POURING LONG HOPPER WITH BOX 1 #######################
    # TODO

    ########################### MOVE THE ROBOT TO HOME POSE WITH BOX 1 AGAIN #######################
    print ("============ Press `Enter` to execute a movement to: '"+ "HOME"  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_hopper_long_pouringpoint_frame_name, tutorial.eef_link, is_goal_home=True)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### MOVE ROBOT TO PICK UP POSITION 1 AGAIN #######################
    # TODO
    print ("============ Press `Enter` to execute a movement to: '"+ tutorial.tf_cabinet_pickpoint_frame_name_l1p1  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p1, tutorial.pouring_box_bottom_name)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### DETACH THE BOX 1 FROM ROBOT #######################
    print ("============ Press `Enter` to detach the cylinder/box from the Fanuc robot ...")
    input()
    tutorial.detach_cylinder(tutorial.cylinder_name_l1p1)

    ########################### MOVE THE ROBOT TO HOME POSE FREELY FROM PICKING POINT 1 #######################
    print ("============ Press `Enter` to execute a movement to: '"+ "HOME"  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p1, tutorial.eef_link, is_goal_home=True)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ POINT 2 - SHORT HOPPER: BEGIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    ########################### MOVE ROBOT TO PICK UP POSITION 2 #######################
    # TODO
    print ("============ Press `Enter` to execute a movement to: '"+ tutorial.tf_cabinet_pickpoint_frame_name_l1p2  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p2, tutorial.pouring_box_bottom_name)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### ATTACH BOX 2 TO THE ROBOT #######################
    print ("============ Attaching a Cylinder to the Fanuc robot ...")
    input("============ Press `Enter` to attach a Cylinder/box 2 to the Fanuc robot ...")
    result = tutorial.attach_cylinder(tutorial.cylinder_name_l1p2)
    print("Is attach a Cylinder/box to the Fanuc robot successful: " + str(result))

    ########################### MOVE THE ROBOT TO HOME POSE WITH BOX 2 #######################
    print ("============ Press `Enter` to execute a movement to: '"+ "HOME"  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p2, tutorial.eef_link, is_goal_home=True)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### MOVE THE ROBOT TO SHORT HOPPER WITH BOX 2 #######################
    print ("============ Press `Enter` to execute a movement to: '"+ tutorial.tf_hopper_short_pouringpoint_frame_name  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_hopper_short_pouringpoint_frame_name, tutorial.pouring_box_tip_name)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### POUR TO SHORT HOPPER WITH BOX 2 #######################
    # TODO

    ########################### REVERT POURING SHORT HOPPER WITH BOX 2 #######################
    # TODO

    ########################### MOVE THE ROBOT TO HOME POSE WITH BOX 2 AGAIN #######################
    print ("============ Press `Enter` to execute a movement to: '"+ "HOME"  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_hopper_short_pouringpoint_frame_name, tutorial.eef_link, is_goal_home=True)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### MOVE ROBOT TO PICK UP POSITION 2 AGAIN #######################
    # TODO
    print ("============ Press `Enter` to execute a movement to: '"+ tutorial.tf_cabinet_pickpoint_frame_name_l1p2  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p2, tutorial.pouring_box_bottom_name)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))

    ########################### DETACH THE BOX 2 FROM ROBOT #######################
    print ("============ Press `Enter` to detach the cylinder/box from the Fanuc robot ...")
    input()
    tutorial.detach_cylinder(tutorial.cylinder_name_l1p2)

    ########################### MOVE THE ROBOT TO HOME POSE FREELY FROM PICKING POINT 1 #######################
    print ("============ Press `Enter` to execute a movement to: '"+ "HOME"  +"' ...")
    input()
    result = tutorial.go_to_tf_pose_goal(tutorial.tf_cabinet_pickpoint_frame_name_l1p2, tutorial.eef_link, is_goal_home=True)
    print("Is go_to_pose_goal result sufficiently close to goal: " + str(result))


    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



    ########################### REMOVE COLLISION BOXES FROM THE SCENE #######################
    tutorial.remove_cylinder(tutorial.cylinder_name_l1p1)
    tutorial.remove_cylinder(tutorial.cylinder_name_l1p2)
    tutorial.remove_cylinder(tutorial.cylinder_name_l2p1)
    tutorial.remove_cylinder(tutorial.cylinder_name_l2p2)


    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
