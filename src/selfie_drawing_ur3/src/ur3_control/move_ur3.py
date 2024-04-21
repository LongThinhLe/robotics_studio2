#!/usr/bin/env python3

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
import numpy as np

import threading 

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a numpy array, a Pose or a PoseStamped
  @param: actual     A list of floats, a numpy array, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if isinstance(goal, (list, np.ndarray)) and isinstance(actual, (list, np.ndarray)):
      # If both are lists or numpy arrays, compare element-wise
      if isinstance(goal, list):
          goal = np.array(goal)
      if isinstance(actual, list):
          actual = np.array(actual)
      return np.all(np.abs(actual - goal) <= tolerance)
  elif isinstance(goal, geometry_msgs.msg.PoseStamped) and isinstance(actual, geometry_msgs.msg.PoseStamped):
      return all_close(goal.pose, actual.pose, tolerance)
  elif isinstance(goal, geometry_msgs.msg.Pose) and isinstance(actual, geometry_msgs.msg.Pose):
      # Compare positions
      if not all_close_position(goal.position, actual.position, tolerance):
          return False
      # Compare orientations (quaternions)
      if not all_close_orientation(goal.orientation, actual.orientation, tolerance):
          return False
  else:
      # Unsupported types
      return False
  return True

def all_close_position(goal_pos, actual_pos, tolerance):
    return np.allclose([goal_pos.x, goal_pos.y, goal_pos.z], [actual_pos.x, actual_pos.y, actual_pos.z], atol=tolerance)

def all_close_orientation(goal_ori, actual_ori, tolerance):
    goal_quat = np.array([goal_ori.x, goal_ori.y, goal_ori.z, goal_ori.w])
    actual_quat = np.array([actual_ori.x, actual_ori.y, actual_ori.z, actual_ori.w])
    # Calculate the quaternion difference
    quaternion_diff = np.abs(goal_quat - actual_quat)
    # Ensure that quaternion_diff is within 2*pi of the correct value
    quaternion_diff = np.minimum(quaternion_diff, 2 * np.pi - quaternion_diff)
    # Check if all elements of the quaternion difference are within tolerance
    return np.all(quaternion_diff < tolerance)

class UR3_Movement(object):

  def __init__(self):
    try:
        # First, initialize rospy node
        rospy.init_node('selfie_drawing_ur3_movement', anonymous=True)
        # Then initialize MoveIt components
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate MoveIt objects
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Store objects as attributes
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher

        # Print information for debugging
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        print("============ Planning frame:", planning_frame)
        print("============ End effector link:", eef_link)
        print("============ Available Planning Groups:", group_names)
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.target_pose = None
        self.move_thread = None
        self.paused_pose = None
        self.goal_pose_list = [] # List of goal pose

        self.stop_flag = threading.Event()  # Event to signal stop
        self.stop_flag.clear()



        self.completeGoal_flag = threading.Event()
        self.completeGoal_flag.clear() # clear is not finished, set is finished

        self._is_running = threading.Event()
        self._is_running.clear() # clear is not running, set is running

        self.check_goal_reached_thread = threading.Thread(target=self._check_goal_reached)
        self.check_goal_reached_thread.start()

        # self.check_goals_thread = threading.Thread(target=self._check_goal_pose_list)
        # self.check_goals_thread.start()


    except rospy.ROSException as e:
        print("Error initializing UR3_Movement:", str(e))


  def _check_goal_pose_list(self):
    """
    Check if there is a goal pose/joint in self.goal_pose_list.
    If a goal exists, set it as self.target_pose for robot movement.
    If the goal is a pose, convert it to a numpy array before setting.
    """
    rate = rospy.Rate(10)
    while True:
      if self.goal_pose_list:
        # Get the first goal from the list
        goal = self.goal_pose_list[0]
        self.target_pose = goal

        # Wait for completion
        self.completeGoal_flag.wait()



        # REmove the goal from the list
        self.goal_pose_list.pop(0)
      

        # if isinstance(goal, geometry_msgs.msg.Pose):
        #   # If the goal is a pose, convert it to a numpy array
        #   self.target_pose = pose_to_list(goal)
        # else:
        #   # If the goal is a joint, directly set it as the target pose
        #   self.target_pose = goal


      # if len(self.goal_pose_list) != 0: # = if self.goal_pose_list : is empty / if not self.goal_pose_list : is not empty
      #   print("inside goal pose list")
      #   if self.target_pose is None:
      #     print("Set Target Pose\n")
      #     # Remove and return the first goal pose from the list
      #     next_goal_pose = self.goal_pose_list.pop(0)

      #     # Assign the first goal pose to self.target_pose
      #     self.target_pose = next_goal_pose
      #     print(self.target_pose)

      rate.sleep()

#------------------- Draw from Gcode file

  def test_drawing_cartesian_path(self,scale = 1):
    waypoints = []

    wpose = self.move_group.get_current_pose().pose

    list_length = len(self.goal_pose_list[0])
    for i in range (min(10, list_length)):
      wpose.position.z = scale * self.goal_pose_list[0][i][2]
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = scale * self.goal_pose_list[0][i][0]
      wpose.position.y = scale * self.goal_pose_list[0][i][1]
      waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
    self.move_group.execute(plan, wait = False)

    # Still use minus offset !!!




    # waypoints = []

    # current_pose = self.move_group.get_current_pose().pose
    # current_x = current_pose.position.x
    # current_y = current_pose.position.y
    # current_z = current_pose.position.z

    # print ("Current Z = ", current_z)
    # print ("Next Z = ", self.goal_pose_list[0][0][2])

    # offset_x = self.goal_pose_list[0][0][0] - current_x
    # offset_y = self.goal_pose_list[0][0][1] - current_y
    # offset_z = self.goal_pose_list[0][0][2] - current_z

    # print ("Offset z = ", offset_z)


    # wpose = copy.deepcopy(current_pose)
    # # Update z position
    # wpose.position.z = scale * offset_z
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x = scale * offset_x
    # wpose.position.y = scale * offset_y
    # waypoints.append(copy.deepcopy(wpose))

    # print ("Waypoint position z = ", wpose.position.z)

    # list_length = len(self.goal_pose_list[0]) 
    # for i in range (min(10, list_length)):
    #   for j in range(3):
        


    # waypoints = []

    # # Extract current position
    # current_pose = self.move_group.get_current_pose().pose
    # current_x = current_pose.position.x
    # current_y = current_pose.position.y
    # current_z = current_pose.position.z

    # offset_x = self.goal_pose_list[0][0] - current_x
    # offset_y = goal_pose[1] - current_y
    # offset_z = goal_pose[2] - current_z



    # # Loop through goal poses
    # for goal_pose in self.goal_pose_list[0][:10]:
    #   # Calculate offset
    #   offset_x = goal_pose[0] - current_x
    #   offset_y = goal_pose[1] - current_y
    #   offset_z = goal_pose[2] - current_z


    #   # Update current position
    #   current_x = goal_pose[0]
    #   current_y = goal_pose[1]
    #   current_z = goal_pose[2]

    #   # Create a new waypoint
    #   wpose = copy.deepcopy(current_pose)

    #   # Update z position
    #   wpose.position.z += scale * offset_z
    #   waypoints.append(copy.deepcopy(wpose))

    #   # Update x position
    #   wpose.position.x += scale * offset_x
    #   wpose.position.y += scale * offset_y
    #   waypoints.append(copy.deepcopy(wpose))

    # print ("\n\nTHIS IS OUR WAY POINTS HAHA: \n")
    # for point in waypoints:
    #    print(f"X: {point.position.x}, Y: {point.position.y}, Z: {point.position.z}")

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)

    # self.move_group.execute(plan, wait=False)
     

    # TESTING HERE:
    # waypoints = []

    # wpose = self.move_group.get_current_pose().pose
    # wpose.position.z += scale * 0.02
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.y += scale * 0.15
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.y -= scale * 0.3
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.y += scale * 0.15
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.z -= scale * 0.02
    # waypoints.append(copy.deepcopy(wpose))

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)

    # self.move_group.execute(plan, wait = False)
    # print ("Self Goal Pose List--------------: ", self.goal_pose_list)

    # print("self.goal pose list [0] ::::: ", self.goal_pose_list[0][0])
    # print("self.goal pose list [1] ::::: ", self.goal_pose_list[0][1])
    # pose_difference = [
    #   self.goal_pose_list[0][1][0] - self.goal_pose_list[0][0][0], # X difference
    #   self.goal_pose_list[0][1][1] - self.goal_pose_list[0][0][1], # Y difference
    #   self.goal_pose_list[0][1][2] - self.goal_pose_list[0][0][2]  # Z difference
    # ]
    # print("self.goal pose list [1-0] ::::: ", pose_difference)

  def set_pose_goals_list(self, pose_goal_positions):
    self.goal_pose_list.append(pose_goal_positions)

    print ("Self Goal Pose List: ", self.goal_pose_list)


#-------------------  Movement threading
  def homing_ur3(self):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_deg = [20, -100, -125, -26, -280, 20]
    joint_goal = np.deg2rad(joint_deg)

    # self.goal_pose_list.append(joint_goal)
    self.start_movement(joint_goal)


  def start_movement(self, target_pose): # THIS FUNCTION IS USED FOR STARTING THE THREAD TO ROBOT GO TO PRESET GOAL
    self.target_pose = target_pose
    self.move_thread = threading.Thread(target=self._move_to_target) # Start and Stop this thread to run the robot
    self.move_thread.start()

  def stop_movement(self): # Stop the robot and stop the thread (join)
    self.stop_flag.set()
    if self.move_thread:
        self.move_thread.join()
        self.move_group.stop()

  def release_stop_event(self):
    # Reset the stop flag after homing
    print("\n----------Clear Stop Flag----------\n")
    self.stop_flag.clear()

  def _check_goal_reached(self):
    rate = rospy.Rate(10)
    while True:
      # ---------------------------- CHECK IF THE GOAL IS REACHED -> USING SELF.TARGET_POSE
      if np.all(self.target_pose):
        # ----- Check if the current pose and the target pose is equal
        current_joints = self.move_group.get_current_joint_values()
        if all_close(self.target_pose, current_joints, 0.01):
          self.completeGoal_flag.set()
          print("\n----------------\nGoal Joint is reached !!!\n------------------\n")
          self.target_pose = None

      elif isinstance(self.target_pose, geometry_msgs.msg.Pose):
        current_pose = self.move_group.get_current_pose().pose
        if all_close(self.target_pose, current_pose, 0.01):
          self.completeGoal_flag.set()
          print("\n----------------\nGoal Pose is reached !!!\n------------------\n")
          self.target_pose = None

      rate.sleep()

  def _move_to_target(self):  ## FIX PLAN CARTERSIAN AND EXECUTE PLAN INSTEAD OF STATIC JOINT ANGLES
    rate = rospy.Rate(10)  # Adjust the rate based on your requirements
    print ("Go into move to target thread \n")
    target_pose = self.target_pose
    self.completeGoal_flag.clear() # Always clear Complete Goal Flag before moving to goal

    while not self.stop_flag.is_set():

      # ---------------------------- SET PLAN THE TARGET POSE COPY ONLY ONCE
      if np.all(target_pose): # ---- Joint angles
        print("\n--------------------------\nRobot is running... !!!\n--------------------------\n")
        # ----- Move to joint target
        self.move_group.set_joint_value_target(target_pose)
        plan = self.move_group.go(wait=False)
        if plan:
          self.move_group.stop()  # Stop any ongoing movement
          target_pose = None  # Reset target pose after movement

      elif isinstance(target_pose, geometry_msgs.msg.Pose): # ----- Pose target
        print("\n--------------------------\nRobot is running... !!!\n--------------------------")
        # ----- Move to pose target
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=False)
        if plan:
          self.move_group.stop()
          target_pose = None

      rate.sleep()  # Ensure that the loop runs at a specific rate INSIDE THE LOOP WHILE

    print("\n--------------------------\nRobot Stops !!!\n--------------------------")

 


#------------------- Update TCP threading
  def update_robot_tcp_thread(self):
    self.update_tcp_thread = threading.Thread(target= self._get_robot_tcp)
    self.update_tcp_thread.start()

  def get_tcp(self):

    return self.tcp_pose_x, self.tcp_pose_y, self.tcp_pose_z, self.tcp_ori_x, self.tcp_ori_y, self.tcp_ori_z

  def _get_robot_tcp(self):
    rate = rospy.Rate(10)

    while True:
      self.tcp_pose = self.move_group.get_current_pose().pose
      tcp_pose = self.tcp_pose
      self.tcp_pose_x = tcp_pose.position.x
      self.tcp_pose_y = tcp_pose.position.y
      self.tcp_pose_z = tcp_pose.position.z

      self.tcp_ori_x = tcp_pose.orientation.x
      self.tcp_ori_y = tcp_pose.orientation.y
      self.tcp_ori_z = tcp_pose.orientation.z

      rate.sleep()


