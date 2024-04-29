#!/usr/bin/env python3

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint

import numpy as np
import time
import math

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
        # rospy.init_node('selfie_drawing_ur3_movement', anonymous=True)
        
        # Then initialize MoveIt components
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate MoveIt objects
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander("manipulator")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        orientation_constraint = OrientationConstraint()
        path_constraints = Constraints()

        # Store objects as attributes
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.orientation_constraint = orientation_constraint
        self.path_constraints = path_constraints

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

        # Adjust the tolerance of reaching goal !!!!!!!!!!!!!!!!!!!!!!!!
        self.move_group.set_goal_tolerance(0.00001)
        self.move_group.set_max_velocity_scaling_factor(0.2)

        self.target_pose = None
        self.move_thread = None
        self.paused_pose = None
        self.goal_pose_list = [] # List of goal pose

        self.stop_flag = threading.Event()  # Event to signal stop
        self.stop_flag.clear()

        self.completeGoal_flag = threading.Event()
        self.completeGoal_flag.clear() # clear is not finished, set is finished

        self.set_pos_goal_once = threading.Event() # Running one time for move to target with pose goal
        self.set_pos_goal_once.clear()

        self.enable_check_event = threading.Event() # Flag for checking goal
        self.enable_check_event.set()

        self.check_goal_reached_thread = threading.Thread(target=self._check_goal_reached)
        self.check_goal_reached_thread.start()

        self.home_joint_angle = [20, -100, -125, -26, -280, 20] #0.349, -1.7453, -2.1816, -0.4537, -4.8869, 0.349
        self.home_joint_angle = np.deg2rad(self.home_joint_angle)

    except rospy.ROSException as e:
        print("Error initializing UR3_Movement:", str(e))


#------------------- Draw from Gcode file
  def start_drawing(self):


    # Start the thread drawing
    self.check_goals_thread = threading.Thread(target=self._check_goal_pose_list)
    self.check_goals_thread.start()
    
  def set_pose_goals_list(self, pose_goal_positions):
    self.goal_pose_list.clear()
    self.goal_pose_list.append(pose_goal_positions)
    self.enable_check_event.set()
    print ("\nAll goals are imported !")

  def _check_goal_pose_list(self):
    """
    Check if there is a goal pose/joint in self.goal_pose_list.
    If a goal exists, set it as self.target_pose for robot movement.
    If the goal is a pose, convert it to a numpy array before setting.
    """
    rate = rospy.Rate(10)
    print ("Go into the thread check goal")
    count_goal = 0
    while self.enable_check_event.is_set():
      # print ("STILL HERE" , self.goal_pose_list)
      if self.goal_pose_list:
        try:
          time.sleep(0.2)
          count_goal += 1
          goal = self.goal_pose_list[0][0]
          self.start_movement(goal)
          # Get the first goal from the list
          print (count_goal, ".New Goal: ", goal)
          # Wait for completion
          self.completeGoal_flag.wait()
          print ("Finish check Goal")
          # time.sleep(0.2)
          self.goal_pose_list[0].pop(0)
          print ("Pop the firts goal")
          time.sleep(0.3)
        except:
          print("Finish all goals")
          self.homing_ur3()
          self.enable_check_event.clear()

      rate.sleep()


#-------------------  Movement threading
  def homing_ur3(self):
    self.clear_orientation_constraints()
    # joint_deg = [20, -100, -125, -26, -280, 20] #0.349, -1.7453, -2.1816, -0.4537, -4.8869, 0.349
    # joint_goal = np.deg2rad(joint_deg)
    self.start_movement(self.home_joint_angle)

  def start_movement(self, target_pose): # THIS FUNCTION IS USED FOR STARTING THE THREAD TO ROBOT GO TO PRESET GOAL
    self.target_pose = target_pose
    self.completeGoal_flag.clear() # Always clear Complete Goal Flag before moving to goal
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

  def set_origin_pose(self):
    # Set orientation constraint
    self.orientation_constraint.header.frame_id = self.move_group.get_planning_frame()
    self.orientation_constraint.link_name = self.move_group.get_end_effector_link()
    self.start_pose = self.move_group.get_current_pose().pose
    self.orientation_constraint.orientation = self.start_pose.orientation # Keep the current orientation for the next goal
    self.orientation_constraint.absolute_x_axis_tolerance = 0.05
    self.orientation_constraint.absolute_y_axis_tolerance = 0.05
    self.orientation_constraint.absolute_z_axis_tolerance = 0.05
    self.orientation_constraint.weight = 1.0 # 1.0 is fully considered during motion planning

    # Create path constraints
    self.path_constraints.orientation_constraints.append(self.orientation_constraint)
    self.move_group.set_path_constraints(self.path_constraints)

    self.target_pose = None
    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation
    print ("This is origin orientation: \n", self.start_orientation)


  def clear_orientation_constraints(self):
    # Clear orientation constraint after use
    self.path_constraints.orientation_constraints = []
    self.move_group.set_path_constraints(self.path_constraints)

  def clear_all_goals(self):
    self.goal_pose_list.clear()
    self.enable_check_event.clear()

  def move_with_orientation_constraint(self, target_pose, scale = 1):
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    wpose.position.z = target_pose[2]
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = target_pose[0]
    wpose.position.y = target_pose[1]
    wpose.orientation.x = self.start_orientation.x
    wpose.orientation.y = self.start_orientation.y
    wpose.orientation.z = self.start_orientation.z
    wpose.orientation.w = self.start_orientation.w
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= 0.001, jump_threshold= 0.0)
    if fraction >= 0.2:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()
      
    else: print("----------------\nFailed to plan the trajectory!\n------------------")
    del waypoints
    del wpose

  def _check_goal_reached(self): ########
    rate = rospy.Rate(10)
    while True:
      
      # ---------------------------- CHECK IF THE GOAL IS REACHED -> USING SELF.TARGET_POSE
      if np.all(self.target_pose):
        # print ("Check length self.target pose = ", len(self.target_pose))
        if len(self.target_pose) == 6:
          # ----- Check if the current pose and the target pose is equal
          current_joints = self.move_group.get_current_joint_values()
          if all_close(self.target_pose, current_joints, 0.003):
            print("\n----------------\nGoal Joint is reached !!!\n------------------\n")
            self.target_pose = None
            self.completeGoal_flag.set()
            if self.move_thread.is_alive():
              self.move_thread.join()
              self.move_group.stop()


        elif len(self.target_pose) == 3: 
          # ----- Check if the position is reached
          current_position = []
          current_x = self.move_group.get_current_pose().pose.position.x
          current_position.append(current_x)
          current_y = self.move_group.get_current_pose().pose.position.y
          current_position.append(current_y)
          current_z = self.move_group.get_current_pose().pose.position.z
          current_position.append(current_z)

          if all_close(self.target_pose, current_position, 0.005):
            self.target_pose = None
            self.completeGoal_flag.set()
            # time.sleep(0.3)
            if self.move_thread.is_alive():
              self.move_thread.join()
              self.move_group.stop()
            print("\n----------------\nGoal Position is reached !!!\n------------------\n")
            del current_position
            


      elif isinstance(self.target_pose, geometry_msgs.msg.Pose):
        current_pose = self.move_group.get_current_pose().pose
        if all_close(self.target_pose, current_pose, 0.003):
          print("\n----------------\nGoal Pose is reached !!!\n------------------\n")
          self.target_pose = None
          self.completeGoal_flag.set()
          if self.move_thread.is_alive():
            self.move_thread.join()
            self.move_group.stop()

      rate.sleep()


  ####################
  def _move_to_target(self):  ## FIX PLAN CARTERSIAN AND EXECUTE PLAN INSTEAD OF STATIC JOINT ANGLES
    rate = rospy.Rate(10)  # Adjust the rate based on your requirements
    print ("\nGo into move to target thread \n")
    target_pose = self.target_pose
    self.set_pos_goal_once.clear() 

    while not self.stop_flag.is_set() and not self.completeGoal_flag.is_set():

      # ---------------------------- SET PLAN THE TARGET POSE COPY ONLY ONCE
      if np.all(target_pose): # ---- Joint angles
        if len(target_pose) == 6:
          print("\n--------------------------\nRobot is running... !!!\n--------------------------\n")
          # ----- Move to joint target
          plan = self.move_group.go(target_pose, wait=False)
          if plan:
            self.move_group.stop()  # Stop any ongoing movement
            target_pose = None  # Reset target pose after movement

        elif len(target_pose) == 3:
          if not self.set_pos_goal_once.is_set():
            print("\n--------------------------\nRobot is running... !!!\n--------------------------\n")
            self.move_with_orientation_constraint(target_pose)
            self.set_pos_goal_once.set() # Set this flag so that this condition will run one time
          # else: print("It's stop here")

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

 

#------------------- Changing Pen
  def change2leftpen(self):
    current_joint = self.move_group.get_current_joint_values()
    print("\nCurrent Joint = \n", current_joint)

    # Access to the last joint angle
    current_joint[-1] += math.radians(90)

    if current_joint[-1] >= 2*math.pi:
      print("\nThe TCP joint angle exceeds Joint Limit ! Cannot move !")
      current_joint[-1] -= math.radians(90)

    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation

    self.start_movement(current_joint)


  def change2rightpen(self):
    current_joint = self.move_group.get_current_joint_values()
    print("\nCurrent Joint = \n", current_joint)

    # Access to the last joint angle
    current_joint[-1] -= math.radians(90)

    if current_joint[-1] <= -2*math.pi:
      print("\nThe TCP joint angle exceeds Joint Limit ! Cannot move !")
      current_joint[-1] += math.radians(90)

    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation

    self.start_movement(current_joint)


  def change2Pen1(self):
    current_joint = copy.deepcopy(self.home_joint_angle)
    current_joint[-1] = math.radians(20)
    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation
    self.start_movement(current_joint)


  def change2Pen2(self):
    current_joint = copy.deepcopy(self.home_joint_angle)
    current_joint[-1] = math.radians(110)
    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation
    self.start_movement(current_joint)

  
  def change2Pen3(self):
    current_joint = copy.deepcopy(self.home_joint_angle)
    current_joint[-1] = math.radians(-70)
    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation
    self.start_movement(current_joint)



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


