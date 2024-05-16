#!/usr/bin/env python3

import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import PoseStamped


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
       
        path_constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        position_constraint = PositionConstraint()

        # Store objects as attributes
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        
        self.path_constraints = path_constraints
        self.orientation_constraint = orientation_constraint
        self.position_constraint = position_constraint

        # Print information for debugging
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = robot.get_group_names()
        

        print("============ Planning frame:", self.planning_frame)
        print("============ End effector link:", self.eef_link)
        print("============ Available Planning Groups:", self.group_names)
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Path constraints
        self.path_constraints.name = "path_constraint"

        # Setting for Movegroup API
        self.move_group.set_goal_tolerance(0.00001)
        self.move_group.set_goal_joint_tolerance(0.000001)
        self.move_group.set_max_velocity_scaling_factor(0.5) # 0.5
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_planning_time(10)  # Adjust the planning time as needed
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_planner_id("RRTConnect")
        print("\nMove Group Get planner ID ---: ", self.move_group.get_planner_id())

        # print("\nJoint state name: ", self.move_group.get_current_state().joint_state.name)

        self.joint_names_list = self.move_group.get_current_state().joint_state.name

        self.target_pose = None
        self.move_thread = None
        self.paused_pose = None
        self.goal_pose_list = [] # List of goal pose

        self.timer_exceed = threading.Event()
        self.timer_exceed.clear()

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

        # This joint angle is perfect for 150mm Square without self-collision: [0.3438, -1.7949, -2.1418, -0.4759, -4.8844, -5.9389]
        # ur3e : [-1.122, -1.7949, -2.1418, -0.4759, -4.8844, 0.349]
        # ur3e further: [-1.17545, -1.94233, -1.93044, -0.53136, -4.86835, 0.2972]
        self.home_joint_angle_original = [-1.17545, -1.94233, -1.93044, -0.53136, -4.86835, 0.2972] # ur3[0.3438, -1.7949, -2.1418, -0.4759, -4.8844, 0.349] #[20, -100, -125, -26, -280, 20]  #0.349, -1.7453, -2.1816, -0.4537, -4.8869, 0.349
        # self.home_joint_angle_original = np.deg2rad(self.home_joint_angle_original)

        self.home_joint_angle = self.home_joint_angle_original # ur3[0.3438, -1.7949, -2.1418, -0.4759, -4.8844, 0.349] # [20, -100, -125, -26, -280, 20]
        # self.home_joint_angle = np.deg2rad(self.home_joint_angle)

        self.step_to_draw = 0 # 0: draw SVG file, 1: draw Square Frame, 2: draw Signature
        
        self.frame_pose_goals_list = [] # List of goal pose of a frame
        self.signature_pose_goals_list = [] # List of goal pose of Signature

    except rospy.ROSException as e:
        print("Error initializing UR3_Movement:", str(e))


# ------------------ Get robot type
  def set_robot_type(self, robot_type):
    self.robot_type = robot_type
    self.init_home_pose()

  def init_home_pose(self):
    if self.robot_type == "ur3e":
      self.home_joint_angle_original = [-1.17545, -1.94233, -1.93044, -0.53136, -4.86835, 0.2972]
      self.home_joint_angle = self.home_joint_angle_original
    else: 
      self.home_joint_angle_original = [-2.7466, -1.94233, -1.93044, -0.53136, -4.86835, 0.2972]
      self.home_joint_angle = self.home_joint_angle_original

#------------------- Draw from Gcode file
  def start_drawing(self):
    print("Here are all coordinates \n", self.goal_pose_list)
    self.set_origin_pose()
    time.sleep(0.5)

    # Start the thread drawing
    self.check_goals_thread = threading.Thread(target=self._check_goal_pose_list)
    self.check_goals_thread.start()
    
  def set_pose_goals_list(self, pose_goal_positions):
    self.goal_pose_list.clear()
    self.goal_pose_list.append(pose_goal_positions)
    self.enable_check_event.set()
    print ("\nAll goals are imported !")

  def set_frame_pose_goals_list(self, frame_pose_goal_positions):
    self.frame_pose_goals_list.clear()
    self.frame_pose_goals_list.append(frame_pose_goal_positions)
    print ("\nFrame Ready !")

  def set_signature_pose_goals_list(self, signature_pose_goal_positions):
    self.signature_pose_goals_list.clear()
    self.signature_pose_goals_list.append(signature_pose_goal_positions)
    print ("\nSignature Ready !")

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
          time.sleep(0.12) # 0.45 simulation # 0.15
          count_goal += 1
          goal = self.goal_pose_list[0][0]

          self.start_time = time.time()

          self.start_movement(goal)
          # Get the first goal from the list
          print (count_goal, ".New Goal: ", goal)
          # Wait for completion
          self.completeGoal_flag.wait()
          print ("Finish check Goal")

          if self.timer_exceed.is_set(): # Robot cannot fully reach goal -> run that goal again
            print("\n----------\nRun Again Current Goal!!!\n----------\n")

            
          else: # Robot can reach goal -> Pass new goal
            self.goal_pose_list[0].pop(0)
            print ("Pop the firts goal")

          time.sleep(0.15) # 0.5 simulation # 0.15 real robot

        except:
          print("Finish all goals")
          self.homing_ur3()
          self.step_to_draw += 1
          
          if self.step_to_draw == 1:
            self.draw_frame()
          elif self.step_to_draw == 2:
            self.draw_signature()
          elif self.step_to_draw == 3:
            print("Enjoy Drawing !!!")
            self.step_to_draw = 0
            self.enable_check_event.clear()
          

      rate.sleep()


  def draw_frame(self):
    print("\nChanging Pen 2...")
    self.change2Pen2()
    time.sleep(3)
    print("\nWaiting to draw Frame ............")
    self.set_origin_pose()
    print("\nContinue drawing Frame............")
    self.goal_pose_list.clear()
    self.goal_pose_list.extend(copy.deepcopy(self.frame_pose_goals_list))
    time.sleep(0.2)
    self.enable_check_event.set()
  
  def draw_signature(self):
    self.change2Pen3()
    time.sleep(4)
    print("\nWaiting to draw Signature ............")
    self.set_origin_pose()
    print("\nContinue drawing Signature............")
    self.goal_pose_list.clear()
    self.goal_pose_list.extend(copy.deepcopy(self.signature_pose_goals_list))
    print("Self Goal Pose list\n", self.goal_pose_list)
    time.sleep(0.2)
    self.enable_check_event.set()



  def set_joint_constraint(self, w1, w2, w3): #### EXPERIMENTAL
    # Joint name list : ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    self.clear_joint_constraints()

    current_joint_state = self.move_group.get_current_state().joint_state

    joint1_constraint = moveit_msgs.msg.JointConstraint(
      joint_name = "shoulder_pan_joint",
      position = current_joint_state.position[0],
      tolerance_above = math.pi/2,
      tolerance_below = math.pi/2,
      weight = w1,
    )

    joint2_constraint = moveit_msgs.msg.JointConstraint(
      joint_name = "shoulder_lift_joint",
      position = current_joint_state.position[1],
      tolerance_above = math.pi/2,
      tolerance_below = math.pi/2,
      weight = w2,
    )

    joint3_constraint = moveit_msgs.msg.JointConstraint(
      joint_name = "elbow_joint",
      position = current_joint_state.position[2],
      tolerance_above = math.pi/2,
      tolerance_below = math.pi/2,
      weight = w3,
    )

    self.path_constraints.joint_constraints.extend([joint1_constraint, joint2_constraint, joint3_constraint])
    self.move_group.set_path_constraints(self.path_constraints)
    # print("Set path constraints in Set Joint constraint: \n", self.move_group.get_path_constraints())
    # print("\nSelf.path_constraints = \n", self.path_constraints)


  def set_position_constraint(self, value_x, value_y, value_z, weight = 1.0):
    self.clear_position_constraints()

    self.position_constraint.header.frame_id = self.planning_frame
    self.position_constraint.link_name = self.eef_link

    constraint_pose = geometry_msgs.msg.Pose() # change to get_current_pose().pose if neccessary
    constraint_pose.position.x = value_x # value = 0 if keep it fixed
    constraint_pose.position.y = value_y
    constraint_pose.position.z = value_z 

    self.position_constraint.constraint_region.primitive_poses.append(constraint_pose)
    self.position_constraint.weight = weight

    self.path_constraints.position_constraints.append(self.position_constraint)
    self.move_group.set_path_constraints(self.path_constraints)


  def set_orientation_constraint(self, tol_x, tol_y, tol_z, weight = 1.0):
    self.clear_orientation_constraints()

    self.orientation_constraint.header.frame_id = self.planning_frame
    self.orientation_constraint.link_name = self.eef_link

    self.start_pose = self.move_group.get_current_pose().pose
    self.orientation_constraint.orientation = self.start_pose.orientation # Keep the current orientation for the next goal
    self.orientation_constraint.absolute_x_axis_tolerance = tol_x # 0.00005
    self.orientation_constraint.absolute_y_axis_tolerance = tol_y # 0.00005
    self.orientation_constraint.absolute_z_axis_tolerance = tol_z # 0.00005
    self.orientation_constraint.weight =  weight # 1.0 is fully considered during motion planning

    self.path_constraints.orientation_constraints.append(self.orientation_constraint)
    self.move_group.set_path_constraints(self.path_constraints)

#-------------------  Movement threading
  def homing_ur3(self):
    self.clear_all_constrainst()
    self.start_movement(self.home_joint_angle)

  def start_movement(self, target_pose): # THIS FUNCTION IS USED FOR STARTING THE THREAD TO ROBOT GO TO PRESET GOAL
    # start_pose = self.move_group.get_current_pose().pose
    # self.start_orientation = start_pose.orientation
    
    self.target_pose = target_pose
    self.completeGoal_flag.clear() # Always clear Complete Goal Flag before moving to goal
    self.move_thread = threading.Thread(target=self._move_to_target) # Start and Stop this thread to run the robot
    self.move_thread.start()

  def continue_drawing(self):
    self.release_stop_event()
    if not self.move_thread or not self.move_thread.is_alive():
        self.move_thread = threading.Thread(target=self._move_to_target)
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

  def set_new_home(self):
    self.home_joint_angle = copy.deepcopy(self.move_group.get_current_joint_values())

  def reset_home(self):
    self.home_joint_angle = copy.deepcopy(self.home_joint_angle_original)


  def set_origin_pose(self):
    self.set_orientation_constraint(0.0005, 0.0005, 0.0005)
    self.target_pose = None
    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation
    print ("This is origin orientation: \n", self.start_orientation)


  def clear_joint_constraints(self):
    self.path_constraints.joint_constraints = []
    self.move_group.clear_path_constraints()

  def clear_orientation_constraints(self):
    # Clear orientation constraint after use
    self.path_constraints.orientation_constraints = []
    self.move_group.clear_path_constraints()

  def clear_position_constraints(self):
    # Clear position constraint after use
    self.path_constraints.position_constraints = []
    self.move_group.clear_path_constraints()


  def clear_all_constrainst(self):
    self.clear_joint_constraints()
    self.clear_orientation_constraints()
    self.clear_position_constraints()

  def clear_all_goals(self):
    self.goal_pose_list.clear()
    self.enable_check_event.clear()




  ###################################################################################
  def move_with_orientation_constraint(self, target_pose):     ## EXPERIMENTAL cannot work because cannot plan

    self.set_joint_constraint(0.7, 0.7, 0.7) # GOOD = 1 joint constraint (2)
    self.clear_orientation_constraints()
    waypoints = []
    time.sleep(0.1)
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    delta_goal_z = target_pose[2] - current_pose.position.z
    
    if delta_goal_z != 0.00: 
      wpose.position.z += delta_goal_z
      waypoints.append(copy.deepcopy(wpose))

    delta_goal_x = target_pose[0] - current_pose.position.x
    delta_goal_y = target_pose[1] - current_pose.position.y

    if delta_goal_x != 0.00:
      wpose.position.x += delta_goal_x
    
    if delta_goal_y != 0.00:
      wpose.position.y += delta_goal_y

    wpose.orientation.x = self.start_orientation.x
    wpose.orientation.y = self.start_orientation.y
    wpose.orientation.z = self.start_orientation.z
    wpose.orientation.w = self.start_orientation.w
    waypoints.append(copy.deepcopy(wpose))

    eef_step = self.eef_step_processing(delta_goal_x, delta_goal_y, delta_goal_z)
    
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    self.timestamp_processing(plan)

    if fraction >= 0.1:
      self.move_group.execute(plan, wait = True)
    else: print("----------------\nFailed to plan the trajectory!\n------------------")
    
    self.move_group.clear_path_constraints()
    del waypoints
    del wpose
    del plan


        
  def timestamp_processing(self, plan, epsilon=1e-6):
      """
      Adjusts the time_from_start of waypoints in a trajectory if the last two waypoints have the same time_from_start.
      
      Args:
          plan (RobotTrajectory): The trajectory to adjust.
          epsilon (float): A small value to add to the time_from_start if adjustment is needed.
      """
      if len(plan.joint_trajectory.points) >= 2:
          new_trajectory_points = [plan.joint_trajectory.points[0]]  # Initialize with the first point

          for i in range(1, len(plan.joint_trajectory.points)):
              current_point = plan.joint_trajectory.points[i]
              previous_point = plan.joint_trajectory.points[i - 1]

              current_timestamp = current_point.time_from_start.to_sec()
              previous_timestamp = previous_point.time_from_start.to_sec()

              if current_timestamp == previous_timestamp == 0.0:
                  # Skip adding the current point if both current and previous timestamps are 0
                  continue

              new_trajectory_points.append(current_point)

          # Update the trajectory points with the new list
          plan.joint_trajectory.points = new_trajectory_points


  def eef_step_processing(self, delta_goal_x, delta_goal_y, delta_goal_z): # For Small segments with more accuracy
    # distance < 20mm : eef_step = 0.0075
    # if eef_step > 0.02 cannot go
    if delta_goal_z > 0.005:
      eef_step = 0.025 # 0.012 
    else: # If Z is not moving
      distance = math.sqrt(delta_goal_x**2 + delta_goal_y**2)
      if distance <= 30/1000: eef_step = 0.0075 #latest 0.007 # 0.0082 # distance <= 20
      elif distance > 40/1000 and distance <= 80/1000: eef_step = 0.015 #0.01 # distance > 20
      else: eef_step = 0.02

    print("\nCurrent eef_step: \n", eef_step)
    return eef_step

#### from 0.008 to 0.015


########################################################
  def _check_goal_reached(self): 
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
          # print ("Current Position: \n", current_position)
          # print ("Target Pose: \n", self.target_pose)

          # Calculate elapsed time
          elapsed_time = time.time() - self.start_time
          print("Elapsed time = ", elapsed_time)
          if all_close(self.target_pose, current_position, 0.001): # REAL: 0.001 / SIM: 0.002
            self.target_pose = None
            self.completeGoal_flag.set()
            time.sleep(0.05)
            self.timer_exceed.clear()
            if self.move_thread.is_alive():
              self.move_thread.join()
              self.move_group.stop()
            print("\n----------------\nGoal Position is reached !!!\n------------------\n")
            del current_position
            time.sleep(0.05)
          
          elif elapsed_time > 5:
              self.target_pose = None
              self.completeGoal_flag.set()
              time.sleep(0.05)
              self.timer_exceed.set()
              if self.move_thread.is_alive():
                self.move_thread.join()
                self.move_group.stop()
              print("\n----------------\nTimer Exceeds !!!\n------------------\n")
              del current_position
              time.sleep(0.5)
            


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


  #########################################################################
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

        elif len(target_pose) == 3: # ----- Gcode X,Y,Z
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
    self.clear_all_constrainst()

    current_joint = copy.deepcopy(self.move_group.get_current_joint_values())
    print("\nCurrent Joint = \n", current_joint)

    # Access to the last joint angle
    current_joint[-1] += math.radians(90)

    if current_joint[-1] >= 2*math.pi:
      print("\nThe TCP joint angle exceeds Joint Limit ! Cannot move !")
      current_joint[-1] -= math.radians(90)

    self.start_movement(current_joint)


  def change2rightpen(self):
    self.clear_all_constrainst()

    current_joint = copy.deepcopy(self.move_group.get_current_joint_values())
    print("\nCurrent Joint = \n", current_joint)

    # Access to the last joint angle
    current_joint[-1] -= math.radians(90)

    if current_joint[-1] <= -2*math.pi:
      print("\nThe TCP joint angle exceeds Joint Limit ! Cannot move !")
      current_joint[-1] += math.radians(90)

    self.start_movement(current_joint)


  def change2Pen1(self):
    self.clear_all_constrainst()
    current_joint = copy.deepcopy(self.home_joint_angle)
    current_joint[-1] = math.radians(20)

    self.start_movement(current_joint)


  def change2Pen2(self):
    self.clear_all_constrainst()
    current_joint = copy.deepcopy(self.home_joint_angle)
    current_joint[-1] = math.radians(110)

    self.start_movement(current_joint)

  
  def change2Pen3(self):
    self.clear_all_constrainst()
    current_joint = copy.deepcopy(self.home_joint_angle)
    current_joint[-1] = math.radians(-70)

    self.start_movement(current_joint)

# ------------------ Jogging movement
  def increase_x_tcp(self, value):
    self.set_position_constraint(value_x= value, value_y= 0.05, value_z= 0.05, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.05, tol_y= 0.05, tol_z= 0.05, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    wpose.position.x += value/1000
    waypoints.append(copy.deepcopy(wpose))

    if value > 10: eef_step = 0.01
    else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    time.sleep(0.2)
    if fraction >= 0.1:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose


  def decrease_x_tcp(self, value):
    self.set_position_constraint(value_x= value, value_y= 0.05, value_z= 0.05, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    wpose.position.x -= value/1000
    waypoints.append(copy.deepcopy(wpose))

    if value > 10: eef_step = 0.01
    else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    time.sleep(0.2)
    if fraction >= 0.1:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose


  def increase_y_tcp(self, value):
    self.set_position_constraint(value_x= 0.05, value_y= value, value_z= 0.05, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose

    wpose = copy.deepcopy(current_pose)

    wpose.position.y += value/1000
    waypoints.append(copy.deepcopy(wpose))

    if value > 10: eef_step = 0.01
    else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    time.sleep(0.2)
    if fraction >= 0.1:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose


  def decrease_y_tcp(self, value):
    self.set_position_constraint(value_x= 0.05, value_y= value, value_z= 0.05, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose

    wpose = copy.deepcopy(current_pose)

    wpose.position.y -= value/1000
    waypoints.append(copy.deepcopy(wpose))

    if value > 10: eef_step = 0.01
    else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    time.sleep(0.2)
    if fraction >= 0.1:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose


  def increase_z_tcp(self, value):
    self.set_position_constraint(value_x= 0.05, value_y= 0.05, value_z= value, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose

    wpose = copy.deepcopy(current_pose)

    wpose.position.z += value/1000
    waypoints.append(copy.deepcopy(wpose))

    if value > 10: eef_step = 0.01
    else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    time.sleep(0.2)
    if fraction >= 0.1:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose


  def decrease_z_tcp(self, value):
    self.set_position_constraint(value_x= 0.05, value_y= 0.05, value_z= value, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose

    wpose = copy.deepcopy(current_pose)

    wpose.position.z -= value/1000
    waypoints.append(copy.deepcopy(wpose))

    if value > 10: eef_step = 0.01
    else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    time.sleep(0.2)
    if fraction >= 0.1:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose












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


