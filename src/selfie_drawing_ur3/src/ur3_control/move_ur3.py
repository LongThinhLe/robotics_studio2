#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from moveit_commander.conversions import pose_to_list
import numpy as np

import threading 

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

        self.stop_flag = threading.Event()  # Event to signal stop
        self.stop_flag.clear()
        self.pause_flag = threading.Event() 
        self.pause_flag.clear()

    except rospy.ROSException as e:
        print("Error initializing UR3_Movement:", str(e))


  def init_joint_state(self):

    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_deg = [20, -100, -125, -26, -280, 20]
    joint_goal = np.deg2rad(joint_deg)
    print ("Start threading inside init joint state")

    move_group.go(joint_goal, wait=True)

    move_group.stop()
    
    # Reset the stop flag after homing
    self.stop_flag.clear()
    self.pause_flag.clear()

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  

  def init_pose(self):

    move_group = self.move_group

    angle_rad = math.radians(10)

    ## Learn deeply at: https://eater.net/quaternions/video/intro   
    pose_goal = geometry_msgs.msg.Pose() 
    pose_goal.orientation.x = 0.0 
    pose_goal.orientation.y = math.cos(angle_rad)
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = math.sin(angle_rad)
    pose_goal.position.x = 0.35
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.12

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
  

  def list_pose_goal(self):
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose() 
    pose_goal_positions = [
        [0.35, 0.1, 0.1],  
        [0.35, (0.1-(297/1000)/2), 0.1],
        [0.35+210/2000, (0.1-(297/1000)/2), 0.1],
        [0.35+210/2000, (0.1-(297/1000)/2) + 297/1000, 0.1],
        [0.35, (0.1-(297/1000)/2) + 297/1000, 0.1],
        [0.35, 0.1, 0.1],
        [0.35, 0.1, 0.105],
    ]

    angle_rad = math.radians(10)
    pose_goal.orientation.x = 0.0 
    pose_goal.orientation.y = math.cos(angle_rad)
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = math.sin(angle_rad)


    for position in pose_goal_positions:
      pose_goal.position.x = position[0]
      pose_goal.position.y = position[1]
      pose_goal.position.z = position[2]

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()


  def gcode_to_pose_goal(self):
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal_positions = []

    gcode_commands = [
        "G1 X0.35 Y0.1 Z0.1",
        "G1 X0.35 Y-0.0485 Z0.1",
        "G1 X0.455 Y-0.0485 Z0.1",
        "G1 X0.455 Y0.2485 Z0.1",
        "G1 X0.35 Y0.2485 Z0.1",
        "G1 X0.35 Y0.1 Z0.1",
        "G1 X0.35 Y0.1 Z0.105"
        # Add more GCode commands here as needed
    ]

    for command in gcode_commands:
        # Parse GCode command to extract movement information
        if command.startswith('G1'):
            # Example assumption: G1 commands are used for linear movement
            # Extract X, Y, and Z coordinates from the command
            x = float(command.split('X')[1].split(' ')[0])
            y = float(command.split('Y')[1].split(' ')[0])
            z = float(command.split('Z')[1].split(' ')[0])

            # print ("\nx = ", x)
            # print ("\ny = ", y)
            # print ("\nz = ", z)
            # Update current position
            current_position = [x, y, z]

            # Append current position to pose_goal_positions
            pose_goal_positions.append(current_position)
    print (pose_goal_positions)

    angle_rad = math.radians(10)
    pose_goal.orientation.x = 0.0 
    pose_goal.orientation.y = math.cos(angle_rad)
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = math.sin(angle_rad)
       

    for position in pose_goal_positions:
      pose_goal.position.x = position[0]
      pose_goal.position.y = position[1]
      pose_goal.position.z = position[2]

      move_group.set_pose_target(pose_goal)
      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      print("\nPose Information:", self.move_group.get_current_pose().pose)


#-------------------  Movement threading
  def homing_ur3(self):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_deg = [20, -100, -125, -26, -280, 20]
    joint_goal = np.deg2rad(joint_deg)

    self.start_movement(joint_goal)


  def set_pose(self):
    angle_rad = math.radians(10)

    pose_goal = geometry_msgs.msg.Pose() 
    pose_goal.orientation.x = 0.0 
    pose_goal.orientation.y = math.cos(angle_rad)
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = math.sin(angle_rad)
    pose_goal.position.x = 0.35
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.12

    self.pause_flag.clear()
    self.start_movement(pose_goal)


  def start_movement(self,target_pose):
    self.target_pose = target_pose
    self.move_thread = threading.Thread(target=self._move_to_target)
    self.move_thread.start()

  def stop_movement(self):
    self.stop_flag.set()
    if self.move_thread:
        self.move_thread.join()
        self.move_group.stop()

  def pause_movement(self):
    self.pause_flag.set()
    if self.move_thread:
        self.move_thread.join()
        self.move_group.stop()

  def release_stop_event(self):
    # Reset the stop flag after homing
    self.stop_flag.clear()
    self.pause_flag.clear()

  def _move_to_target(self):
    rate = rospy.Rate(10)  # Adjust the rate based on your requirements
    move_group = self.move_group
    while not self.stop_flag.is_set() and not self.pause_flag.is_set():
      if type(self.target_pose) is geometry_msgs.msg.Pose: # Moving with Pose
        if self.target_pose:
          move_group.set_pose_target(self.target_pose)
          plan = move_group.go(wait=False)  # Set wait=False to execute asynchronously

          # Check if the movement plan was successfully executed
          if plan:
            move_group.stop()  # Stop any ongoing movement
            self.target_pose = None  # Reset target pose after movement

      else: # Moving with Joint angles
        if np.all(self.target_pose):
          move_group.set_joint_value_target(self.target_pose)
          plan = move_group.go(wait=False) 

          if plan:
            move_group.stop()
            self.target_pose = None

      rate.sleep()  # Ensure that the loop runs at a specific rate


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






# def main():
#   try:
#     print("")
#     print("----------------------------------------------------------")
#     print("MoveIt GCode Control UR3 Robot")
#     print("----------------------------------------------------------")
#     print("Press Ctrl-D to exit at any time")
#     print("")
#     # input()
#     ur3_movement = UR3_Movement()

#     ur3_movement.init_joint_state()



#     # ur3_movement.init_pose()
    
#     # ur3_movement.gcode_to_pose_goal()
#     # while (True):
#         # ur3_movement.list_pose_goal()
#         # ur3_movement.gcode_to_pose_goal()
#         # continue
#     print("===== Python tutorial demo complete! =====")


#   except rospy.ROSInterruptException:
#     return
#   except KeyboardInterrupt:
#     return

# if __name__ == '__main__':
#   main()