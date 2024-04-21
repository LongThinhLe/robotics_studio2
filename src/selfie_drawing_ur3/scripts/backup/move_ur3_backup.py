
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




# NO USE
   # while True:
    #   # while self._is_running.is_set():
    #   if self._is_running.is_set():
    #     print("Robot is running...")
    #     if self.target_pose is not None:
    #       print("WE are having target pose")
    #       if np.all(self.target_pose): # Joint value
    #         self.move_group.set_joint_value_target(self.target_pose)
    #         plan = self.move_group.go(wait=False)

    #         # current_joints = self.move_group.get_current_joint_values()
    #         # if all_close(self.target_pose, current_joints, 0.01):
    #         #   self.move_group.stop()
    #         #   self.target_pose = None

    #         if plan:
    #           print ("delete target pose in plan 1111111")
    #           self.move_group.stop()
    #           self.target_pose = None

    #       else:
    #         self.move_group.set_pose_target(self.target_pose)
    #         plan = self.move_group.go(wait=False)
    #         print("Go here 222222222222222222222222222")

    #         # current_poses = self.move_group.get_current_pose().pose
    #         # if all_close(self.target_pose, current_poses, 0.01):
    #         #   self.move_group.stop()
    #         #   self.target_pose = None

    #         if plan:
    #           print ("delete target pose in plan 2222222")
    #           self.move_group.stop()
    #           self.target_pose = None

    #     # rate.sleep()
      
    #   # self.move_group.stop()
    #   # print("Robot stops")
    #   rate.sleep()



      # if self.target_pose is not None: # If there is a goal pose in Target_pose then run
        
      #   if isinstance(self.target_pose, geometry_msgs.msg.Pose):  # Moving with Pose
      #     if self.target_pose:
      #       move_group.set_pose_target(self.target_pose)
      #       plan = move_group.go(wait=False)  # Set wait=False to execute asynchronously
            
      #       # Check if the movement plan was successfully executed
      #       if plan:
      #         move_group.stop()  # Stop any ongoing movement
      #         self.target_pose = None  # Reset target pose after movement


      #   else: # Moving with Joint angles (np.array)
      #     if np.all(self.target_pose):
      #       move_group.set_joint_value_target(self.target_pose)
      #       plan = move_group.go(wait=False) 
            
      #       if plan:
      #         move_group.stop()
      #         self.target_pose = None



      # rate.sleep()  # Ensure that the loop runs at a specific rate INSIDE THE LOOP WHILE


  # def pause_movement(self): # Stop the robot and stop the thread (join)
  #   self.pause_flag.set()
  #   self._is_running.clear()
  #   if self.move_thread:
  #       self.move_thread.join()
  #       self.move_group.stop()



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