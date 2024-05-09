  def move_with_orientation_constraint(self, target_pose): # problem
#--------------------------------------------- Current Use
    
    self.set_orientation_constraint(tol_x= 0.0005, tol_y= 0.0005, tol_z= 0.0005, weight= 1.0)

    waypoints = []
    time.sleep(0.5)
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    delta_goal_z = target_pose[2] - current_pose.position.z
    
    if delta_goal_z != 0.0: 
      wpose.position.z += delta_goal_z
      waypoints.append(copy.deepcopy(wpose))

    delta_goal_x = target_pose[0] - current_pose.position.x
    delta_goal_y = target_pose[1] - current_pose.position.y

    if delta_goal_x != 0.0:
      wpose.position.x += delta_goal_x
      self.set_position_constraint(value_x= wpose.position.x, value_y= 0.0, value_z= 0.0, weight= 0.95)
    
    if delta_goal_y != 0.0:
      wpose.position.y += delta_goal_y
      self.set_position_constraint(value_x= 0.0, value_y= wpose.position.y, value_z= 0.0, weight= 0.95)

    waypoints.append(copy.deepcopy(wpose))
    
    eef_step = 0.001
    # if (abs(delta_goal_x) < 0.05 and abs(delta_goal_x) != 0.0) or (abs(delta_goal_y) < 0.05 and abs(delta_goal_y) != 0.0):
    #   eef_step = 0.0001
    # else: eef_step = 0.01

    # if abs(delta_goal_x) < 0.01 or abs(delta_goal_y) < 0.01: eef_step = 0.0001
    # else: eef_step = 0.01

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0, path_constraints= self.path_constraints)

    if fraction >= 0.2:
      self.move_group.execute(plan, wait = True)
      self.move_group.stop()  
    else: print("----------------\nFailed to plan the trajectory!\n------------------")
    
    del waypoints
    del wpose

    #------------------------------------

    # waypoints = []
    # current_pose = self.move_group.get_current_pose().pose
    # wpose = copy.deepcopy(current_pose)

    # wpose.position.z = target_pose[2]
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x = target_pose[0]
    # wpose.position.y = target_pose[1]
    # wpose.orientation.x = self.start_orientation.x
    # wpose.orientation.y = self.start_orientation.y
    # wpose.orientation.z = self.start_orientation.z
    # wpose.orientation.w = self.start_orientation.w
    # waypoints.append(copy.deepcopy(wpose))

    # offset_x = wpose.position.x - current_pose.position.x
    # offset_y = wpose.position.y - current_pose.position.y

    # if offset_x > 0.01 and offset_y > 0.01: eef_step = 0.01
    # else: eef_step = 0.0001

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= eef_step, jump_threshold= 0.0)
    # if fraction >= 0.2:
    #   self.move_group.execute(plan, wait = False)
    #   self.move_group.stop()  
    # else: print("----------------\nFailed to plan the trajectory!\n------------------")
    
    # del waypoints
    # del wpose 



def set_origin_pose(self):
    # # Set orientation constraint
    # self.orientation_constraint.header.frame_id = self.move_group.get_planning_frame()
    # self.orientation_constraint.link_name = self.move_group.get_end_effector_link()
    # self.start_pose = self.move_group.get_current_pose().pose
    # self.orientation_constraint.orientation = self.start_pose.orientation # Keep the current orientation for the next goal
    # self.orientation_constraint.absolute_x_axis_tolerance = 0.05
    # self.orientation_constraint.absolute_y_axis_tolerance = 0.05
    # self.orientation_constraint.absolute_z_axis_tolerance = 0.05
    # self.orientation_constraint.weight = 1.0 # 1.0 is fully considered during motion planning

    # # Create path constraints
    # self.path_constraints.orientation_constraints.append(self.orientation_constraint)
    # self.move_group.set_path_constraints(self.path_constraints)
  
    self.set_orientation_constraint()
    self.target_pose = None
    start_pose = self.move_group.get_current_pose().pose
    self.start_orientation = start_pose.orientation
    print ("This is origin orientation: \n", self.start_orientation)  

def increase_y_tcp(self, value):
    waypoints = []

    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.x = self.start_orientation.x
    wpose.orientation.y = self.start_orientation.y
    wpose.orientation.z = self.start_orientation.z
    wpose.orientation.w = self.start_orientation.w
    wpose.position.y += value/1000
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,0.01,0.0)
    self.move_group.execute(plan, wait=True)
    self.move_group.stop()

    del waypoints
    del wpose



def decrease_x_tcp(self, value):
    self.set_position_constraint(value_x= value, value_y= 0, value_z= 0, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    wpose.position.x -= value/1000
    waypoints.append(copy.deepcopy(wpose))

    print("Self Path Constraint: \n", self.path_constraints)
    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= 0.0001, jump_threshold= 0.0, path_constraints= self.path_constraints)
    if fraction >= 0.2:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose

    # self.set_orientation_constraint() # add

    # constraints = Constraints()
    # position_constraint = PositionConstraint()

    # position_constraint.header.frame_id = self.planning_frame
    # position_constraint.link_name = self.eef_link

    # constraint_pose = geometry_msgs.msg.Pose()
    # constraint_pose.position.x = value 
    # constraint_pose.position.y = 0 # no constraint on Y
    # constraint_pose.position.z = 0 # no constraint on Z
    # position_constraint.constraint_region.primitive_poses.append(constraint_pose)

    # position_constraint.weight = 1.0

    # constraints.position_constraints.append(position_constraint)

    # self.move_group.set_path_constraints(constraints)

    # waypoints = []
    # current_pose = self.move_group.get_current_pose().pose
    # wpose = copy.deepcopy(current_pose)

    # # wpose.orientation.x = self.start_orientation.x
    # # wpose.orientation.y = self.start_orientation.y
    # # wpose.orientation.z = self.start_orientation.z
    # # wpose.orientation.w = self.start_orientation.w
    # wpose.position.x -= value/1000
    # # wpose.position.y = current_pose.position.y
    # # wpose.position.z = current_pose.position.z
    
    # waypoints.append(copy.deepcopy(wpose))

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= 0.01, jump_threshold= 0.0, path_constraints= constraints)
    # if fraction >= 0.2:
    #   self.move_group.execute(plan, wait = False)
    #   self.move_group.stop()  

    # self.clear_position_constraints()
    # self.clear_orientation_constraints() # add
    # del waypoints
    # del wpose


    # # waypoints = []

    # # wpose = self.move_group.get_current_pose().pose
    # # wpose.orientation.x = self.start_orientation.x
    # # wpose.orientation.y = self.start_orientation.y
    # # wpose.orientation.z = self.start_orientation.z
    # # wpose.orientation.w = self.start_orientation.w
    # # wpose.position.x -= value/1000
    # # waypoints.append(copy.deepcopy(wpose))

    # # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints,0.01,0.0)
    # # self.move_group.execute(plan, wait=True)
    # # self.move_group.stop()

    # # del waypoints
    # # del wpose
  

def increase_x_tcp(self, value):
    self.set_position_constraint(value_x= value, value_y= 0, value_z= 0, weight= 1.0)
    self.set_orientation_constraint(tol_x= 0.005, tol_y= 0.005, tol_z= 0.000005, weight= 1.0)
    
    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    wpose = copy.deepcopy(current_pose)

    wpose.position.x += value/1000
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= 0.0001, jump_threshold= 0.0, path_constraints= self.path_constraints)
    if fraction >= 0.2:
      self.move_group.execute(plan, wait = False)
      self.move_group.stop()  

    del waypoints
    del wpose


    #---------------------------------------

    # self.set_orientation_constraint() ##

    # constraints = Constraints()
    # position_constraint = PositionConstraint()

    # position_constraint.header.frame_id = self.planning_frame
    # position_constraint.link_name = self.eef_link

    # constraint_pose = geometry_msgs.msg.Pose()
    # constraint_pose.position.x = value 
    # constraint_pose.position.y = 0 # no constraint on Y
    # constraint_pose.position.z = 0 # no constraint on Z
    # position_constraint.constraint_region.primitive_poses.append(constraint_pose)

    # position_constraint.weight = 1.0

    # constraints.position_constraints.append(position_constraint)

    # self.move_group.set_path_constraints(constraints)

    # waypoints = []
    # current_pose = self.move_group.get_current_pose().pose
    # wpose = copy.deepcopy(current_pose)

    # # wpose.orientation.x = self.start_orientation.x
    # # wpose.orientation.y = self.start_orientation.y
    # # wpose.orientation.z = self.start_orientation.z
    # # wpose.orientation.w = self.start_orientation.w
    # wpose.position.x += value/1000
    # # wpose.position.y = current_pose.position.y
    # # wpose.position.z = current_pose.position.z
    
    # waypoints.append(copy.deepcopy(wpose))

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= 0.01, jump_threshold= 0.0, path_constraints= constraints)
    # if fraction >= 0.2:
    #   self.move_group.execute(plan, wait = False)
    #   self.move_group.stop()  

    # self.clear_position_constraints()
    # self.clear_orientation_constraints() ##
    # del waypoints
    # del wpose





def move_with_orientation_constraint(self, target_pose, scale = 1):
    # start_orientation_list = [self.start_orientation.x, self.start_orientation.y, self.start_orientation.z, self.start_orientation.w]
    # self.move_group.set_orientation_target(start_orientation_list)
    # plan = self.move_group.go(wait= True)
    # self.move_group.stop()
    # self.move_group.clear_pose_targets()

    waypoints = []
    current_pose = self.move_group.get_current_pose().pose
    current_x = current_pose.position.x
    current_y = current_pose.position.y
    current_z = current_pose.position.z

    offset_x = target_pose[0] - current_x
    offset_y = target_pose[1] - current_y
    offset_z = target_pose[2] - current_z

    wpose = copy.deepcopy(current_pose) # deepcopy to not to change the original value

    wpose.position.z += scale * offset_z
    # wpose.orientation.z += current_pose.orientation.z - self.start_orientation.z
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * offset_x
    wpose.position.y += scale * offset_y
    # wpose.orientation.x += current_pose.orientation.x - self.start_orientation.x
    # wpose.orientation.y += current_pose.orientation.y - self.start_orientation.y

    waypoints.append(copy.deepcopy(wpose))
    
    # current_pose = self.move_group.get_current_pose().pose
    
    # goal_pose = geometry_msgs.msg.Pose()
    # goal_pose.position.x = target_pose[0]
    # goal_pose.position.y = target_pose[1]
    # goal_pose.position.z = target_pose[2]
    # goal_pose.orientation = self.start_orientation # Maintain same orientation

    # # Compute Cartesian path from current pose to goal pose
    # waypoints = [current_pose, goal_pose]

    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints= waypoints, eef_step= 0.001, jump_threshold= 0.0, path_constraints= self.path_constraints)

    
    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
    

    if fraction == 1.0:
      self.move_group.execute(plan, wait = True)
      # # Ensure unique timestamps for waypoints
      # for i in range(len(plan.joint_trajectory.points)):
      #     plan.joint_trajectory.points[i].time_from_start = rospy.Duration(i * 0.0001)  # Example timestamp increment
      # self.move_group.execute(plan)

      # self.move_group.stop()
    else: print ("----------------\nFailed to plan the trajectory!\n------------------")

    del waypoints
    del wpose



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


    # #----------- THIS FUNCTION TO CHECK IF THE GOAL IS REACH WITH CARTESIAN COMPUTE PATH
    # print("The first goal is = ", self.goal_pose_list[0][0])
    # self.target_pose = self.goal_pose_list[0][0]


#-------------------------------------- TESTING (should inside the loop) KEEP THIS -> This function to run robot
    # waypoints = []

    # current_pose = self.move_group.get_current_pose().pose
    # current_x = current_pose.position.x
    # current_y = current_pose.position.y
    # current_z = current_pose.position.z

    # goal_x = -0.37
    # goal_y = 0.02
    # goal_z = 0.105

    # offset_x = goal_x - current_x
    # offset_y = goal_y - current_y
    # offset_z = goal_z - current_z

    # wpose = copy.deepcopy(current_pose)



    # wpose.position.x += scale * offset_x
    # wpose.position.y += scale * offset_y
    # waypoints.append(copy.deepcopy(wpose))

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
    # self.move_group.execute(plan, wait = False)




#------------------------------ SET POSITION TARGET FUNCTION -> Wrong
    # target_position = [-0.33, 0.02, 0.18]
    # self.move_group.set_position_target(target_position)

    # self.move_group.go(wait = False)

    # self.move_group.stop()

    # self.move_group.clear_pose_targets()


#--------------------------------------------------

    # waypoints = []

    # wpose = self.move_group.get_current_pose().pose
    # list_length = len(self.goal_pose_list[0])


    # for i in range (min(10, list_length)):
    #   wpose.position.z = scale * self.goal_pose_list[0][i][2]
    #   waypoints.append(copy.deepcopy(wpose))

    #   wpose.position.x = scale * self.goal_pose_list[0][i][0]
    #   wpose.position.y = scale * self.goal_pose_list[0][i][1]
    #   waypoints.append(copy.deepcopy(wpose))

    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
    # self.move_group.execute(plan, wait = False)

    # Still use minus offset !!!

#--------------------------------------------------
    # waypoints = []

    # current_pose = self.move_group.get_current_pose().pose
    # current_x = current_pose.position.x
    # current_y = current_pose.position.y
    # current_z = current_pose.position.z

    # # print ("Current Z = ", current_z)
    # # print ("Next Z = ", self.goal_pose_list[0][0][2])

    # offset_x = self.goal_pose_list[0][0][0] - current_x
    # offset_y = self.goal_pose_list[0][0][1] - current_y
    # offset_z = self.goal_pose_list[0][0][2] - current_z

    # print ("Current X = ", current_x)
    # print ("Current Y = ", current_y)

    # print ("Offset x = ", offset_x)
    # print ("Offset y = ", offset_y)
    # # print ("Offset z = ", offset_z)


    # wpose = copy.deepcopy(current_pose)
    # # Update z position
    # wpose.position.z += scale * offset_z
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x += scale * offset_x
    # wpose.position.y += scale * offset_y
    # waypoints.append(copy.deepcopy(wpose))


    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
    # self.move_group.execute(plan, wait = False)
    # print ("Waypoint position z = \n", waypoints)





    #---------------------------------------- Convert Gcode to waypoint is wrong !!!!

    # list_length = len(self.goal_pose_list[0]) 
    # # for i in range (min(50, list_length)):
    # for i in range (list_length-1):
    #     current_x = self.goal_pose_list[0][i][0]
    #     current_y = self.goal_pose_list[0][i][1]
    #     current_z = self.goal_pose_list[0][i][2]

    #     next_x = self.goal_pose_list[0][i+1][0]
    #     next_y = self.goal_pose_list[0][i+1][1]
    #     next_z = self.goal_pose_list[0][i+1][2]

    #     # Testing:
    #     offset_x = current_x
    #     offset_y = current_y


    #     # offset_x = next_x - current_x
    #     # offset_y = next_y - current_y
    #     offset_z = next_z - current_z

    #     print ("\n\nOffset x = ", offset_x)
    #     print ("Offset y = ", offset_y)
    #     print ("Offset z = ", offset_z)

    #     if offset_z > 0 or offset_z < 0:
    #       wpose.position.z += scale * offset_z
    #       waypoints.append(copy.deepcopy(wpose))

    #     wpose.position.x = scale * offset_x
    #     wpose.position.y = scale * offset_y
    #     waypoints.append(copy.deepcopy(wpose))

    # # print ("Waypoint position z = \n", waypoints)
    # (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.005, 0.0)
    # self.move_group.execute(plan, wait = False)

    #---------------------------------------- Convert Gcode to waypoint is wrong !!!!

#--------------------------------------------------

        
        


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