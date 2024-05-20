#------------------- CHECKING THREADING

  def _check_goal_pose_list(self): # NOW CHECK FOR MULTIPLE GOAL
    """
    Check if there is a goal pose/joint in self.goal_pose_list.
    If a goal exists, set it as self.target_pose for robot movement.
    If the goal is a pose, convert it to a numpy array before setting.
    """
    rate = rospy.Rate(10)
    print ("Go into the thread check goal")
    count_goal = 0
    while self.enable_check_event.is_set():
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
          
          elif elapsed_time > 3:
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


#---------------------------------- MOVE TO TARGET
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


#----------------------------------- MOVING FUNCTION THREADING
  def move_with_orientation_constraint(self, target_pose):   

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