#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, filedialog
from tkinter.ttk import *

from PIL import Image
import os
from rembg import remove
from PIL import Image
import svgpathtools as svg

from image_processor.image_processing import ImageProcessor
from ur3_control.move_ur3 import UR3_Movement

import threading
import time

import subprocess
import rospy

class SelfieDrawingApp:

    # Define global variables for screen width and height in ratio 4:3
    SCREEN_WIDTH = 320
    SCREEN_HEIGHT = 240

    def __init__(self, master: tk.Tk):
        super().__init__()
        self.process = None
        self.master = master

        # Initialize node for GUI updating
        rospy.init_node('selfie_drawing_ur3', anonymous= True)

        # GUI information
        master.title("Automated Artistic Portraiture")
        master.resizable(False, False)
        master.size

        # Get the user's home directory
        self.home_directory = os.path.expanduser("~")

        # Create a notebook (tabbed interface)
        self.notebook = ttk.Notebook(master)
        self.notebook.pack(fill='both', expand=True)

        # Create the tabs
        self.tab_take_picture = ttk.Frame(self.notebook)
        self.tab_robot_draw = ttk.Frame(self.notebook)

        # Add tabs to the notebook
        self.notebook.add(self.tab_take_picture, text="Take Photo & Processing")
        self.notebook.add(self.tab_robot_draw, text="Robot Draw")

        # X,Y,Z,Rx,Ry,Rz
        self.x_tcp = 0
        self.y_tcp = 0
        self.z_tcp = 0
        self.rx_tcp = 0
        self.ry_tcp = 0
        self.rz_tcp = 0

        # Define desired X and Y coordinate of a picture's center
        self.desire_x_pos = 0
        self.desire_y_pos = 0

        # Initialize Image Processor
        self.image_processor = ImageProcessor()

        # Initialize components for the "Take Picture" tab
        self.init_take_picture_tab()

        # Initialize components for the "Robot Draw" tab
        self.init_robot_draw_tab()
 
        # Initialize Icon for application
        self.init_icon()

        # Update GUI status
        
        self.update_GUI_status = threading.Thread(target= self._update_GUI_status)
        self.update_GUI_status.start()

    #-------------------- Init Icon App
    def init_icon(self):
        # Set the icon for the application window
        pic_path = os.path.join(self.home_directory, 'rs2_ws', 'icon', 'Designer.png')  # Specify the path to your icon file
        # Open the original image
        original_image = Image.open(pic_path)
        # Resize the image to 32x32 pixels (you can adjust the size as needed)
        resized_image = original_image.resize((100, 100))
        window_pic_path = os.path.join(self.home_directory, 'rs2_ws', 'icon', 'Designer_resized.png')  # Specify the path to your icon file
        # Save the resized image
        resized_image.save(window_pic_path)

        # Convert the image to RGB mode (if it's in indexed mode)
        pic_RGB = original_image.convert("RGB")
        resized_icon = pic_RGB.resize((32,32))
        icon_path = os.path.join(self.home_directory, 'rs2_ws', 'icon', 'icon.bmp')
        resized_icon.save(icon_path)

        if not os.path.exists(window_pic_path):
            print("Window Icon file not found at:", window_pic_path)
        elif not os.path.exists(icon_path):
            print("Icon file not found at:", icon_path)
        else:
            try:
                icon = tk.PhotoImage(file=window_pic_path)
                self.master.iconphoto(True, icon)
                self.master.iconbitmap(icon_path)
            except tk.TclError as e:
                print("Error setting icon:", e)

    #-------------------- Init 1st tab
    def init_take_picture_tab(self):
        # Define the desired height for the PREVIEW screens
        screen_width = self.SCREEN_WIDTH
        screen_height = self.SCREEN_HEIGHT


        #--------------------------------LIVE CAMERA
        # Create a frame to hold the preview screen and its label
        preview_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        preview_frame.grid(row=0, column=0, padx=10, pady=10)
        
        # Create a label for the preview screen with increased font size
        lbl_preview = tk.Label(preview_frame, text="Live Camera", font=("Arial", 20))
        lbl_preview.pack()

        # Create a canvas for the preview screen
        self.canvas_preview = tk.Canvas(preview_frame, width=screen_width, height=screen_height)
        self.canvas_preview.pack()


        #--------------------------------CAPTURE SCREEN
        # Create a frame to hold the capture screen and its label
        capture_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        capture_frame.grid(row=0, column=1, padx=10, pady=10)

        # Create a label for the capture screen with increased font size
        lbl_capture = tk.Label(capture_frame, text="Capture Image", font=("Arial", 20))
        lbl_capture.pack()

        # Create a canvas for the captured picture screen
        self.canvas_capture = tk.Canvas(capture_frame, width=screen_width, height=screen_height)
        self.canvas_capture.pack()


        #--------------------------------REMOVE BACKGROUND DISPLAY
        # Create a frame for "Removed Background Image" on the left
        removed_bg_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        removed_bg_frame.grid(row=1, column=0, padx=10, pady=10)

        # Create a label for "Removed Background Image"
        lbl_removed_bg = tk.Label(removed_bg_frame, text="Removed Background", font=("Arial", 20))
        lbl_removed_bg.pack()

        # Create a canvas for displaying the processed image
        self.canvas_processed_image = tk.Canvas(removed_bg_frame, width=screen_width, height=screen_height)
        self.canvas_processed_image.pack()
        
        
        #---------------------------------SVG OUTLINE DISPLAY
        # Create a frame for "Traced Outline Image" on the right
        traced_outline_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        traced_outline_frame.grid(row=1, column=1, padx=10, pady=10)

        # Create a label for "Traced Outline Image"
        lbl_traced_outline = tk.Label(traced_outline_frame, text="Trace Outline SVG", font=("Arial", 20))
        lbl_traced_outline.pack()

        # Create a canvas for displaying the traced outline image
        self.canvas_traced_outline_image = tk.Canvas(traced_outline_frame, width=screen_width, height=screen_height)
        self.canvas_traced_outline_image.pack()

        # Initialize traced outline image variable
        self.traced_outline_image = None


        #----------------------------------BUTTONS
        # Create a frame to hold the additional buttons
        countdown_frame = tk.Frame(self.tab_take_picture)
        countdown_frame.grid(row=0, column=2, padx=10, pady=10, sticky='ns')

        # Create a label for the countdown buttons
        lbl_countdown = tk.Label(countdown_frame, text="Countdown Timer", font=("Arial", 20, "bold"))
        lbl_countdown.grid(row=0, column=0, columnspan=4, pady=5)

        
        # Create buttons for 0, 3, 5, and 10 seconds
        self.countdown_value = 0

        self.btn_0_sec = tk.Button(countdown_frame, text="None", command=lambda: self.set_countdown(0), highlightthickness=1, highlightbackground="black")
        self.btn_3_sec = tk.Button(countdown_frame, text="3s", command=lambda: self.set_countdown(3), highlightthickness=0, highlightbackground="black")
        self.btn_5_sec = tk.Button(countdown_frame, text="5s", command=lambda: self.set_countdown(5), highlightthickness=0, highlightbackground="black")
        self.btn_10_sec = tk.Button(countdown_frame, text="10s", command=lambda: self.set_countdown(10), highlightthickness=0, highlightbackground="black")

        # Grid buttons in the countdown frame
        self.btn_0_sec.grid(row=1, column=0, padx=5, pady=0)
        self.btn_3_sec.grid(row=1, column=1, padx=5, pady=0)
        self.btn_5_sec.grid(row=1, column=2, padx=5, pady=0)
        self.btn_10_sec.grid(row=1, column=3, padx=5, pady=0)

        # Bind click events to button animations
        self.btn_0_sec.bind("<ButtonPress-1>", self.button_pressed)
        self.btn_3_sec.bind("<ButtonPress-1>", self.button_pressed)
        self.btn_5_sec.bind("<ButtonPress-1>", self.button_pressed)
        self.btn_10_sec.bind("<ButtonPress-1>", self.button_pressed)



        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.tab_take_picture)
        button_frame.grid(row=1, column=2, padx=10, pady=10, sticky='ns')  # Adjust row and column as needed

        # Create a label for the Function buttons
        lbl_function_buttons = tk.Label(button_frame, text="Function Buttons", font=("Arial", 20, "bold"))
        lbl_function_buttons.grid(row=0, column=0, pady=5)

        # Create a label for the countdown display
        self.lbl_countdown_display = tk.Label(countdown_frame, text="Countdown: 0s", font=("Arial", 16))
        self.lbl_countdown_display.grid(row=4, column=0, columnspan=4, pady=5)

        # Create the buttons for taking a picture and resetting
        # self.btn_capture = tk.Button(button_frame, text="Take Picture", command=lambda: self.image_processor.take_picture(self.canvas_capture, screen_width, screen_height), width=15, height= 5)
        self.btn_capture = tk.Button(button_frame, text="Take Picture", width=15, height= 5)
        btn_process_img = tk.Button(button_frame, text="Process Image", command=lambda: self.process_img(), width=15, height= 5)
        btn_generate_gcode = tk.Button(button_frame, text="Generate Gcode", command=lambda: self.generate_gcode(), width=15, height= 5)

        self.btn_capture.grid(row=1, column=0, padx=10, pady=10, sticky='ew')  
        btn_process_img.grid(row=2, column=0, padx=10, pady=10, sticky='ew')  
        btn_generate_gcode.grid(row=3, column=0, padx=10, pady=10, sticky='ew') 

        # Bind click events to button animations
        self.btn_capture.bind("<ButtonPress-1>", self.button_pressed)


        #----------------------------------Initialize
        # Initialize captured photo variable
        self.photo = None

        # Start the webcam preview
        self.update_preview()

    #-------------------- Init 2nd tab
    def init_robot_draw_tab(self):
        # Create a frame for the name
        name_frame = tk.Frame(self.tab_robot_draw)
        name_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=10, ipadx=5, ipady=5, sticky="w")

        # Create a label for "Name"
        lbl_name = tk.Label(name_frame, text="Name", font=("Arial", 20))
        lbl_name.grid(row=0, column=0, sticky="w")

        # Create a frame to hold the robot name label and the status label
        status_frame = tk.Frame(name_frame)
        status_frame.grid(row=0, column=1, sticky="w")

        # Create a label for the robot name
        robot_name = tk.Label(status_frame, text="UR3 Robot Arm", font=("Arial", 20, "bold"), fg= "blue")
        robot_name.pack(side=tk.LEFT, padx=(20, 40))  # Adjust padx as needed

        # Create a frame for "Robot Status"
        robot_status_frame = tk.Frame(status_frame)
        robot_status_frame.pack(side=tk.LEFT)

        # Create a label for "Robot Status"
        lbl_robot_status = tk.Label(robot_status_frame, text="Status", font=("Arial", 20))
        lbl_robot_status.pack(side=tk.LEFT)

        # Create a label to indicate the status (UPDATE REAL-TIME)
        self.robot_status_label = tk.Label(robot_status_frame, text="Disconnected", font=("Arial", 16), bg="red", fg="white")
        self.robot_status_label.pack(side=tk.LEFT, padx=20, ipadx=10)

        # Create a frame for "IP Address"
        ip_frame = tk.Frame(name_frame)
        ip_frame.grid(row=1, column=0, columnspan=2, ipadx=5, ipady=10, sticky="ew")

        # Create a label for "IP Address"
        lbl_ip = tk.Label(ip_frame, text="IP Address", font=("Arial", 20))
        lbl_ip.pack(side=tk.LEFT)

        # Create an entry for IP address input
        self.ip_entry = tk.Entry(ip_frame, font=("Arial", 16), width=15)
        self.ip_entry.pack(side=tk.LEFT, padx=10)

        # Set the initial value of the entry widget
        self.ip_entry.insert(0, "192.168.0.250")

        # Create a button to connect
        self.connect_button = tk.Button(ip_frame, text="Connect", font=("Arial", 16), command= lambda: self.connect_to_robot())
        self.connect_button.pack(side=tk.LEFT)


        #-------------------------------------------- TCP display

        # Create a frame for "UR3 TCP" on the right
        ur3_tcp_frame = tk.Frame(self.tab_robot_draw, bd=2, relief=tk.SOLID)
        ur3_tcp_frame.grid(row=0, column=2, rowspan=2, padx=10, pady=10, ipadx=5, ipady=5, sticky="w")

        # Create a label for "UR3 TCP"
        lbl_ur3_tcp = tk.Label(ur3_tcp_frame, text="UR3 TCP", font=("Arial", 20))
        lbl_ur3_tcp.pack()

        # Create a frame to stack labels vertically for coordinates
        coordinates_frame = tk.Frame(ur3_tcp_frame)
        coordinates_frame.pack()

        # Create labels to display the coordinates
        self.lbl_x = tk.Label(coordinates_frame, text="X:{:.2f} mm".format(self.x_tcp), font=("Arial", 16))
        self.lbl_y = tk.Label(coordinates_frame, text="Y:{:.2f} mm".format(self.y_tcp), font=("Arial", 16))
        self.lbl_z = tk.Label(coordinates_frame, text="Z:{:.2f} mm".format(self.z_tcp), font=("Arial", 16))
        self.lbl_rx = tk.Label(coordinates_frame, text="Rx:{:.2f} rad".format(self.rx_tcp), font=("Arial", 16))
        self.lbl_ry = tk.Label(coordinates_frame, text="Ry:{:.2f} rad".format(self.ry_tcp), font=("Arial", 16))
        self.lbl_rz = tk.Label(coordinates_frame, text="Rz:{:.2f} rad".format(self.rz_tcp), font=("Arial", 16))

        self.lbl_x.grid(row=0, column=0, padx=5)
        self.lbl_y.grid(row=1, column=0, padx=5)
        self.lbl_z.grid(row=2, column=0, padx=5)
        self.lbl_rx.grid(row=0, column=1, padx=5)
        self.lbl_ry.grid(row=1, column=1, padx=5)
        self.lbl_rz.grid(row=2, column=1, padx=5)


        # ------------------------------------------ File Selection
        open_file_frame = tk.Frame(self.tab_robot_draw, relief=tk.SOLID)
        open_file_frame.grid(row=1, column=2, rowspan=2, padx=10, pady=60, ipadx=5, ipady=5, sticky="w")

        open_file_button = tk.Button(open_file_frame, text="Open Gcode file", font=("Arial", 20), command= lambda: self.open_file_dialog())
        open_file_button.grid(row=0, column=0)

        import_gcode_button = tk.Button(open_file_frame, text= "Import Gcode", font=("Arial", 20), command= lambda: self.import_gcode())
        import_gcode_button.grid(row=1, column=0, pady= 5)

        #------------------------------------------- Connection Selection
        # Create a frame for connection selection
        connection_frame = tk.Frame(self.tab_robot_draw)
        connection_frame.grid(row=1, column=0, columnspan=1, sticky="w", padx= 10)

        # Create a label for connection selection
        lbl_connection = tk.Label(connection_frame, text="Connection Type:", font=("Arial", 20))
        lbl_connection.grid(row=0, column=0, sticky="w")

        # Variable to hold the selected connection type
        self.connection_type = tk.StringVar()

        # Set default connection type to "Real"
        self.connection_type.set("real")

        # Create radio buttons for real and simulation connections
        real_radio = tk.Radiobutton(connection_frame, text="Real", font=("Arial", 16), variable=self.connection_type, value="real")
        real_radio.grid(row=0, column=1, padx=10)

        simulation_radio = tk.Radiobutton(connection_frame, text="Simulation", font=("Arial", 16), variable=self.connection_type, value="simulation")
        simulation_radio.grid(row=0, column=2, padx=10)

        #------------------------------------------- Terminate button
        terminate_frame = tk.Frame(self.tab_robot_draw)
        terminate_frame.grid(row=2, column=0, columnspan=1, sticky="w", pady= 10)
        terminate_button = tk.Button(terminate_frame,text="Terminate All", font=("Arial", 16), command= lambda: self.terminate_process(), width=52)
        terminate_button.pack(side=tk.LEFT, padx=10)

        #------------------------------------------- Launch Simulation and Initialize Robot
        # Create a frame for the buttons
        buttons_frame = tk.Frame(self.tab_robot_draw)
        buttons_frame.grid(row=3, column=0, columnspan=1, sticky="w")

        # Create a button to launch UR3 Gazebo
        self.launch_gazebo_button = tk.Button(buttons_frame, text="Launch Gazebo", font=("Arial", 16), command= lambda: self.launch_gazebo())
        self.launch_gazebo_button.pack(side=tk.LEFT, padx=10)

        # Create a button to launch MoveIt Planning
        launch_moveit_planning_button = tk.Button(buttons_frame, text="Launch MoveIt Planning", font=("Arial", 16), command= lambda: self.launch_moveit_planning())
        launch_moveit_planning_button.pack(side=tk.LEFT, padx=10)

        # Create a button to launch MoveIt
        launch_moveit_button = tk.Button(buttons_frame, text="Launch MoveIt", font=("Arial", 16), command= lambda: self.launch_moveit())
        launch_moveit_button.pack(side=tk.LEFT, padx=10)


        # Create a frame for Robot initialize
        initRobot_buttons_frame = tk.Frame(self.tab_robot_draw)
        initRobot_buttons_frame.grid(row=4, column=0, columnspan=1, sticky="w", pady= 10)

        # Create a button for "Init"
        initRobot_button = tk.Button(initRobot_buttons_frame, text="Initialize Robot UR3", font=("Arial", 16), width= 52, command= lambda: self.init_robot_ur3())
        initRobot_button.pack(side=tk.LEFT, padx=10)
        

        #--------------------------------------------- Homing
        # Create a frame for Robot Homing
        set_button_frame = tk.Frame(self.tab_robot_draw)
        set_button_frame.grid(row=5, column=0, columnspan=1, sticky="w", pady= 10)

        # Create a button for "Init"
        homing_button = tk.Button(set_button_frame, text="Homing", font=("Arial", 16), width= 5, command= lambda: self.homing_ur3())
        homing_button.pack(side=tk.LEFT, padx=10)

        # Create a button for "Set New Home"
        set_new_home_button = tk.Button(set_button_frame, text= "New Home", font=("Arial", 16), width= 10, command= lambda: self.set_new_home_ur3())
        set_new_home_button.pack(side=tk.LEFT, padx=5)

        # Create button for "Reset Home"
        reset_home_button = tk.Button(set_button_frame, text= "Reset Home", font=("Arial", 16), width= 10, command= lambda: self.reset_home_ur3() )
        reset_home_button.pack(side= tk.LEFT, padx=5)

        # Create a button for "Set Zero Position" / "Set Origin"
        set_origin_button = tk.Button(set_button_frame, text= "Offset Picture", font=("Arial", 16), width= 10, command= lambda: self.offset_picture())
        set_origin_button.pack(side=tk.LEFT, padx=5)


        # # Create a button for Importing file Gcode
        # import_file_gcode_button = tk.Button(set_button_frame, text= "Import Gcode", font=("Arial", 16), width= 10, command= lambda: self.import_gcode())
        # import_file_gcode_button.pack(side=tk.LEFT, padx=5)

        #--------------------------------------------- Draw Buttons
        # Create a frame for drawing buttons
        additional_buttons_frame = tk.Frame(self.tab_robot_draw)
        additional_buttons_frame.grid(row=6, column=0, columnspan=1, sticky="w")

        # Create a button for "Draw!"
        draw_button = tk.Button(additional_buttons_frame, text="Draw!", font=("Arial", 16), command= lambda: self.start_drawing())
        draw_button.pack(side=tk.LEFT, padx=10)

        # Create a button for "Stop"
        stop_button = tk.Button(additional_buttons_frame, text="Stop", font=("Arial", 16), command=lambda: self.stop_drawing())
        stop_button.pack(side=tk.LEFT, padx=10)

        # Create a button for "Run Test"
        run_test_button = tk.Button(additional_buttons_frame, text="Release Stop", font=("Arial", 16), command=lambda: self.release_stop())
        run_test_button.pack(side=tk.LEFT, padx=10)

        # Create a button for "Print Pose"
        print_pose_button = tk.Button(additional_buttons_frame, text="Print Pose", font=("Arial", 16), command= lambda: self.print_ur3_pose())
        print_pose_button.pack(side=tk.LEFT, padx=10)

        # Create a button for "Clear Goal"
        clear_goals_button = tk.Button(additional_buttons_frame, text= "Clear Goals", font=("Arial", 16), command= lambda: self.clear_all_goals())
        clear_goals_button.pack(side= tk.LEFT, padx=10)


        #------------------------------------------------ Button Changing Pen
        # Create a frame for Changing Pen
        changing_pen_frame = tk.Frame(self.tab_robot_draw)
        changing_pen_frame.grid(row= 7, column=0, columnspan=1, sticky= "w", pady= 10)

        # Create a button for Changing pen left / right
        left_pen = tk.Button(changing_pen_frame, text="Left Pen", font=("Arial", 16), command= lambda: self.change2leftpen())
        left_pen.pack(side=tk.LEFT, padx= 10)

        right_pen = tk.Button(changing_pen_frame, text= "Right Pen", font=("Arial", 16), command= lambda: self.change2rightpen())
        right_pen.pack(side=tk.LEFT, padx= 10)

        pen_1 = tk.Button(changing_pen_frame, text="Pen 1", font=("Arial", 16), command= lambda: self.selectPen1())
        pen_1.pack(side=tk.LEFT, padx= 10)

        pen_2 = tk.Button(changing_pen_frame, text="Pen 2", font=("Arial", 16), command= lambda: self.selectPen2())
        pen_2.pack(side=tk.LEFT, padx= 10)

        pen_3 = tk.Button(changing_pen_frame, text="Pen 3", font=("Arial", 16), command= lambda: self.selectPen3())
        pen_3.pack(side=tk.LEFT, padx= 10)

        #--------------------------------------------- Button Moving X+ Y+ X- Y- TCP

        moving_tcp_frame = tk.Frame(self.tab_robot_draw)
        moving_tcp_frame.grid(row= 8, column= 0, columnspan=1, sticky= "w", pady= 5)

        increase_x = tk.Button(moving_tcp_frame, text= "X+", font=("Arial", 16), command= lambda: self.increase_x_movement())
        increase_x.pack(side= tk.LEFT, padx= 10)

        decrease_x = tk.Button(moving_tcp_frame, text= "X-", font=("Arial", 16), command= lambda: self.decrease_x_movement())
        decrease_x.pack(side= tk.LEFT, padx= 10)

        increase_y = tk.Button(moving_tcp_frame, text= "Y+", font=("Arial", 16), command= lambda: self.increase_y_movement())
        increase_y.pack(side= tk.LEFT, padx= 10)

        decrease_y = tk.Button(moving_tcp_frame, text= "Y-", font=("Arial", 16), command= lambda: self.decrease_y_movement())
        decrease_y.pack(side= tk.LEFT, padx= 10)

        # Lable for unit input 
        lbl_unit_mm = tk.Label(moving_tcp_frame, text= "Unit in mm", font =("Arial", 20))
        lbl_unit_mm.pack(side=tk.LEFT)

        # Create an entry for unit mm input
        self.unit_mm_entry = tk.Entry(moving_tcp_frame, font=("Arial", 16), width=15)
        self.unit_mm_entry.pack(side=tk.LEFT, padx=10)
        self.unit_mm_entry.insert(0,"5")




    #-------------------- Button for Jogging X,Y
    def increase_x_movement(self):
        value = float(self.unit_mm_entry.get())
        self.ur3_operate.increase_x_tcp(value)

    def decrease_x_movement(self):
        value = float(self.unit_mm_entry.get())
        self.ur3_operate.decrease_x_tcp(value)

    def increase_y_movement(self):
        value = float(self.unit_mm_entry.get())
        self.ur3_operate.increase_y_tcp(value)

    def decrease_y_movement(self):
        value = float(self.unit_mm_entry.get())
        self.ur3_operate.decrease_y_tcp(value)



    #-------------------- Button for Changing Pen
    def change2leftpen(self):
        self.ur3_operate.change2leftpen()

    def change2rightpen(self):
        self.ur3_operate.change2rightpen()

    def selectPen1(self):
        self.ur3_operate.change2Pen1()

    def selectPen2(self):
        self.ur3_operate.change2Pen2()

    def selectPen3(self):
        self.ur3_operate.change2Pen3()



    #-------------------- Button for File selection
    def open_file_dialog(self):
        relative_path = "rs2_ws/gcode"
        initial_directory = os.path.join(self.home_directory, relative_path)
        self.gcode_path = filedialog.askopenfilename(title="Select Gcode file", initialdir= initial_directory, filetypes=[("Gcode files", "*.gcode")])
        if self.gcode_path:
            print("Selected Gcode file:", self.gcode_path)
            # self.import_gcode()
        else:
            print("No file selected.")

    def import_gcode(self): # should include after opening file
        if self.gcode_path:
            # Perform import operations using the selected Gcode file path
            print("Open Gcode file at: ", self.gcode_path)
            offset_gcode_path = self.offset_gcode(self.gcode_path, self.desire_x_pos, self.desire_y_pos)
            pose_goal_positions = self.gcode2pose(offset_gcode_path)
            self.ur3_operate.set_pose_goals_list(pose_goal_positions)
        else:
            print("No Gcode file selected.")
    #-------------------- Buttons for Robot
    def connect_to_robot(self):
        # Get the IP address from the entry widget
        robot_ip = self.ip_entry.get()

        # Construct the command to execute
        command = ["roslaunch", "ur_robot_driver", "ur3_bringup.launch", f"robot_ip:={robot_ip}"]

        # Execute the command
        self.process = subprocess.Popen(command)

    def terminate_process(self):
        # Terminate the process if it exists
        if self.process:
            self.process.terminate()
            self.process.wait()  # Wait for the process to terminate completely
            self.process = None

        # Initialize ROS if it hasn't been initialized yet
        if not rospy.core.is_initialized():
            rospy.init_node('process_terminator_node', anonymous=True)

        # Use subprocess to execute rosnode list command
        process = subprocess.Popen(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = process.communicate()

        # Check if there was any error
        if process.returncode != 0:
            print("Error executing rosnode list command:", error.decode())
            return

        # Split the output into individual node names
        node_names = output.decode().split()

        # Print the list of node names for debugging
        print("List of ROS nodes:", node_names)

        # Iterate through the node names and kill each node
        for node_name in node_names:
            node_process = subprocess.Popen(['rosnode', 'kill', node_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            _, node_error = node_process.communicate()
            if node_process.returncode != 0:
                print(f"Error killing node {node_name}: {node_error.decode()}")

        # Kill Gazebo processes
        gazebo_process = subprocess.run(['pkill', '-f', 'gazebo'])
        if gazebo_process.returncode != 0:
            print("Error killing Gazebo processes")

        # Kill all remaining ROS nodes
        all_nodes_process = subprocess.run(['rosnode', 'kill', '-a'])
        if all_nodes_process.returncode != 0:
            print("Error killing all ROS nodes")

        rospy.signal_shutdown('Shutting down node running')

    def launch_gazebo(self):
        # Construct the command to execute
        command = ['roslaunch', 'ur_gazebo', 'ur3_bringup.launch']
        # Execute the command
        self.process = subprocess.Popen(command) 
    
    def launch_moveit_planning(self):
        if self.connection_type.get() == "real": command = ["roslaunch", "ur3_moveit_config", "moveit_planning_execution.launch"]
        else: command = ["roslaunch", "ur3_moveit_config", "moveit_planning_execution.launch", "sim:=true"]
        
        # Execute the command
        self.process = subprocess.Popen(command) 

    def launch_moveit(self):
        # Construct the command to execute
        command = ["roslaunch", "ur3_moveit_config", "moveit_rviz.launch"]
        # Execute the command
        self.process = subprocess.Popen(command) 

    def init_robot_ur3(self):
        # Initialize UR3
        self.ur3_operate = UR3_Movement()

        # Start get thread Robot TCP
        self.ur3_operate.update_robot_tcp_thread()
        time.sleep(0.2)
        # Start update thread Robot TCP
        self.init_update_tcp_thread()
    
    def homing_ur3(self):
        # Homing robot with specific joint state
        self.ur3_operate.homing_ur3()

    def set_new_home_ur3(self):
        self.ur3_operate.set_new_home()

    def reset_home_ur3(self):
        self.ur3_operate.reset_home()

    def offset_picture(self):
        tcp_pose_x,tcp_pose_y,tcp_pose_z,tcp_ori_x,tcp_ori_y,tcp_ori_z = self.ur3_operate.get_tcp()
        self.desire_x_pos = tcp_pose_x
        self.desire_y_pos = tcp_pose_y
        print("\nSet Desired X,Y pos: [",self.desire_x_pos, ", ", self.desire_y_pos, "]")

        self.ur3_operate.set_origin_pose()
        print ("\nSet origin pose")
 

    

    def start_drawing(self):
        self.ur3_operate.start_drawing()

    def stop_drawing(self):
        self.ur3_operate.stop_movement()

    def release_stop(self):
        self.ur3_operate.release_stop_event()

    def print_ur3_pose(self):
        print("\nCurrent pose:", self.ur3_operate.move_group.get_current_pose().pose)
        print("\nCurrent Joint: ", self.ur3_operate.move_group.get_current_joint_values())

    def clear_all_goals(self):
        print("\n----------------\nAll Goals are clear !!!\n------------------\n")
        self.ur3_operate.clear_all_goals()




    def gcode2pose(self, new_gcode_path):
        # Read the Gcode file and extract the pose goal positions
        # gcode_file_path = os.path.join(self.home_directory, "rs2_ws", "gcode", "ur3_draw_offset.gcode")
        gcode_file_path = new_gcode_path
        pose_goal_positions = []

        with open (gcode_file_path, 'r') as file:
            for line in file:
                if line.startswith('G0'):
                    # Extract X,Y coordinates from the gcode file
                    x = float(line.split('X')[1].split(' ')[0])
                    y = float(line.split('Y')[1].split(' ')[0])
                    z = 0.16
                    pose_goal_positions.append([x,y,z])

                elif line.startswith('G1'):
                    # Extract X,Y coordinates from the gcode file
                    x = float(line.split('X')[1].split(' ')[0])
                    y = float(line.split('Y')[1].split(' ')[0])
                    z = 0.125
                    pose_goal_positions.append([x,y,z])

        return pose_goal_positions

    #------------------- Update TCP of UR3 Threading
    def init_update_tcp_thread(self):
        self.update_gui_thread = threading.Thread(target= self._update_tcp_gui)
        self.update_gui_thread.start()

    def _update_tcp_gui(self):
        rate = rospy.Rate(10)
        while True:
            try:
                tcp_pose_x, tcp_pose_y, tcp_pose_z, tcp_ori_x, tcp_ori_y, tcp_ori_z = self.ur3_operate.get_tcp()
            except:
                print("TCP not ready!")

            self.x_tcp = tcp_pose_x
            self.y_tcp = tcp_pose_y
            self.z_tcp = tcp_pose_z
            self.x_ori = tcp_ori_x
            self.y_ori = tcp_ori_y
            self.z_ori = tcp_ori_z

            # Update TCP coordinates in the GUI labels
            self.lbl_x.config(text="X:{:.2f} mm".format(self.x_tcp))
            self.lbl_y.config(text="Y:{:.2f} mm".format(self.y_tcp))
            self.lbl_z.config(text="Z:{:.2f} mm".format(self.z_tcp))
            self.lbl_rx.config(text="Rx:{:.2f} rad".format(self.x_ori))
            self.lbl_ry.config(text="Ry:{:.2f} rad".format(self.y_ori))
            self.lbl_rz.config(text="Rz:{:.2f} rad".format(self.z_ori))
            rate.sleep()




    #-------------------- Threading for Updating GUI status
    def _update_GUI_status(self):
        rate = rospy.Rate(10)
        while True:
            # Update button status based on Connection type
            if self.connection_type.get() == "real":
                self.launch_gazebo_button.configure(state= tk.DISABLED)
                self.ip_entry.configure(state= tk.NORMAL)
                self.connect_button.configure(state= tk.NORMAL)

            elif self.connection_type.get() == "simulation":
                self.launch_gazebo_button.configure(state= tk.NORMAL)
                self.ip_entry.configure(state= tk.DISABLED)
                self.connect_button.configure(state= tk.DISABLED)
        
            rate.sleep()


        

    #-------------------- Threading for Timer

    def init_countdown(self, duration, callback):
        self.duration = duration
        self.callback = callback # self.take_picture()
        self.timer_thread = None
        self.running = False
    
    def start_countdown(self):
        self.running = True
        self.timer_thread = threading.Thread(target=self._run_timer)
        self.timer_thread.start()

    def stop(self):
        self.running = False

    def _run_timer(self):
        # count = self.duration
        count = self.countdown_value
        while count >= 0 and self.running:
            print(f"Countdown now: {count}")
            self.lbl_countdown_display.config(text=f"Countdown: {count}s")
            time.sleep(1)  # Sleep for 1 second
            count -= 1

        if self.running:
            if self.callback:
                self.callback()
                self.stop()

    def set_countdown(self,seconds):
        self.countdown_value = seconds
        self.update_countdown_display()

    def update_countdown_display(self):
        self.lbl_countdown_display.config(text=f"Countdown: {self.countdown_value}s")


    #-------------------- Buttons for Image Processing
    def button_pressed(self, event):
        event.widget.config(highlightthickness=1)

        if event.widget == self.btn_0_sec:
            self.btn_3_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_5_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_10_sec.config(relief=tk.RAISED, highlightthickness=0)

        elif event.widget == self.btn_3_sec:
            self.btn_0_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_5_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_10_sec.config(relief=tk.RAISED, highlightthickness=0)

        elif event.widget == self.btn_5_sec:
            self.btn_0_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_3_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_10_sec.config(relief=tk.RAISED, highlightthickness=0)
            
        elif event.widget == self.btn_10_sec:
            self.btn_0_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_3_sec.config(relief=tk.RAISED, highlightthickness=0)
            self.btn_5_sec.config(relief=tk.RAISED, highlightthickness=0)

        elif event.widget == self.btn_capture:
            if (self.countdown_value == 0): self.take_picture()
            else: 
                self.init_countdown(duration= self.countdown_value, callback= self.take_picture)
                self.start_countdown()
             
    def take_picture(self):
        self.image_processor.take_picture(self.canvas_capture, self.SCREEN_WIDTH, self.SCREEN_HEIGHT)

    def update_preview(self):
        self.image_processor.update_preview(self.canvas_preview, self.SCREEN_WIDTH, self.SCREEN_HEIGHT)

    def process_img(self):
        self.image_processor.process_img(self.canvas_processed_image, self.canvas_traced_outline_image)

    #--------------------- Buttons for Gcode processing

    def generate_gcode(self): # convert SVG file to Gcode
        # Check if the svg file exists
        svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")
        if not os.path.exists(svg_path):
            print("SVG file not found.")
            return

        # Load SVG file
        paths, _ = svg.svg2paths(svg_path)

        # Calculate SVG center
        svg_center_x, svg_center_y = self.calculate_svg_center(paths)

        # Desired center point (center coordinate)
        desired_center_x, desired_center_y = 0, 0  # Modify as needed

        # Calculate displacement
        displacement_x = desired_center_x - svg_center_x
        displacement_y = desired_center_y - svg_center_y

        # Scale factor (adjust as needed)
        scale = 0.001  # Experiment with this value

        save_folder_gcode = os.path.join(self.home_directory, "rs2_ws", "gcode")
        
        # Ensure the save folder exists, create it if it doesn't
        if not os.path.exists(save_folder_gcode):
            os.makedirs(save_folder_gcode)

        gcode_path = os.path.join(self.home_directory, "rs2_ws", "gcode", "ur3_draw.gcode")

        # Open G-code file
        with open(gcode_path, 'w') as f:
            for path in paths:
                for i, segment in enumerate(path):
                    # Extract segment information
                    start = segment.start
                    end = segment.end

                    # Convert coordinates to G-code coordinates
                    start_x, start_y = (start.real + displacement_x) * scale, (start.imag + displacement_y) * scale
                    end_x, end_y = (end.real + displacement_x) * scale, (end.imag + displacement_y) * scale

                    # Adjust Y coordinate to match G-code coordinate system (mirror along Y-axis)
                    start_y = -start_y
                    end_y = -end_y

                    # Write G-code commands
                    if i == 0:
                        f.write(f"G0 X{start_x:.10f} Y{start_y:.10f}\n")  # Rapid move to start point
                    f.write(f"G1 X{end_x:.10f} Y{end_y:.10f}\n")  # Linear move to end point


        print("Generate Gcode Done!")
        
    def calculate_svg_center(self,paths):
        # Calculate the bounding box of all paths
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')
        for path in paths:
            for segment in path:
                start = segment.start
                end = segment.end
                min_x = min(min_x, start.real, end.real)
                max_x = max(max_x, start.real, end.real)
                min_y = min(min_y, start.imag, end.imag)
                max_y = max(max_y, start.imag, end.imag)

        # Calculate the center point
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        return center_x, center_y
    
    def offset_gcode(self, gcode_path, offset_x, offset_y):

        new_gcode_path = os.path.splitext(gcode_path)[0] + "_offset.gcode"
        with open(gcode_path, 'r') as file:
            with open(new_gcode_path, 'w') as new_file:
                for line in file:
                    if line.startswith('G0') or line.startswith('G1'):
                        # Extract X and Y coordinates
                        parts = line.split()
                        x_coord = None
                        y_coord = None
                        for part in parts:
                            if part.startswith('X'):
                                x_coord = float(part[1:])
                            elif part.startswith('Y'):
                                y_coord = float(part[1:])
                        if x_coord is not None and y_coord is not None:
                            # Apply offset
                            x_coord += offset_x * 1.08 # Adjust this parameter to modify the frame inside working area
                            y_coord += offset_y
                            # Write modified line to new file
                            new_line = f"{parts[0]} X{x_coord:.6f} Y{y_coord:.6f}\n"
                            new_file.write(new_line)
                    else:
                        # Write non-coordinate lines unchanged
                        new_file.write(line)

        return new_gcode_path


# def main():
#     root = tk.Tk()
#     SelfieDrawingApp(root)
#     root.mainloop()

# if __name__ == "__main__":
#     main()