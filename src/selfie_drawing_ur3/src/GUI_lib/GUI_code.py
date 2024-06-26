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
from gcode_processor.gcode_processing import GcodeProcessing
from ur3_control.move_ur3 import UR3_Movement

import threading
import time

import subprocess
import rospy

class SelfieDrawingApp:

    # Define global variables for screen width and height in ratio 4:3
    SCREEN_WIDTH    = 450  # 320px
    SCREEN_HEIGHT   = 340 # 240px

    def __init__(self, master: tk.Tk):
        super().__init__()
        self.process = None
        self.master = master

        # Initialize node for GUI updating
        rospy.init_node('selfie_drawing_ur3', anonymous= True)

        # GUI information
        master.title("Selfie Robot")
        master.resizable(False, False)
        master.size

        # Get the user's home directory
        self.home_directory = os.path.expanduser("~")

        # Create a notebook (tabbed interface)
        self.notebook = ttk.Notebook(master)
        self.notebook.pack(fill='both', expand=True)

        # Create the tabs
        self.tab_take_picture = ttk.Frame(self.notebook)
        self.tab_easy_mode = ttk.Frame(self.notebook)
        self.tab_robot_draw = ttk.Frame(self.notebook)

        # Add tabs to the notebook
        self.notebook.add(self.tab_take_picture, text="1. Take Photo & Processing")
        self.notebook.add(self.tab_easy_mode, text="2. Draw")
        self.notebook.add(self.tab_robot_draw, text="3. Dev. Mode")

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

        # Initialize Gcode Processor
        self.gcode_processor = GcodeProcessing()

        # Initialize components for the "Photo" tab
        self.init_take_picture_tab()
        
        # Initialize components for the "Easy Draw" tab
        self.init_easy_tab()

        # Initialize components for the "Dev Draw" tab
        self.init_robot_draw_tab()
 
        # Initialize Icon for application
        # self.init_icon()

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
        live_cam_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        live_cam_frame.grid(row=0, column=0, padx=10, pady=10)

        # Create a label for the preview screen with increased font size
        lbl_live_cam = tk.Label(live_cam_frame, text="Live Camera", font=("Arial", 20, "bold"), fg="#c71010")
        lbl_live_cam.pack()

        # Create a canvas for the preview screen
        self.canvas_live_camera = tk.Canvas(live_cam_frame, width=screen_width, height=screen_height)
        self.canvas_live_camera.pack()

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

        self.btn_0_sec = tk.Button(countdown_frame, text="None", command=lambda: self.set_countdown(0), bg="#9289f5", fg="#FFFFFF", highlightthickness=1, highlightbackground="black", relief="raised", borderwidth=3)
        self.btn_3_sec = tk.Button(countdown_frame, text="3s", command=lambda: self.set_countdown(2), bg="#8079d1", fg="#FFFFFF", highlightthickness=0, highlightbackground="black", relief="raised", borderwidth=3)
        self.btn_5_sec = tk.Button(countdown_frame, text="5s", command=lambda: self.set_countdown(4), bg="#716bb5", fg="#FFFFFF", highlightthickness=0, highlightbackground="black", relief="raised", borderwidth=3)
        self.btn_10_sec = tk.Button(countdown_frame, text="10s", command=lambda: self.set_countdown(9), bg="#5e5996", fg="#FFFFFF", highlightthickness=0, highlightbackground="black", relief="raised", borderwidth=3)

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

        # Create a frame for the Name input and Draw button
        name_draw_frame = tk.Frame(self.tab_take_picture)
        name_draw_frame.grid(row=0, column=3, padx=10, pady=10, sticky='ns')

        # Create a label for the Name input
        lbl_name = tk.Label(name_draw_frame, text="Name & Hashtag", font=("Arial", 20, "bold"))
        lbl_name.grid(row=0, column=0, pady=5)

        # Create a textbox for Name input
        self.entry_name = tk.Entry(name_draw_frame, font=("Aharoni", 25), bd=2, relief=tk.SOLID, width= 16)
        self.entry_name.grid(row=1, column=0, padx=5, pady=5)

        self.entry_hashtag = tk.Entry(name_draw_frame, font= ("Aharoni", 25), bd=2, relief=tk.SOLID, width= 16)
        self.entry_hashtag.grid(row=2, column=0, padx=5, pady=5)

        # Create the Draw button
        self.btn_draw = tk.Button(name_draw_frame, text="Draw!", command=lambda: self.start_drawing_easy(), bg="grey", fg="#FFFFFF", width=15, height=5, relief="raised", borderwidth=3, highlightthickness=2, state=tk.DISABLED)
        self.btn_draw.grid(row=3, column=0, padx=10, pady=10)

        # Create the Stop drawing button
        self.btn_stop = tk.Button(name_draw_frame, text="Stop Drawing!", command=lambda: self.end_drawing(), bg="red", fg="#FFFFFF", width=15, height=5, relief="raised", borderwidth=3, highlightthickness=2,  state=tk.DISABLED)
        self.btn_stop.grid(row=4, column=0, padx=10, pady=10)

        # Add copy-right text under the "Stop Drawing!" button
        copy_right = tk.Label(name_draw_frame, text="made by\nLong Thinh Le\nDennis Nguyen\nStewart Kelly\nSamuel Bloomfield", font=("Arial", 15))
        copy_right.grid(row=5, column=0, pady=10)


        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.tab_take_picture)
        button_frame.grid(row=1, column=2, padx=10, pady=10, sticky='ns')  # Adjust row and column as needed

        # Create a label for the Function buttons
        lbl_function_buttons = tk.Label(button_frame, text="Menu", font=("Arial", 20, "bold"))
        lbl_function_buttons.grid(row=0, column=0, pady=5)

        # Create a label for the countdown display
        self.lbl_countdown_display = tk.Label(countdown_frame, text="Countdown:\n0s", font=("Arial", 24), fg= "red")
        self.lbl_countdown_display.grid(row=4, column=0, columnspan=4, pady=5)

        # Create the buttons for taking a picture and resetting
        self.btn_capture = tk.Button(button_frame, text="1. Take Selfie", width=15, height=5, relief="raised", bg="#13d12f", fg="#FFFFFF", borderwidth=3, highlightthickness=2)
        btn_process_img = tk.Button(button_frame, text="2. Process Image", command=lambda: self.process_img(), bg="#0fbd29", fg="#FFFFFF", width=15, height=5, relief="raised", borderwidth=3, highlightthickness=2)
        btn_generate_gcode = tk.Button(button_frame, text="3. Confirm", command=lambda: self.generate_gcode(), bg="#0e9c23", fg="#FFFFFF", width=15, height=5, relief="raised", borderwidth=3, highlightthickness=2)

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




    #-------------------- Init Easy Mode
    def init_easy_tab(self):
            # Create a frame for the name
            name_frame = tk.Frame(self.tab_easy_mode)
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
            robot_status_frame_easy = tk.Frame(status_frame)
            robot_status_frame_easy.pack(side=tk.LEFT)

            # Create a label for "Robot Status"
            lbl_robot_status_easy = tk.Label(robot_status_frame_easy, text="Connection:", font=("Arial", 20))
            lbl_robot_status_easy.pack(side=tk.LEFT)

            # Create a label to indicate the status (UPDATE REAL-TIME)
            self.robot_status_label_easy = tk.Label(robot_status_frame_easy, text="Disconnected", font=("Arial", 16), bg="red", fg="white")
            self.robot_status_label_easy.pack(side=tk.LEFT, padx=20, ipadx=10)

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
            self.ip_entry.insert(0, "192.168.1.102") # 192.168.0.250 # ur3e 192.168.1.102 150.22.0.250

            # Create a button to connect
            self.connect_button_easy = tk.Button(ip_frame, text="1. Connect", font=("Arial", 16), command= lambda: self.connect_to_robot_easy(), relief="raised", borderwidth=3, highlightthickness=2)
            self.connect_button_easy.pack(side=tk.LEFT)

            # Create a frame for Robot initialize
            initRobot_buttons_frame = tk.Frame(self.tab_easy_mode)
            initRobot_buttons_frame.grid(row=3, column=0, columnspan=1, sticky="w", pady= 10)

            # Create a button for "Init"
            initRobot_button = tk.Button(initRobot_buttons_frame, text="2. Initialize UR3 Robot", font=("Arial", 16), bg="#FFA500", fg="#FFFFFF", width= 56, command= lambda: self.init_robot_ur3_easy(), relief="raised", borderwidth=3, highlightthickness=2)
            initRobot_button.pack(side=tk.LEFT, padx=10)

            #--------------------------------------------- Draw Buttons
            # Create a frame for drawing buttons
            additional_buttons_frame = tk.Frame(self.tab_easy_mode)
            additional_buttons_frame.grid(row=4, column=0, columnspan=1, sticky="w", pady= 10)

            # Create a button for "Draw!"
            draw_button = tk.Button(additional_buttons_frame, text="3. Draw!", font=("Arial", 16), bg="#008000", fg="#FFFFFF", width=56, command= lambda: self.start_drawing_easy(), relief="raised", borderwidth=3, highlightthickness=2)
            draw_button.pack(side=tk.LEFT, padx=(10, 30))

            #--------------------------------------------- Stopping Buttons
            # Create a frame for Stopping buttons
            stopping_buttons_frame = tk.Frame(self.tab_easy_mode)
            stopping_buttons_frame.grid(row=5, column=0, columnspan=1, sticky="w", pady= 10)
            
            # Create a button for "Stop"
            stop_button = tk.Button(stopping_buttons_frame, text="Stop", font=("Arial", 16), bg="#ff0000", fg="#FFFFFF", width=25, command=lambda: self.stop_drawing(), relief="raised", borderwidth=3, highlightthickness=2)
            stop_button.pack(side=tk.LEFT, padx=(10, 30))

            # Create a button for "Continue"
            run_test_button = tk.Button(stopping_buttons_frame, text="Continue", font=("Arial", 16), bg="#0000FF", fg="#FFFFFF", width=25, command=lambda: self.continue_drawing(), relief="raised", borderwidth=3, highlightthickness=2)
            run_test_button.pack(side=tk.LEFT, padx=(10))
            
            #------------------------------------------- End drawing button
            # Create a frame for end drawing button
            terminate_frame = tk.Frame(self.tab_easy_mode)
            terminate_frame.grid(row=6, column=0, columnspan=1, sticky="w", pady= 10)
            # Create a button for termination
            terminate_button = tk.Button(terminate_frame,text="4. End Drawing", font=("Arial", 16), bg="#000000", fg="#FFFFFF", command= lambda: self.end_drawing(), width=56, relief="raised", borderwidth=3, highlightthickness=2)
            terminate_button.pack(side=tk.LEFT, padx=10)
            
            # Create frame for copy-right text.
            cr_frame = tk.Frame(self.tab_easy_mode)
            cr_frame.grid(row=10, column=2, padx=10, pady=10, sticky='ns')
            
            # Add copy-right text.
            copy_right = tk.Label(cr_frame, text="made by\nLong Thinh Le\nDennis Nguyen\nStewart Kelly\nSamuel Bloomfield", font=("Arial", 15))
            copy_right.grid(row=0, column=0)
            

    #-------------------- Init Dev Tab
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
        self.ip_entry.insert(0, "192.168.1.102") # 192.168.0.250 # ur3e 192.168.1.102 150.22.0.250

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
        set_origin_button = tk.Button(set_button_frame, text= "Set Origin", font=("Arial", 16), width= 10, command= lambda: self.set_origin_ur3())
        set_origin_button.pack(side=tk.LEFT, padx=5)


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
        
        # Create frame for copy-right text.
        cr_frame = tk.Frame(self.tab_robot_draw)
        cr_frame.grid(row=10, column=2, padx=10, pady=10, sticky='ns')
        
        # Add copy-right text.
        copy_right = tk.Label(cr_frame, text="made by\nLong Thinh Le\nDennis Nguyen\nStewart Kelly\nSamuel Bloomfield", font=("Arial", 15))
        copy_right.grid(row=0, column=0)

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

        increase_z = tk.Button(moving_tcp_frame, text= "Z+", font=("Arial", 16), command= lambda: self.increase_z_movement())
        increase_z.pack(side= tk.LEFT, padx= 10)

        decrease_z = tk.Button(moving_tcp_frame, text= "Z-", font=("Arial", 16), command= lambda: self.decrease_z_movement())
        decrease_z.pack(side= tk.LEFT, padx= 10)
        

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

    def increase_z_movement(self):
        value = float(self.unit_mm_entry.get())
        self.ur3_operate.increase_z_tcp(value)

    def decrease_z_movement(self):
        value = float(self.unit_mm_entry.get())
        self.ur3_operate.decrease_z_tcp(value)



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
            
            # Import Pose Goal position of Frame
            # frame_gcode_path = os.path.join(self.home_directory, 'rs2_ws', 'gcode', 'frame_square_150mm.gcode')
            frame_gcode_path = os.path.join(self.home_directory, 'rs2_ws', 'gcode', 'username.gcode')
            offset_frame_gcode_path = self.offset_gcode(frame_gcode_path, self.desire_x_pos, self.desire_y_pos)
            frame_pose_goal_positions = self.gcode2pose(offset_frame_gcode_path)
            self.ur3_operate.set_frame_pose_goals_list(frame_pose_goal_positions)

            # Import Pose Goal position of Signature
            signature_gcode_path = os.path.join(self.home_directory, 'rs2_ws', 'gcode', 'hashtag.gcode')
            offset_signature_gcode_path = self.offset_gcode(signature_gcode_path, self.desire_x_pos, self.desire_y_pos)
            signature_pose_goal_positions = self.gcode2pose(offset_signature_gcode_path)
            self.ur3_operate.set_signature_pose_goals_list(signature_pose_goal_positions)

        else:
            print("No Gcode file selected.")




    #-------------------- Buttons for Robot
    def connect_to_robot(self):
        # Get the IP address from the entry widget
        robot_ip = self.ip_entry.get()

        if robot_ip == "192.168.0.250":
            command = ["roslaunch", "ur_robot_driver", "ur3_bringup.launch", f"robot_ip:={robot_ip}"]
            self.robot_type = "ur3"
            
        elif robot_ip == "192.168.1.104":
            command = ["roslaunch", "ur_robot_driver", "ur3e_bringup.launch", f"robot_ip:={robot_ip}"]
            self.robot_type = "ur3e"

        else:
            command = ["roslaunch", "ur_robot_driver", "ur3e_bringup.launch", f"robot_ip:={robot_ip}"]
            self.robot_type = "ur3e"

        self.robot_status_label.config(text="Connected", bg="green")
        self.robot_status_label_easy.config(text="Connected", bg="green")

        self.btn_draw.configure(state=tk.NORMAL, bg= "green")
        self.btn_stop.configure(state=tk.NORMAL)


        # Execute the command
        self.process = subprocess.Popen(command)

    def connect_to_robot_easy(self):
        # Get the IP address from the entry widget
        robot_ip = self.ip_entry.get()
        self.robot_type = "ur3"
        
        if robot_ip == "192.168.0.250":
            command = ["roslaunch", "ur_robot_driver", "ur3_bringup.launch", f"robot_ip:={robot_ip}"]
            self.robot_type = "ur3"
        elif robot_ip == "192.168.1.102":
            command = ["roslaunch", "ur_robot_driver", "ur3e_bringup.launch", f"robot_ip:={robot_ip}"]
            self.robot_type = "ur3e"
        else:
            command = ["roslaunch", "ur_robot_driver", "ur3e_bringup.launch", f"robot_ip:={robot_ip}"]
            self.robot_type = "ur3e"

        self.robot_status_label.config(text="Connected", bg="green")
        self.robot_status_label_easy.config(text="Connected", bg="green")
        self.btn_draw.configure(state=tk.NORMAL, bg= "green")
        self.btn_stop.configure(state=tk.NORMAL)

        # # # Execute the command
        self.process = subprocess.Popen(command)
        
        time.sleep(5)
        self.launch_moveit_planning_easy(robot_type= self.robot_type)
        time.sleep(1)
        print("----------------------\nPlease Run 'ur_robot_driver' on your robot!\n---------------------------------")
        
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
        command = ['roslaunch', 'ur_gazebo', 'ur3e_bringup.launch']
        self.robot_type = "ur3e"
        self.robot_status_label.config(text="Connected", bg="green")
        self.btn_draw.configure(state=tk.NORMAL, bg= "green")
        self.btn_stop.configure(state=tk.NORMAL)
        # Execute the command
        self.process = subprocess.Popen(command) 
    
    def launch_moveit_planning(self): # Change the robot type here
        if self.connection_type.get() == "real": 
            if self.robot_type == "ur3e":
                command = ["roslaunch", "ur3e_moveit_config", "moveit_planning_execution.launch"]
            elif self.robot_type == "ur3": 
                command = ["roslaunch", "ur3_moveit_config", "moveit_planning_execution.launch"]
        else: 
            if self.robot_type == "ur3e":
                command = ["roslaunch", "ur3e_moveit_config", "moveit_planning_execution.launch", "sim:=true"]
            elif self.robot_type == "ur3": 
                command = ["roslaunch", "ur3_moveit_config", "moveit_planning_execution.launch", "sim:=true"]
            
        
        # Execute the command
        self.process = subprocess.Popen(command) 

    def launch_moveit_planning_easy(self, robot_type:str):
        if robot_type == "ur3e":
            command = ["roslaunch", "ur3e_moveit_config", "moveit_planning_execution.launch"]
        elif robot_type == "ur3": 
            command = ["roslaunch", "ur3_moveit_config", "moveit_planning_execution.launch"]

        # Execute the command
        self.process = subprocess.Popen(command) 

    def launch_moveit(self): # Change robot type here
        # Construct the command to execute
        command = ["roslaunch", "ur3e_moveit_config", "moveit_rviz.launch"]
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
        self.ur3_operate.set_robot_type(self.robot_type)

    def init_robot_ur3_easy(self):
        # Initialize UR3
        self.ur3_operate = UR3_Movement()

        # Start get thread Robot TCP
        self.ur3_operate.update_robot_tcp_thread()
        time.sleep(0.2)
        # Start update thread Robot TCP
        self.init_update_tcp_thread()

        print("----------------------\nCareful !! Robot will move !!\n---------------------------------")
        time.sleep(5)
        self.homing_ur3()

    def homing_ur3(self):
        # Homing robot with specific joint state
        self.ur3_operate.homing_ur3()

    def set_new_home_ur3(self):
        self.ur3_operate.set_new_home()

    def reset_home_ur3(self):
        self.ur3_operate.reset_home()

    def set_origin_ur3(self):
        tcp_pose_x,tcp_pose_y,tcp_pose_z,tcp_ori_x,tcp_ori_y,tcp_ori_z = self.ur3_operate.get_tcp()
        self.desire_x_pos = tcp_pose_x
        self.desire_y_pos = tcp_pose_y
        print("\nSet Desired X,Y pos: [",self.desire_x_pos, ", ", self.desire_y_pos, "]")

        self.ur3_operate.set_origin_pose()
        print ("\nSet origin pose")
 
    def start_drawing(self):
        self.ur3_operate.start_drawing()

    def start_drawing_easy(self):
        self.homing_ur3()
        self.ur3_operate.completeGoal_flag.wait()
        self.set_origin_ur3()
        print("----------------------\nOrigin is set !\n---------------------------------")
        self.open_file_dialog()
        self.import_gcode()
        if self.gcode_path:
            print("Robot is ready to Draw!")
            print("Ready in 2...")
            time.sleep(1)
            print("Ready in 1...")
            time.sleep(1)
            self.start_drawing()
        else: print("Cannot Start the Robot.")

    def continue_drawing(self):
        self.ur3_operate.continue_drawing()

    def end_drawing(self):
        self.clear_all_goals()
        time.sleep(0.2)
        self.stop_drawing()

    def stop_drawing(self):
        self.ur3_operate.stop_movement()
        time.sleep(0.3)
        self.ur3_operate.release_stop_event()
        time.sleep(0.1)
        self.ur3_operate.homing_ur3()

    def release_stop(self):
        self.ur3_operate.release_stop_event()

    def print_ur3_pose(self):
        print("\nCurrent pose:", self.ur3_operate.move_group.get_current_pose().pose)
        print("\nCurrent Joint: ", self.ur3_operate.move_group.get_current_joint_values())

    def clear_all_goals(self):
        print("\n----------------\nAll Goals are clear !!!\n------------------\n")
        self.ur3_operate.clear_all_goals()



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

            self.x_tcp = tcp_pose_x * 1000 # convert m to mm
            self.y_tcp = tcp_pose_y * 1000
            self.z_tcp = tcp_pose_z * 1000
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

    def stop_countdown(self):
        self.running = False

    def _run_timer(self):
        # count = self.duration
        count = self.countdown_value
        while count >= 0 and self.running:
            print(f"Countdown now: {count}")
            self.lbl_countdown_display.config(text=f"Countdown:\n{count}s")
            time.sleep(1)  # Sleep for 1 second
            count -= 1

        if self.running:
            if self.callback:
                self.callback()
                self.stop_countdown()

    def set_countdown(self,seconds):
        self.countdown_value = seconds
        self.update_countdown_display()

    def update_countdown_display(self):
        self.lbl_countdown_display.config(text=f"Countdown:\n{self.countdown_value}s")


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
        self.image_processor.update_preview(self.canvas_live_camera, self.SCREEN_WIDTH, self.SCREEN_HEIGHT)

    def process_img(self):
        get_name = self.entry_name.get()
        get_hashtag = self.entry_hashtag.get()
        self.image_processor.get_user_name(get_name= get_name)
        self.image_processor.get_hashtag(get_hashtag= get_hashtag)
        self.image_processor.process_img(self.canvas_processed_image, self.canvas_traced_outline_image)

    #--------------------- Buttons for Gcode processing

    def generate_gcode(self): # convert SVG file to Gcode
        self.gcode_processor.generate_gcode()
        self.gcode_processor.generate_gcode_text()


    def offset_gcode(self, gcode_path, offset_x, offset_y):
        new_gcode_path = self.gcode_processor.offset_gcode(gcode_path= gcode_path, offset_x= offset_x, offset_y= offset_y)
        return new_gcode_path

    def gcode2pose(self, new_gcode_path): # adjust the Z different
        pose_goal_positions = self.gcode_processor.gcode2pose(new_gcode_path= new_gcode_path, robot_type= self.robot_type)
        return pose_goal_positions
