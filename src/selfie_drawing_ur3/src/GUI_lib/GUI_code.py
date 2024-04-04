#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk, Scale
from tkinter.ttk import *

from PIL import Image, ImageTk
import os
from rembg import remove
from PIL import Image
import svgpathtools as svg

from image_processor.image_processing import ImageProcessor

import threading
import time

class SelfieDrawingApp:

    # Define global variables for screen width and height in ratio 4:3
    SCREEN_WIDTH = 320
    SCREEN_HEIGHT = 240

    def __init__(self, master: tk.Tk):
        self.master = master
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

        # Initialize Image Processor
        self.image_processor = ImageProcessor()

        # Initialize components for the "Take Picture" tab
        self.init_take_picture_tab()

        # Initialize components for the "Robot Draw" tab
        self.init_robot_draw_tab()
 
        # Initialize Icon for application
        self.init_icon()


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

        # Create the buttons for taking a picture and resetting
        # self.btn_capture = tk.Button(button_frame, text="Take Picture", command=lambda: self.image_processor.take_picture(self.canvas_capture, screen_width, screen_height), width=15, height= 5)
        self.btn_capture = tk.Button(button_frame, text="Take Picture", width=15, height= 5)
        btn_process_img = tk.Button(button_frame, text="Process Image", command=lambda: self.process_img(), width=15, height= 5)
        btn_generate_gcode = tk.Button(button_frame, text="Generate Gcode", command=self.generate_gcode, width=15, height= 5)

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
        # Create a frame for "Robot Status"
        robot_status_frame = tk.Frame(self.tab_robot_draw, bd=2, relief=tk.SOLID)
        robot_status_frame.grid(row=0, column=0, padx=20, pady=10, ipadx=5, ipady=5)

        # Create a label for "Robot Status"
        lbl_robot_status = tk.Label(robot_status_frame, text="Robot Status", font=("Arial", 20))
        lbl_robot_status.pack()

        # Create a label to indicate the status
        self.robot_status_label = tk.Label(robot_status_frame, text="Disconnected", font=("Arial", 16), bg="red", fg="white")
        self.robot_status_label.pack()

        # Create a frame for "End-effector Position"
        end_effector_frame = tk.Frame(self.tab_robot_draw, bd=2, relief=tk.SOLID)
        end_effector_frame.grid(row=0, column=1, padx=20, pady=10, ipadx=5, ipady=5)

        # Create a label for "End-effector Position"
        lbl_end_effector = tk.Label(end_effector_frame, text="UR3 TCP", font=("Arial", 20))
        lbl_end_effector.pack()

        # Create a frame to stack labels horizontally
        coordinates_frame = tk.Frame(end_effector_frame)
        coordinates_frame.pack()

        x = 10
        y = 5
        z = 2
        rx = 10
        ry = 0
        rz = 0

        # Create labels to display the coordinates
        self.lbl_x = tk.Label(coordinates_frame, text="X:{:.2f} mm".format(x), font=("Arial", 16), bg="lightblue")
        self.lbl_y = tk.Label(coordinates_frame, text="Y:{:.2f} mm".format(y), font=("Arial", 16))
        self.lbl_z = tk.Label(coordinates_frame, text="Z:{:.2f} mm".format(z), font=("Arial", 16))
        self.lbl_rx = tk.Label(coordinates_frame, text="Rx:{:.2f} deg".format(rx), font=("Arial", 16))
        self.lbl_ry = tk.Label(coordinates_frame, text="Ry:{:.2f} deg".format(ry), font=("Arial", 16))
        self.lbl_rz = tk.Label(coordinates_frame, text="Rz:{:.2f} deg".format(rz), font=("Arial", 16))

        self.lbl_x.pack(side=tk.LEFT, padx=5)
        self.lbl_y.pack(side=tk.LEFT, padx=5)
        self.lbl_z.pack(side=tk.LEFT, padx=5)
        self.lbl_rx.pack(side=tk.LEFT, padx=5)
        self.lbl_ry.pack(side=tk.LEFT, padx=5)
        self.lbl_rz.pack(side=tk.LEFT, padx=5)
        
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
        count = self.duration
        while count >= 0 and self.running:
            print(f"Countdown now: {count}")
            time.sleep(1)  # Sleep for 1 second
            count -= 1

        if self.running:
            if self.callback:
                self.callback()
                self.stop()

    def set_countdown(self,seconds):
        self.countdown_value = seconds


    #-------------------- Button Event Function
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
        scale = 0.05  # Experiment with this value

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
                        f.write(f"G0 X{start_x:.2f} Y{start_y:.2f}\n")  # Rapid move to start point
                    f.write(f"G1 X{end_x:.2f} Y{end_y:.2f}\n")  # Linear move to end point


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
    


def main():
    root = tk.Tk()
    SelfieDrawingApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()