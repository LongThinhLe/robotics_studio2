import tkinter as tk
from tkinter import ttk, filedialog
from tkinter.ttk import *

import cv2
from PIL import Image, ImageTk
import os
import numpy as np
import svgwrite
import io
import cairosvg
from rembg import remove
from PIL import Image
import svgpathtools as svg
from pygcode import GCode

class SelfieDrawingApp:

    # Define global variables for screen width and height in ratio 4:3
    SCREEN_WIDTH = 320
    SCREEN_HEIGHT = 240

    def __init__(self, master):
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


        # Initialize components for the "Take Picture" tab
        self.init_take_picture_tab()

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
        #-----------------------------------------------------------------------
        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.tab_take_picture)
        button_frame.grid(row=0, column=2, padx=10, pady=10, sticky='ns')  # Adjust row and column as needed

        # Create the buttons for taking a picture and resetting
        btn_capture = tk.Button(button_frame, text="Take Picture", command=self.take_picture, width=15, height= 5)  # Set width to fill available horizontal space
        btn_process_img = tk.Button(button_frame, text="Process Image", command=self.process_img, width=15, height= 5)
        btn__generate_gcode = tk.Button(button_frame, text="Generate Gcode", command=self.generate_gcode, width=15, height= 5)

        btn_capture.grid(row=0, column=0, padx=10, pady=10, sticky='ew')  #
        btn_process_img.grid(row=1, column=0, padx=10, pady=10, sticky='ew')  #
        btn__generate_gcode.grid(row=2, column=0, padx=10, pady=10, sticky='ew')  #

        # Initialize captured photo variable
        self.photo = None

        # Start the webcam preview
        self.start_preview()

    #-------------------- Webcam Function
    def start_preview(self):
        # Iterate through device indexes until a valid one is found
        for i in range(10):  # Try up to 10 devices
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                self.cap = cap
                print(f"Using webcam device: {i}")
                break

        if self.cap is None:
            print("No webcam device found.")
            return

        # Display the live feed in the preview canvas
        self.update_preview()
        
    def update_preview(self):
        # Capture a frame
        ret, frame = self.cap.read()

        # Define the desired height for the PREVIEW screens
        screen_width = self.SCREEN_WIDTH
        screen_height = self.SCREEN_HEIGHT

        if ret:

            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to fit the preview canvas
            screen_height = self.canvas_preview.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (screen_width, screen_height))

            # Convert the frame to ImageTk format
            photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

            # Update the preview canvas with the new frame
            self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
            self.canvas_preview.image = photo

        # Schedule the next update
        self.canvas_preview.after(10, self.update_preview)


    #-------------------- Button Event Function
    def take_picture(self):
        # Capture a frame
        ret, frame = self.cap.read()

        # Define the desired height for the PREVIEW screens
        screen_width = self.SCREEN_WIDTH
        screen_height = self.SCREEN_HEIGHT

        if ret:
            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to fit the capture canvas
            screen_height = self.canvas_capture.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (screen_width, screen_height))

            # Convert the frame to ImageTk format
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

            # Update the capture canvas with the captured picture
            self.canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
            self.canvas_capture.image = self.photo

            # Determine the save folder path
            save_folder = os.path.join(self.home_directory, "rs2_ws", "img")

            # Ensure the save folder exists, create it if it doesn't
            if not os.path.exists(save_folder):
                os.makedirs(save_folder)

            # Save the captured picture to the specified folder
            
            file_path = os.path.join(save_folder, "captured_picture.png")
            cv2.imwrite(file_path, cv2.cvtColor(frame_resized, cv2.COLOR_RGB2BGR))
            print("\nPicture saved:", file_path)

    def process_img(self):
        # Process image after taking picture
        print("\nProcessing Image, please wait...")
        self.remove_background_image()
        self.trace_outline()

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
    
    #-------------------- Remove Background and Display Function

    def remove_background_image(self):
        # Check if the captured image exists
        file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture.png")
        if not os.path.exists(file_path):
            print("Captured image not found.")
            return

        # Store path
        store_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")

        # Processing the image 
        input_image = Image.open(file_path)

        # Removing the background from the given Image 
        output_image = remove(input_image)  

        # Removing the background from the given Image 
        output_image.save(store_path)

        # Display the processed image
        self.display_processed_image(output_image)
        print("\nRemoved background image:", store_path)

    def display_processed_image(self, image):
        # Clear the canvas
        self.canvas_processed_image.delete("all")

        # Convert the image to ImageTk format
        photo = ImageTk.PhotoImage(image=image)

        # Update the canvas with the processed image
        self.canvas_processed_image.create_image(0, 0, anchor=tk.NW, image=photo)
        self.canvas_processed_image.image = photo


    #-------------------- Trace Outline and Display Function
    def trace_outline(self):
        # Check if the captured image exists
        file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")
        if not os.path.exists(file_path):
            print("Image not found.")
            return

        output_svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")

        # Read the image
        image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)

        # Apply Canny edge detection
        edges = cv2.Canny(image, 100, 200)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create SVG object
        svg = svgwrite.Drawing(output_svg_path, profile='tiny')

        # Iterate through contours
        for contour in contours:
            # Approximate contour to reduce points
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Convert contour points to SVG format
            points = [(int(point[0][0]), int(point[0][1])) for point in approx]
            svg.add(svg.polyline(points, stroke="black", fill="none"))

        # Set the size of the SVG drawing
        svg['width'] = '640px'  # Set the width of the SVG
        svg['height'] = '480px'  # Set the height of the SVG

        # Save SVG file
        svg.save()

        # Display the processed image
        self.display_trace_outline(svg)
        print("\nSVG image saved:", output_svg_path)

    def svg_to_png(self,svg_data): # for display purpose
        png_data = cairosvg.svg2png(bytestring=svg_data)
        return io.BytesIO(png_data)

    def display_trace_outline(self,svg):
        file_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")
        with open(file_path, "rb") as f:
            svg_data = f.read()
        
        png_data = self.svg_to_png(svg_data)

        # Open the PNG image with PIL
        svg_image = Image.open(png_data)

        # Convert the PIL Image to a PhotoImage
        photo = ImageTk.PhotoImage(svg_image)

        # Display the image on the canvas
        self.canvas_traced_outline_image.create_image(0, 0, anchor=tk.NW, image=photo)
        self.canvas_traced_outline_image.image = photo






def main():
    root = tk.Tk()
    app = SelfieDrawingApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()