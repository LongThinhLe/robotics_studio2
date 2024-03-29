import tkinter as tk
from tkinter import ttk, filedialog
from tkinter.ttk import *

import cv2
from PIL import Image, ImageTk
import os
import numpy as np
from rembg import remove
from PIL import Image


class SelfieDrawingApp:
    def __init__(self, master):
        self.master = master
        master.title("Automated Artistic Portraiture")
        master.resizable(False, False)

        # Get the user's home directory
        self.home_directory = os.path.expanduser("~")

        # Create a notebook (tabbed interface)
        self.notebook = ttk.Notebook(master)
        self.notebook.pack(fill='both', expand=True)

        # Create the tabs
        self.tab_take_picture = ttk.Frame(self.notebook)
        self.tab_image_processing = ttk.Frame(self.notebook)
        self.tab_generate_gcode = ttk.Frame(self.notebook)
        self.tab_robot_draw = ttk.Frame(self.notebook)

        # Add tabs to the notebook
        self.notebook.add(self.tab_take_picture, text="Take Picture")
        self.notebook.add(self.tab_image_processing, text="Image Processing")
        self.notebook.add(self.tab_generate_gcode, text="Generate Gcode")
        self.notebook.add(self.tab_robot_draw, text="Robot Draw")


        # Initialize components for the "Take Picture" tab
        self.init_take_picture_tab()

        # Initialize components for the "Image Processing" tab
        self.init_image_processing_tab()

        # Initialize Icon for application
        self.init_icon()



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
                

    def init_take_picture_tab(self):
        # Create a frame to hold the preview screen and its label
        preview_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        preview_frame.pack(side=tk.LEFT, padx=10, pady=10)

        # Define the desired height for the screens
        screen_height = 480  # Adjusted to 480 pixels

        # Create a canvas for the preview screen
        self.canvas_preview = tk.Canvas(preview_frame, width=640, height=screen_height)
        self.canvas_preview.pack()

        # Create a label for the preview screen with increased font size
        lbl_preview = tk.Label(preview_frame, text="Preview Screen", font=("Arial", 20))
        lbl_preview.pack()

        # Create a frame to hold the capture screen and its label
        capture_frame = tk.Frame(self.tab_take_picture, bd=2, relief=tk.SOLID)
        capture_frame.pack(side=tk.LEFT, padx=10, pady=10)

        # Create a canvas for the captured picture screen
        self.canvas_capture = tk.Canvas(capture_frame, width=640, height=screen_height)
        self.canvas_capture.pack()

        # Create a label for the capture screen with increased font size
        lbl_capture = tk.Label(capture_frame, text="Capture Screen", font=("Arial", 20))
        lbl_capture.pack()

        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.tab_take_picture)
        button_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill='y')  # Fill the available vertical space

        # Create the buttons for taking a picture and resetting
        btn_capture = tk.Button(button_frame, text="Take Picture", command=self.take_picture, width=20)  # Set width to fill available horizontal space
        btn_reset = tk.Button(button_frame, text="Reset", command=self.reset, width=20)  # Set width to fill available horizontal space
        btn_capture.pack(side=tk.TOP, padx=10, pady=5, fill='x')  # Fill the available horizontal space
        btn_reset.pack(side=tk.TOP, padx=10, pady=5, fill='x')  # Fill the available horizontal space

        # Initialize captured photo variable
        self.photo = None

        # Start the webcam preview
        self.start_preview()

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

        if ret:
            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to fit the preview canvas
            screen_height = self.canvas_preview.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (640, screen_height))

            # Convert the frame to ImageTk format
            photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

            # Update the preview canvas with the new frame
            self.canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
            self.canvas_preview.image = photo

        # Schedule the next update
        self.canvas_preview.after(10, self.update_preview)

    def take_picture(self):
        # Capture a frame
        ret, frame = self.cap.read()

        if ret:
            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to fit the capture canvas
            screen_height = self.canvas_capture.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (640, screen_height))

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
            print("Picture saved:", file_path)

    def reset(self):
        # Clear the captured photo canvas
        self.canvas_capture.delete("all")
        self.photo = None

    def __del__(self):
        # Release the webcam when the application is closed
        if hasattr(self, 'cap'):
            self.cap.release()

    def init_image_processing_tab(self):
        # Create a frame to hold the image processing components
        processing_frame = tk.Frame(self.tab_image_processing, bd=2, relief=tk.SOLID)
        processing_frame.pack(fill='both', padx=10, pady=10)

        # Create a label for the processed image
        lbl_processed_image = tk.Label(processing_frame, text="Processed Image", font=("Arial", 20))
        lbl_processed_image.pack()

        # Create a canvas for displaying the processed image
        self.canvas_processed_image = tk.Canvas(processing_frame, width=640, height=480)
        self.canvas_processed_image.pack()

        # Create a button to process the captured image
        btn_process_image = tk.Button(processing_frame, text="Process Captured Image", command=self.process_captured_image, width=20)
        btn_process_image.pack(pady=5)

        # Initialize processed image variable
        self.processed_image = None

    def process_captured_image(self):
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


    def display_processed_image(self, image):
        # Clear the canvas
        self.canvas_processed_image.delete("all")

        # Convert the image to ImageTk format
        photo = ImageTk.PhotoImage(image=image)

        # Update the canvas with the processed image
        self.canvas_processed_image.create_image(0, 0, anchor=tk.NW, image=photo)
        self.canvas_processed_image.image = photo



def main():
    root = tk.Tk()
    app = SelfieDrawingApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()