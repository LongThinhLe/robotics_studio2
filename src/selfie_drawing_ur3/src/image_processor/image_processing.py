#!/usr/bin/env python3

import cv2
import tkinter as tk
from tkinter import Canvas
from tkinter.ttk import *
from PIL import Image, ImageTk
import os
from rembg import remove
import svgwrite
import cairosvg
import io

class ImageProcessor:
    def __init__(self) -> None:
        # Get the user's home directory
        self.home_directory = os.path.expanduser("~")
        self.cap = None
        # Initialize Webcam
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
        
    
    def update_preview(self,canvas_preview, screen_width, screen_height):
        # Capture a frame
        ret, frame = self.cap.read()
        # ret = None
        # frame = None
        if ret:

            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to fit the preview canvas
            screen_height = canvas_preview.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (screen_width, screen_height))

            # Convert the frame to ImageTk format
            photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

            # Update the preview canvas with the new frame
            canvas_preview.create_image(0, 0, anchor=tk.NW, image=photo)
            canvas_preview.image = photo

        # Schedule the next update
        # Explain:  widget.after(delay_ms, callback, *args)   --- in tkinter
        canvas_preview.after(10, self.update_preview, canvas_preview, screen_width, screen_height)

    def take_picture(self,canvas_capture: Canvas, screen_width:int , screen_height: int):
        # Capture a frame
        ret, frame = self.cap.read()

        if ret:
            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Resize the frame to fit the capture canvas
            screen_height = canvas_capture.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (screen_width, screen_height))

            # Convert the frame to ImageTk format
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

            # Update the capture canvas with the captured picture
            canvas_capture.create_image(0, 0, anchor=tk.NW, image=self.photo)
            canvas_capture.image = self.photo

            # Determine the save folder path
            save_folder = os.path.join(self.home_directory, "rs2_ws", "img")

            # Ensure the save folder exists, create it if it doesn't
            if not os.path.exists(save_folder):
                os.makedirs(save_folder)

            # Save the captured picture to the specified folder
            
            file_path = os.path.join(save_folder, "captured_picture.png")
            cv2.imwrite(file_path, cv2.cvtColor(frame_resized, cv2.COLOR_RGB2BGR))
            print("\nPicture saved:", file_path)

    def process_img(self, canvas_processed_image, canvas_traced_outline_image):
        # Process image after taking picture
        print("\nProcessing Image, please wait...")
        self.remove_background_image(canvas_processed_image)
        self.trace_outline(canvas_traced_outline_image)

    def remove_background_image(self, canvas_processed_image):
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
        self.display_processed_image(canvas_processed_image,output_image)
        print("\nRemoved background image:", store_path) 

    def display_processed_image(self,canvas_processed_image: Canvas, image):
        # Clear the canvas
        canvas_processed_image.delete("all")

        # Convert the image to ImageTk format
        photo = ImageTk.PhotoImage(image=image)

        # Update the canvas with the processed image
        canvas_processed_image.create_image(0, 0, anchor=tk.NW, image=photo)
        canvas_processed_image.image = photo

    def trace_outline(self, canvas_traced_outline_image):
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
        self.display_trace_outline(canvas_traced_outline_image)
        print("\nSVG image saved:", output_svg_path)

    def display_trace_outline(self,canvas_traced_outline_image: Canvas):
        file_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")
        with open(file_path, "rb") as f:
            svg_data = f.read()
        
        png_data = self.svg_to_png(svg_data)

        # Open the PNG image with PIL
        svg_image = Image.open(png_data)

        # Convert the PIL Image to a PhotoImage
        photo = ImageTk.PhotoImage(svg_image)

        # Display the image on the canvas
        canvas_traced_outline_image.create_image(0, 0, anchor=tk.NW, image=photo)
        canvas_traced_outline_image.image = photo

    def svg_to_png(self,svg_data): # for display purpose
        png_data = cairosvg.svg2png(bytestring=svg_data)
        return io.BytesIO(png_data)