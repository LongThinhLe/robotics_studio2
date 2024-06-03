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
import dlib
import numpy as np
from imutils import face_utils


class ImageProcessor:
    # Set constant for enabling Camera
    CAMERA_ENABLE = True


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

        # Path for training model
        self.path_training_model = os.path.join(self.home_directory, "rs2_ws", "training_model",  "shape_predictor_68_face_landmarks.dat")

        # Initialize dlib's face detector (HOG-based) and then create the facial landmark predictor
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(self.path_training_model)


    # Define the function to interpolate points for Showing points recognizing Facial Features
    def interpolate_points(self, points, factor=2):
        new_points = []
        for i in range(len(points) - 1):
            x_vals = np.linspace(points[i][0], points[i+1][0], factor + 1)
            y_vals = np.linspace(points[i][1], points[i+1][1], factor + 1)
            new_points.extend(zip(x_vals, y_vals))
        new_points.append(points[-1])
        return np.array(new_points, dtype=int)


    def update_preview(self, canvas_live_camera, screen_width, screen_height):
        # Capture a frame
        if self.CAMERA_ENABLE:
            ret, frame = self.cap.read()
        else:
            ret = None
            frame = None

        #----- Detect Face Features in Live Camera


        if ret:
            # Convert the frame from BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Covert the frame from BGR to Gray
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces in the grayscale image
            rects = self.detector(frame_gray, 0)

            # Loop over the face detections
            for (i, rect) in enumerate(rects):
                # Determine the facial landmarks for the face region, then convert the facial landmark (x, y)-coordinates to a NumPy array
                shape = self.predictor(frame_gray, rect)
                shape = face_utils.shape_to_np(shape)
                
                # Interpolate points to increase resolution
                interpolated_shape = self.interpolate_points(shape, factor=3)
                
                # Loop over the (x, y)-coordinates for the facial landmarks and draw them on the image
                for (x, y) in interpolated_shape:
                    cv2.circle(frame_rgb, (x, y), 1, (0, 255, 0), -1)

            # Resize the frame to fit the preview canvas
            screen_height = canvas_live_camera.winfo_height()
            frame_resized = cv2.resize(frame_rgb, (screen_width, screen_height))

            # Convert the frame to ImageTk format
            photo = ImageTk.PhotoImage(image=Image.fromarray(frame_resized))

            # Update the preview canvas with the new frame
            canvas_live_camera.create_image(0, 0, anchor=tk.NW, image=photo)
            canvas_live_camera.image = photo




        # Schedule the next update
        # Explain:  widget.after(delay_ms, callback, *args)   --- in tkinter
        canvas_live_camera.after(10, self.update_preview, canvas_live_camera, screen_width, screen_height)

    def take_picture(self,canvas_capture: Canvas, screen_width:int , screen_height: int):
        # Capture a frame       
        if self.CAMERA_ENABLE:
            ret, frame = self.cap.read()
        else:
            ret = None
            frame = None


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



    def chaikin_smoothing(self, points, iterations=2):
        for _ in range(iterations):
            new_points = []
            for i in range(len(points) - 1):
                p0 = points[i]
                p1 = points[i + 1]
                new_points.append((0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])) # 1/4 and 3/4 rules
                new_points.append((0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1]))
            new_points.append(points[-1])
            points = new_points
        return points

    def perpendicular_distance(self, point, start, end):
        if start == end:
            return np.linalg.norm(np.array(point) - np.array(start))
        else:
            return np.abs(np.cross(np.array(end) - np.array(start), np.array(start) - np.array(point))) / np.linalg.norm(np.array(end) - np.array(start))

    def rdp(self, points, minimum_length): # Ramer-Douglas-Peucker algorithm
        if len(points) < 3:
            return points

        start, end = points[0], points[-1]
        max_distance = 0
        index = 0

        for i in range(1, len(points) - 1):
            distance = self.perpendicular_distance(points[i], start, end)
            if distance > max_distance:
                index, max_distance = i, distance

        if max_distance > minimum_length:
            results1 = self.rdp(points[:index + 1], minimum_length)
            results2 = self.rdp(points[index:], minimum_length)
            return results1[:-1] + results2
        else:
            return [start, end]


    def create_circle(self, center, radius, num_points=4):
        circle_points = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            circle_points.append((int(x), int(y)))
        circle_points.append(circle_points[0])  # Close the circle
        return circle_points



    def trace_outline(self, canvas_traced_outline_image): # GOOD 
        # Check if the captured image exists
        file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")
        if not os.path.exists(file_path):
            print("Image not found.")
            return

        output_svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")

        # Read the RGB image and convert it to grayscale
        rgb_image = cv2.imread(file_path)
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and enhance feature detection
        blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)
        
        # Apply adaptive thresholding to further enhance edges
        # EXPLAIN: retval, dst = cv2.threshold(src, thresh, maxval, type)
        # thresh: The threshold value. Any pixel value in the source image that is greater than this value will be assigned the maxval,
        # and any pixel value that is less than or equal to this value will be assigned zero (or another specified value depending on the thresholding type).
        
        # maxval: When the pixel value is greater than the threshold value, it is set to this maxval.
        _, threshold_image = cv2.threshold(blurred_image, 100, 255, cv2.THRESH_BINARY) # 80, 255: GOOD  140, 255 OUTLINE ONLY

        # threshold_image = cv2.adaptiveThreshold(blurred_image, 50, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)

        # Apply Canny edge detection
        # threshold 1: IF the gradient magnitude is below this threshold, it is considered not an edge
        # threshold 2: IF the gradient mangitude is above this threshold, it is considered an edge
        edges = cv2.Canny(gray_image, 130, 220) # 100, 255
        # edges = cv2.Canny(threshold_image, 130, 220) # 100, 255

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create SVG object
        svg = svgwrite.Drawing(output_svg_path, profile='tiny')

        # Smoothness parameter for contour approximation
        svg_resolution = 0.0005  # Adjust this value for desired smoothness # Keep this value 0.0005
        
        # Iterate through contours
        for contour in contours:
            # Calculate contour area
            area = cv2.contourArea(contour)
            
            # Define minimum area threshold to keep contour
            min_area_threshold = 10  # Adjust this threshold as needed
            
            # If contour area is above threshold, keep it
            if area > min_area_threshold:
                # Approximate contour to reduce points
                epsilon = svg_resolution * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Convert contour points to SVG format
                points = [(int(point[0][0]), int(point[0][1])) for point in approx]
                smoothed_points = self.chaikin_smoothing(points, iterations= 2)
                reduced_points = self.rdp(smoothed_points, minimum_length= 1.5)
                svg.add(svg.polyline(reduced_points, stroke="black", fill="none"))


        # Define indices for each facial feature
        features = {
            "jaw": list(range(0, 17)),
            "left_eyebrow": list(range(17, 22)),
            "right_eyebrow": list(range(22, 27)),
            "nose": list(range(28, 36)),
            "left_eye": list(range(36, 42)),
            "right_eye": list(range(42, 48)),
            "outer_mouth": list(range(48, 60)),
            "inner_mouth": list(range(60, 68))
        }

        # Detect faces in the grayscale image
        rects = self.detector(gray_image, 0)

        # Process each face detection for facial landmarks
        for rect in rects:
            shape = self.predictor(gray_image, rect)
            shape = face_utils.shape_to_np(shape)

            # Interpolate points to increase resolution
            for feature_name, indices in features.items():
                feature_points = shape[indices]
                points = [(int(x), int(y)) for (x, y) in feature_points]
                if feature_name in ["left_eye", "right_eye", "outer_mouth", "inner_mouth"]:
                    points.append(points[0])  # Close the loop
                svg.add(svg.polyline(points, stroke="black", fill="none"))

                # Create retinas for eyes
                if feature_name in ["left_eye", "right_eye"]:
                    eye_center = np.mean(feature_points, axis=0).astype(int)
                    retina_radius = 4  # Adjust radius as needed
                    retina_points = self.create_circle(eye_center, retina_radius)
                    svg.add(svg.polyline(retina_points, stroke="black", fill="none"))

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
    











    # def trace_outline(self, canvas_traced_outline_image): # TESTING
    #     # Check if the captured image exists
    #     file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")
    #     # file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_enhanced.png")
    #     if not os.path.exists(file_path):
    #         print("Image not found.")
    #         return

    #     output_svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")

    #     # Read the RGB image and convert it to grayscale
    #     rgb_image = cv2.imread(file_path)
    #     gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

    #     # Apply Gaussian blur to reduce noise and enhance feature detection
    #     blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)
        

    #     # Apply adaptive thresholding to further enhance edges
    #     # _, threshold_image = cv2.threshold(blurred_image, 1, 10, cv2.THRESH_BINARY) # 140, 255
    #     threshold_image = cv2.adaptiveThreshold(blurred_image, 50, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)

    #     # Apply Canny edge detection
    #     edges = cv2.Canny(threshold_image, 50, 200) # 50, 120

    #     # Find contours
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     # Create SVG object
    #     svg = svgwrite.Drawing(output_svg_path, profile='tiny')

    #     # Smoothness parameter for contour approximation
    #     smoothness = 0.000005  # Adjust this value for desired smoothness # Keep this value 0.0005
        
    #     # Minimum segment length threshold (in pixels)
    #     min_segment_length = 4  # Adjust this threshold as needed

    #     # Iterate through contours
    #     for contour in contours:
    #         # Calculate contour area
    #         area = cv2.contourArea(contour)
            
    #         # Define minimum area threshold to keep contour
    #         min_area_threshold = 5  # Adjust this threshold as needed
            
    #         # If contour area is above threshold, keep it
    #         if area > min_area_threshold:
    #             # Approximate contour to reduce points
    #             epsilon = smoothness * cv2.arcLength(contour, True)
    #             approx = cv2.approxPolyDP(contour, epsilon, True)

    #             # Convert contour points to SVG format
    #             points = [(int(point[0][0]), int(point[0][1])) for point in approx]
    #             # svg.add(svg.polyline(points, stroke="black", fill="none"))


    #             # Combine small line segments into a single line segment
    #             combined_points = [points[0]]
    #             for i in range(1, len(points)):
    #                 if cv2.norm(points[i], points[i-1]) > min_segment_length:
    #                     combined_points.append(points[i])

    #             svg.add(svg.polyline(combined_points, stroke="black", fill="none"))


    #     # Set the size of the SVG drawing
    #     svg['width'] = '640px'  # Set the width of the SVG
    #     svg['height'] = '480px'  # Set the height of the SVG

    #     # Save SVG file
    #     svg.save()

    #     # Display the processed image
    #     self.display_trace_outline(canvas_traced_outline_image)
    #     print("\nSVG image saved:", output_svg_path)


