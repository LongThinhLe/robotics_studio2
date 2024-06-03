
    # def trace_outline(self, canvas_traced_outline_image):
    #     # Check if the captured image exists
    #     file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")
    #     if not os.path.exists(file_path):
    #         print("Image not found.")
    #         return

    #     output_svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")

    #     # Read the image
    #     image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)

    #     # Apply Canny edge detection
    #     edges = cv2.Canny(image, 1, 700)

    #     # Find contours
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     # Create SVG object
    #     svg = svgwrite.Drawing(output_svg_path, profile='tiny')

    #     # Iterate through contours
    #     for contour in contours:
    #         # Approximate contour to reduce points
    #         epsilon = 0.01 * cv2.arcLength(contour, True)
    #         approx = cv2.approxPolyDP(contour, epsilon, True)

    #         # Convert contour points to SVG format
    #         points = [(int(point[0][0]), int(point[0][1])) for point in approx]
    #         svg.add(svg.polyline(points, stroke="black", fill="none"))

    #     # Set the size of the SVG drawing
    #     svg['width'] = '640px'  # Set the width of the SVG
    #     svg['height'] = '480px'  # Set the height of the SVG

    #     # Save SVG file
    #     svg.save()

    #     # Display the processed image
    #     self.display_trace_outline(canvas_traced_outline_image)
    #     print("\nSVG image saved:", output_svg_path)



    # def trace_outline(self, canvas_traced_outline_image): # Experiment
    #     # Check if the captured image exists
    #     file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")
    #     if not os.path.exists(file_path):
    #         print("Image not found.")
    #         return

    #     output_svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")

    #     # Read the RGB image and convert it to grayscale
    #     rgb_image = cv2.imread(file_path)
    #     gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

    #     # Apply Gaussian blur to reduce noise and enhance feature detection
    #     blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
        

    #     # Apply adaptive thresholding to further enhance edges
    #     # _, threshold_image = cv2.threshold(blurred_image, 20, 255, cv2.THRESH_BINARY) # 140, 255
    #     threshold_image = cv2.adaptiveThreshold(blurred_image, 50, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 4) # 19,4

    #     # Apply Canny edge detection
    #     edges = cv2.Canny(threshold_image, 50, 190) # 50, 120

    #     # Find contours
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     # Create SVG object
    #     svg = svgwrite.Drawing(output_svg_path, profile='tiny')

    #     # Smoothness parameter for contour approximation
    #     smoothness = 0.0005  # Adjust this value for desired smoothness # Keep this value 0.0005

    #     # Minimum segment length threshold (in pixels)
    #     min_segment_length = 9  # Adjust this threshold as needed
        
    #     # Iterate through contours
    #     for contour in contours:
    #         # Calculate contour area
    #         area = cv2.contourArea(contour)
            
    #         # Define minimum area threshold to keep contour
    #         min_area_threshold = 10  # Adjust this threshold as needed
            
    #         # If contour area is above threshold, keep it
    #         if area > min_area_threshold:
    #             # Approximate contour to reduce points
    #             epsilon = smoothness * cv2.arcLength(contour, True)
    #             approx = cv2.approxPolyDP(contour, epsilon, True)

    #             # Convert contour points to SVG format
    #             points = [(int(point[0][0]), int(point[0][1])) for point in approx]

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






    # def trace_outline(self, canvas_traced_outline_image): # GOOD
    #     # Check if the captured image exists
    #     file_path = os.path.join(self.home_directory, "rs2_ws", "img", "captured_picture_rmbg.png")
    #     if not os.path.exists(file_path):
    #         print("Image not found.")
    #         return

    #     output_svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")

    #     # Read the RGB image and convert it to grayscale
    #     rgb_image = cv2.imread(file_path)
    #     gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

    #     # Apply Gaussian blur to reduce noise and enhance feature detection
    #     blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
        

    #     # Apply adaptive thresholding to further enhance edges
    #     # _, threshold_image = cv2.threshold(blurred_image, 20, 255, cv2.THRESH_BINARY) # 140, 255
    #     threshold_image = cv2.adaptiveThreshold(blurred_image, 50, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)

    #     # Apply Canny edge detection
    #     edges = cv2.Canny(threshold_image, 50, 190) # 50, 120

    #     # Find contours
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     # Create SVG object
    #     svg = svgwrite.Drawing(output_svg_path, profile='tiny')

    #     # Smoothness parameter for contour approximation
    #     smoothness = 0.0005  # Adjust this value for desired smoothness # Keep this value 0.0005
        
    #     # Iterate through contours
    #     for contour in contours:
    #         # Calculate contour area
    #         area = cv2.contourArea(contour)
            
    #         # Define minimum area threshold to keep contour
    #         min_area_threshold = 10  # Adjust this threshold as needed
            
    #         # If contour area is above threshold, keep it
    #         if area > min_area_threshold:
    #             # Approximate contour to reduce points
    #             epsilon = smoothness * cv2.arcLength(contour, True)
    #             approx = cv2.approxPolyDP(contour, epsilon, True)

    #             # Convert contour points to SVG format
    #             points = [(int(point[0][0]), int(point[0][1])) for point in approx]
    #             svg.add(svg.polyline(points, stroke="black", fill="none"))

    #     # Set the size of the SVG drawing
    #     svg['width'] = '640px'  # Set the width of the SVG
    #     svg['height'] = '480px'  # Set the height of the SVG

    #     # Save SVG file
    #     svg.save()

    #     # Display the processed image
    #     self.display_trace_outline(canvas_traced_outline_image)
    #     print("\nSVG image saved:", output_svg_path)
