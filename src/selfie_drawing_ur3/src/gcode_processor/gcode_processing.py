#!/usr/bin/env python3

import os
import svgpathtools as svg


class GcodeProcessing:
    # Define SVG output size
    SVG_WIDTH   = 180/1000 # unit in mm
    SVG_HEIGHT  = 140/1000 # unit in mm

    def __init__(self) -> None:
        # Get the user's home directory
        self.home_directory = os.path.expanduser("~")

    def generate_gcode(self): # convert SVG file to Gcode
        # Check if the svg file exists
        svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")
        if not os.path.exists(svg_path):
            print("SVG file not found.")
            return

        # Load SVG file
        paths, _ = svg.svg2paths(svg_path)

        # Calculate SVG bounding box dimensions
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

        svg_width = max_x - min_x
        svg_height = max_y - min_y

        # Desired dimensions in mm
        desired_width = self.SVG_WIDTH
        desired_height = self.SVG_HEIGHT

        # Calculate scale factors for both dimensions and choose the smaller one to maintain aspect ratio
        scale_x = desired_width / svg_width
        scale_y = desired_height / svg_height
        scale = min(scale_x, scale_y)

        # Calculate SVG center
        svg_center_x = (min_x + max_x) / 2
        svg_center_y = (min_y + max_y) / 2

        # Desired center point (center coordinate)
        desired_center_x, desired_center_y = 0, 0  # Modify as needed

        # Calculate displacement
        displacement_x = desired_center_x - svg_center_x
        displacement_y = desired_center_y - svg_center_y

        save_folder_gcode = os.path.join(self.home_directory, "rs2_ws", "gcode")
        
        # Ensure the save folder exists, create it if it doesn't
        if not os.path.exists(save_folder_gcode):
            os.makedirs(save_folder_gcode)

        gcode_path = os.path.join(self.home_directory, "rs2_ws", "gcode", "your_portrait.gcode")

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


    def offset_gcode(self, gcode_path, offset_x, offset_y):
        # Create folder for offset files if it doesn't exist
        save_folder_offset = os.path.join(self.home_directory, "rs2_ws", "offset_files")
        if not os.path.exists(save_folder_offset):
            os.makedirs(save_folder_offset)

        new_gcode_path = os.path.splitext(os.path.basename(gcode_path))[0] + "_offset.gcode"
        new_gcode_path = os.path.join(save_folder_offset, new_gcode_path)

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
                            x_coord += offset_x * 1.08  # Adjust this parameter to modify the frame inside robot working area
                            y_coord += offset_y
                            # Write modified line to new file
                            new_line = f"{parts[0]} X{x_coord:.6f} Y{y_coord:.6f}\n"
                            new_file.write(new_line)
                    else:
                        # Write non-coordinate lines unchanged
                        new_file.write(line)

        return new_gcode_path 

    def gcode2pose(self, new_gcode_path, robot_type): # adjust the Z different
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
                    z = 0.14 # 0.16
                    pose_goal_positions.append([x,y,z])

                elif line.startswith('G1'):
                    # Extract X,Y coordinates from the gcode file
                    x = float(line.split('X')[1].split(' ')[0])
                    y = float(line.split('Y')[1].split(' ')[0])
                    if robot_type == "ur3e":
                        z = 0.135 # ur3: 0.125 #ur3e: 0.13 # go up
                    else: z = 0.1285 # go down
                    pose_goal_positions.append([x,y,z])

        return pose_goal_positions