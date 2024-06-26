#!/usr/bin/env python3

import os
import svgpathtools as svg
import xml.etree.ElementTree as ET
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties

class GcodeProcessing:
    # Define SVG output size
    SVG_WIDTH   = 90/1000 # 101.7;180 unit in mm
    SVG_HEIGHT  = 135/1000 # 152.5;140 unit in mm

    def __init__(self) -> None:
        # Get the user's home directory
        self.home_directory = os.path.expanduser("~")


    #------------------------ TEXT TO GCODE
    def parse_svg_for_text(self, svg_file):
        tree = ET.parse(svg_file)
        root = tree.getroot()
        text_elements = []
        ns = {'svg': 'http://www.w3.org/2000/svg'}
        for elem in root.findall('.//{http://www.w3.org/2000/svg}text', ns):
            text_elements.append(elem)
        return text_elements, root, ns


    def text_to_path(self, text_elem, font_properties):
        text = text_elem.text
        x = float(text_elem.attrib['x'])
        y = float(text_elem.attrib['y'])
        font_size = float(text_elem.attrib['font-size'])
        
        text_path = TextPath((x, y), text, size=font_size, prop=font_properties)
        return text_path.to_polygons()


    def text_polygons_to_svg_path_data(self, polygons):
        path_data = []
        for polygon in polygons:
            path = 'M' + ' '.join([f'{x},{y}' for x, y in polygon])
            path_data.append(path)
        return path_data

    def svg_path_to_gcode(self, path_data):
        path = svg.parse_path(path_data)
        gcode_lines = []
        first_move = True
        for segment in path:
            if isinstance(segment, svg.Line):
                start = segment.start
                end = segment.end
                if first_move:
                    gcode_lines.append(f'G0 X{start.real / 1000:.5f} Y{start.imag / 1000:.5f}')
                    first_move = False
                gcode_lines.append(f'G1 X{end.real / 1000:.5f} Y{end.imag / 1000:.5f}')
            elif isinstance(segment, svg.CubicBezier):
                points = [segment.start, segment.control1, segment.control2, segment.end]
                for point in points:
                    if first_move:
                        gcode_lines.append(f'G0 X{point.real / 1000:.5f} Y{point.imag / 1000:.5f}')
                        first_move = False
                    gcode_lines.append(f'G1 X{point.real / 1000:.5f} Y{point.imag / 1000:.5f}')
            else:
                # Handle other segment types similarly
                start = segment.start
                end = segment.end
                if first_move:
                    gcode_lines.append(f'G0 X{start.real / 1000:.5f} Y{start.imag / 1000:.5f}')
                    first_move = False
                gcode_lines.append(f'G1 X{end.real / 1000:.5f} Y{end.imag / 1000:.5f}')
        return gcode_lines


    def convert_svg_text_to_gcode(self, svg_file):
        text_elements, root, ns = self.parse_svg_for_text(svg_file)
        
        font_properties = FontProperties(family="Helvetica", weight="light")

        for i, text_elem in enumerate(text_elements):
            polygons = self.text_to_path(text_elem, font_properties)

            if i == 0: # Username
                offset = -55
            else: # Hashtag 
                offset = 55

            # Get bounding box and calculate the center
            min_x, max_x, min_y, max_y = self.get_bounding_box(polygons)
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2 + offset

            # Translate polygons to origin
            translated_polygons = self.translate_polygons_to_origin(polygons, center_x, center_y)
            
            path_data_list = self.text_polygons_to_svg_path_data(translated_polygons)

            gcode = []
            for path_data in path_data_list:
                gcode.extend(self.svg_path_to_gcode(path_data))

            if i == 0:
                output_gcode_file = os.path.join(self.home_directory, "rs2_ws", "gcode", "username.gcode")
                self.save_gcode_to_file(gcode, output_gcode_file)
                print(f"G-code for text element {i+1} successfully generated and saved to {output_gcode_file}.")
            else:
                output_gcode_file = os.path.join(self.home_directory, "rs2_ws", "gcode", "hashtag.gcode")
                self.save_gcode_to_file(gcode, output_gcode_file)
                print(f"G-code for text element {i+1} successfully generated and saved to {output_gcode_file}.")




    def save_gcode_to_file(self, gcode, output_file):
        with open(output_file, 'w') as f:
            for line in gcode:
                f.write(line + '\n')

    #--------------------------- END TEXT TO GCODE
    def get_bounding_box(self, polygons):
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')
        for polygon in polygons:
            for x, y in polygon:
                min_x = min(min_x, x)
                max_x = max(max_x, x)
                min_y = min(min_y, y)
                max_y = max(max_y, y)
        return min_x, max_x, min_y, max_y


    def translate_polygons_to_origin(self, polygons, dx, dy):
        translated_polygons = []
        for polygon in polygons:
            translated_polygons.append([(x - dx, y - dy) for x, y in polygon])
        return translated_polygons


    def generate_gcode_text(self):
        svg_path = os.path.join(self.home_directory, "rs2_ws", "img", "outline_picture_rmbg.svg")
        self.convert_svg_text_to_gcode(svg_path)

    # ------------------------------------------------

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
                    z = 0.145 + 9.2/1000 # 0.16
                    pose_goal_positions.append([x,y,z])

                elif line.startswith('G1'):
                    # Extract X,Y coordinates from the gcode file
                    x = float(line.split('X')[1].split(' ')[0])
                    y = float(line.split('Y')[1].split(' ')[0])
                    if robot_type == "ur3e":
                          z = 0.138  + 9.2/1000    # 0.135 ur3: 0.125 #ur3e: 0.13 # go up
                    else: z = 0.1285 + 9.2/1000    # 0.1285 go down
                    pose_goal_positions.append([x,y,z])

        return pose_goal_positions
    






# BACK UP 
    # def convert_svg_text_to_gcode(self, svg_file): JUST FOR 1 TEXT GCODE FILE
    #     y_offset = 0 # OFFSET TEXT HERE 55

    #     text_elements, root, ns = self.parse_svg_for_text(svg_file)
        
    #     font_properties = FontProperties(family="Helvetica", weight="light")
    #     all_polygons = []
    #     for text_elem in text_elements:
    #         polygons = self.text_to_path(text_elem, font_properties)
    #         all_polygons.extend(polygons)

    #     # Get bounding box and calculate the center
    #     min_x, max_x, min_y, max_y = self.get_bounding_box(all_polygons)
    #     center_x = (min_x + max_x) / 2
    #     center_y = (min_y + max_y) / 2

    #     # Translate polygons to origin
    #     translated_polygons = self.translate_polygons_to_origin(all_polygons, center_x, center_y - y_offset)
        
    #     path_data_list = self.text_polygons_to_svg_path_data(translated_polygons)

    #     gcode = []
    #     for path_data in path_data_list:
    #         gcode.extend(self.svg_path_to_gcode(path_data))

    #     return gcode