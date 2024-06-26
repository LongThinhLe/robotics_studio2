import os
import xml.etree.ElementTree as ET
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties
import svgpathtools

def parse_svg_for_text(svg_file):
    tree = ET.parse(svg_file)
    root = tree.getroot()
    text_elements = []
    ns = {'svg': 'http://www.w3.org/2000/svg'}
    for elem in root.findall('.//{http://www.w3.org/2000/svg}text', ns):
        text_elements.append(elem)
    return text_elements, root, ns

def text_to_path(text_elem, font_properties):
    text = text_elem.text
    x = float(text_elem.attrib['x'])
    y = float(text_elem.attrib['y'])
    font_size = float(text_elem.attrib['font-size'])
    
    text_path = TextPath((x, y), text, size=font_size, prop=font_properties)
    return text_path.to_polygons()

def text_polygons_to_svg_path_data(polygons):
    path_data = []
    for polygon in polygons:
        path = 'M' + ' '.join([f'{x},{y}' for x, y in polygon])
        path_data.append(path)
    return path_data

def svg_path_to_gcode(path_data):
    path = svgpathtools.parse_path(path_data)
    gcode_lines = []
    for segment in path:
        if isinstance(segment, svgpathtools.Line):
            start = segment.start
            end = segment.end
            gcode_lines.append(f'G0 X{start.real / 1000:.5f} Y{start.imag / 1000:.5f}')
            gcode_lines.append(f'G1 X{end.real / 1000:.5f} Y{end.imag / 1000:.5f}')
        elif isinstance(segment, svgpathtools.CubicBezier):
            points = [segment.start, segment.control1, segment.control2, segment.end]
            for point in points:
                gcode_lines.append(f'G1 X{point.real:.2f} Y{point.imag:.2f}')
        # Handle other segment types similarly
    return gcode_lines

def convert_svg_text_to_gcode(svg_file):
    text_elements, root, ns = parse_svg_for_text(svg_file)
    
    font_properties = FontProperties(family="Arial", weight="bold")
    path_data_list = []
    for text_elem in text_elements:
        polygons = text_to_path(text_elem, font_properties)
        path_data_list.extend(text_polygons_to_svg_path_data(polygons))

    gcode = []
    for path_data in path_data_list:
        gcode.extend(svg_path_to_gcode(path_data))

    return gcode

def save_gcode_to_file(gcode, output_file):
    with open(output_file, 'w') as f:
        for line in gcode:
            f.write(line + '\n')

# Usage
svg_file = '/home/lelongthinh/rs2_ws/img/outline_picture_rmbg.svg'
gcode = convert_svg_text_to_gcode(svg_file)

if not gcode:
    print("No G-code generated from the SVG text elements.")
else:
    output_gcode_file = '/home/lelongthinh/rs2_ws/gcode/text_elements.gcode'
    save_gcode_to_file(gcode, output_gcode_file)
    print(f"G-code successfully generated and saved to {output_gcode_file}.")
