from xml.etree import ElementTree
from typing import List
from copy import deepcopy

from svg_to_gcode.svg_parser import Path, Transformation
from svg_to_gcode.geometry import Curve

NAMESPACES = {'svg': 'http://www.w3.org/2000/svg'}


def _has_style(element: ElementTree.Element, key: str, value: str) -> bool:
    """
    Check if an element contains a specific key and value either as an independent attribute or in the style attribute.
    """
    return element.get(key) == value or (element.get("style") and f"{key}:{value}" in element.get("style"))


def parse_root(root: ElementTree.Element, transform_origin=True, canvas_height=None, draw_hidden=False,
               visible_root=True, root_transformation=None) -> List[Curve]:
    """ Recursively parse an etree root's children into geometric curves. """
    if canvas_height is None:
        height_str = root.get("height", '100px').replace('px', '')
        canvas_height = float(height_str)

    curves = []

    for element in root.findall('.//*'):
        tag = element.tag.split('}')[-1]  # Handling namespace by splitting off the URI

        if tag == 'defs' or _has_style(element, "display", "none"):
            continue

        transform = element.get('transform')
        transformation = deepcopy(root_transformation) if root_transformation else Transformation()
        if transform:
            transformation.add_transform(transform)

        visible = not any([_has_style(element, "visibility", "hidden"),
                           _has_style(element, "visibility", "collapse")]) or _has_style(element, "visibility", "visible")

        if draw_hidden or visible:
            if tag == 'path':
                path_d = element.get('d')
                if path_d:
                    path = Path(path_d, canvas_height, transform_origin, transformation)
                    curves.extend(path.curves)

    return curves


def parse_string(svg_string: str, transform_origin=True, canvas_height=None, draw_hidden=False) -> List[Curve]:
    """
        Recursively parse an svg string into geometric curves. (Wrapper for parse_root)

        :param svg_string: The etree element who's children should be recursively parsed. The root will not be drawn.
        :param canvas_height: The height of the canvas. By default the height attribute of the root is used. If the root
        does not contain the height attribute, it must be either manually specified or transform_origin must be False.
        :param transform_origin: Whether or not to transform input coordinates from the svg coordinate system to standard cartesian
         system. Depends on canvas_height for calculations.
        :param draw_hidden: Whether or not to draw hidden elements based on their display, visibility and opacity attributes.
        :return: A list of geometric curves describing the svg. Use the Compiler sub-module to compile them to gcode.
    """
    root = ElementTree.fromstring(svg_string)
    return parse_root(root, transform_origin, canvas_height, draw_hidden)


def parse_file(file_path: str, transform_origin=True, canvas_height=None, draw_hidden=False) -> List[Curve]:
    """
            Recursively parse an svg file into geometric curves. (Wrapper for parse_root)

            :param file_path: The etree element who's children should be recursively parsed. The root will not be drawn.
            :param canvas_height: The height of the canvas. By default the height attribute of the root is used. If the root
            does not contain the height attribute, it must be either manually specified or transform_origin must be False.
            :param transform_origin: Whether or not to transform input coordinates from the svg coordinate system to standard cartesian
             system. Depends on canvas_height for calculations.
            :param draw_hidden: Whether or not to draw hidden elements based on their display, visibility and opacity attributes.
            :return: A list of geometric curves describing the svg. Use the Compiler sub-module to compile them to gcode.
        """
    tree = ElementTree.parse(file_path)
    root = tree.getroot()
    svg_root = convert_polyline_to_path(root)
    
    return parse_root(svg_root, transform_origin, canvas_height, draw_hidden)


def find_parent(root, child):
    for parent in root.iter():
        for elem in parent:
            if elem == child:
                return parent
    return None  # if no parent found

def convert_polyline_to_path(svg_root):
    ns = {'svg': 'http://www.w3.org/2000/svg'}
    polylines = list(svg_root.findall('.//svg:polyline', namespaces=ns))
    for polyline in polylines:
        points = polyline.get('points').strip()
        points_list = points.split()
        path_d = "M" + " L".join(points_list) + " z"

        path = ElementTree.Element('path')
        path.set('d', path_d)
        if polyline.get('style'):
            path.set('style', polyline.get('style'))
        if polyline.get('stroke'):
            path.set('stroke', polyline.get('stroke'))
        if polyline.get('fill'):
            path.set('fill', polyline.get('fill', 'none'))  # Default to none if not specified

        # Use the helper function to find the parent
        parent = find_parent(svg_root, polyline)
        if parent is not None:
            parent.insert(list(parent).index(polyline), path)
            parent.remove(polyline)

    return svg_root