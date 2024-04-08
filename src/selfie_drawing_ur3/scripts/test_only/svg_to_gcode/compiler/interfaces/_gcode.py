import warnings
import math

from svg_to_gcode import formulas
from svg_to_gcode.compiler.interfaces import Interface
from svg_to_gcode.geometry import Vector
from svg_to_gcode import TOLERANCES

verbose = False


class Gcode(Interface):

    def __init__(self):
        self.position = None
        self.pen_up_height = 5  # Adjust this value as needed for your machine
        self.pen_down_height = - self.pen_up_height  # Adjust this value as needed for your machine

        # Round outputs to the same number of significant figures as the operational tolerance.
        self.precision = abs(round(math.log(TOLERANCES["operation"], 10)))

    def relative_linear_draw(self, x=None, y=None, z=None):
        # Don't do anything if linear move was called without passing a value.
        if x is None and y is None and z is None:
            warnings.warn("relative_linear_move command invoked without arguments.")
            return ''

        command = "G1"

        # Calculate deltas and build command
        if x is not None:
            delta_x = x - self.position.x if self.position is not None else x
            command += f" X{delta_x:.{self.precision}f}"
        if y is not None:
            delta_y = y - self.position.y if self.position is not None else y
            command += f" Y{delta_y:.{self.precision}f}"
        if z is not None:
            delta_z = z - self.position.z if self.position is not None else z
            command += f" Z{delta_z:.{self.precision}f}"

        # Update current position
        if self.position is not None:
            new_x = self.position.x + delta_x if x is not None else self.position.x
            new_y = self.position.y + delta_y if y is not None else self.position.y
            
        # Ensure we update position even if only one axis is provided
        self.position = Vector(new_x, new_y) if self.position is not None else Vector(x or 0, y or 0)

        if verbose:
            print(f"Relative move by {delta_x if x is not None else 0}, {delta_y if y is not None else 0}, {delta_z if z is not None else 0}")

        return command + ';'
    
    def relative_linear_move(self, x=None, y=None, z=None):
        # Don't do anything if linear move was called without passing a value.
        if x is None and y is None and z is None:
            warnings.warn("relative_linear_move command invoked without arguments.")
            return ''

        command = "G0"

        # Calculate deltas and build command
        if x is not None:
            delta_x = x - self.position.x if self.position is not None else x
            command += f" X{delta_x:.{self.precision}f}"
        if y is not None:
            delta_y = y - self.position.y if self.position is not None else y
            command += f" Y{delta_y:.{self.precision}f}"
        if z is not None:
            delta_z = z - self.position.z if self.position is not None else z
            command += f" Z{delta_z:.{self.precision}f}"

        # Update current position
        if self.position is not None:
            new_x = self.position.x + delta_x if x is not None else self.position.x
            new_y = self.position.y + delta_y if y is not None else self.position.y
            
        # Ensure we update position even if only one axis is provided
        self.position = Vector(new_x, new_y) if self.position is not None else Vector(x or 0, y or 0)

        if verbose:
            print(f"Relative move by {delta_x if x is not None else 0}, {delta_y if y is not None else 0}, {delta_z if z is not None else 0}")

        return command + ';'

    def pen_up(self):
        # Adjust the Z coordinate upwards to raise the pen
        return f"G0 Z{self.pen_up_height:.{self.precision}f};"

    def pen_down(self):
        # Adjust the Z coordinate downwards to lower the pen to the paper
        return f"G0 Z{self.pen_down_height:.{self.precision}f};"

    def set_absolute_coordinates(self):
        return "G90;"

    def set_relative_coordinates(self):
        return "G91;"

    def dwell(self, milliseconds):
        return f"G4 P{milliseconds}"

    def set_origin_at_position(self):
        self.position = Vector(0, 0)
        return "G92 X0 Y0 Z0;"

    def set_unit(self, unit):
        if unit == "mm":
            return "G21;"

        if unit == "in":
            return "G20;"

        return ''

    def home_axes(self):
        return "G28;"
