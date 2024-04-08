class Interface:

    """
    Classes which inherit from the abstract Interface class provide a consistent interface_class for the gcode parser.

    The abstract methods below are necessary for the gcode parser to function. Some child classes may choose to also
    implement additional methods like specify_unit and home_axis to provide additional functionality to the parser.

    :param self.position stores the current tool position in 2d
    """

    # Todo convert to abc class
    # Todo add requirement self.position

    def set_movement_speed(self, speed) -> str:
        """
        Changes the speed at which the tool moves.

        :return: Appropriate command.
        """
        raise NotImplementedError("Interface class must implement the set_speed command")

    def linear_move(self, x=None, y=None, z=None) -> str:
        """
        Moves the tool in a straight line.

        :return: Appropriate command.
        """
        raise NotImplementedError("Interface class must implement the linear_move command")

    def pen_up(self):
        """
        Moves the pen up.

        :return: Appropriate command.
        """
        raise NotImplementedError("Interface class must implement the pen_up command")

    def pen_down(self):
        """
        Moves the pen down.

        :return: Appropriate command.
        """
        raise NotImplementedError("Interface class must implement the pen_down command")

    def set_absolute_coordinates(self) -> str:
        """
        Make the coordinate space absolute. ie. move relative to origin not current position.

        return '' if the target of the interface only supports absolute space. If the target only supports
        relative coordinate space, this command should return '' and the child class must transform all future inputs from
        absolute positions to relative positions until set_relative_coordinates is called.

        :return: Appropriate command.
        """
        raise NotImplementedError("Interface class must implement the set_absolute_coordinates command")

    def set_relative_coordinates(self) -> str:
        """
        Make the coordinate space relative. ie. move relative to current position not origin.

        return '' if the target of the interface only supports relative space. If the target only supports
        absolute coordinate space, this command should return '' and the child class must transform all future inputs from
        relative positions to absolute positions until set_absolute_coordinates is called.

        :return: Appropriate command.
        """
        raise NotImplementedError("Interface class must implement the set_relative_coordinates command")

    # Optional commands #
    def dwell(self, milliseconds) -> str:
        """
        Optional method, if implemented dwells for a determined number of milliseconds before moving to the next command.

        :return: Appropriate command.
        """
        pass

    def set_origin_at_position(self) -> str:
        """
        Optional method, if implemented translates coordinate space such that the current position is the new origin.
        If the target of the interface does not implement this command, return '' and the child class must translate all
        input positions to the new coordinate space.

        :return: Appropriate command.
        """
        pass

    def set_unit(self, unit):
        """
        Optional method, if implemented Specifies the unit of measurement.

        :return: Appropriate command. If not implemented return ''.
        """
        pass

    def home_axes(self):
        """
        Optional method, if implemented homes all axes.

        :return: Appropriate command. If not implemented return ''.
        """
        pass
