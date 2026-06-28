import robot_components.boards as boards


def get_name():
    return "Servo"


def get_methods():
    """
    Returns the methods of the library as a dict, whose
    key is the naming in Arduino and whose value is the
    corresponding method.
    Returns:
        A dict with the methods
    """
    methods = {}
    methods["attach"] = ("void", "attach", ['int', '(int)', '(int)'], -1)
    methods["write"] = ("void", "write", ['int'], -1)
    methods["writeMicroseconds"] = ("void", "write_microseconds", ['int'], -1)
    methods["read"] = ("int", "read", [], -1)
    methods["attached"] = ("bool", "attached", [], -1)
    methods["detach"] = ("void", "detach", [], -1)
    return methods


def get_not_implemented():
    return []


class Servo:
    """
    Servo class, represents the movement of a real servo
    """

    OK = 0
    ERROR = -1
    NOT_IMPL_WARNING = -2

    def __init__(self, board=None, name=None):
        """
        Constructor for Servo class
        """
        self.board = board
        self.servo = None
        self.instance_name = name
        self.min = 544
        self.max = 2400
        self.speed = 90

    def set_board(self, board: boards.Board):
        """
        Sets the board that the robot is using
        """
        self.board = board

    def attach(self, pin, min=544, max=2400):
        """
        Attaches the servo to a pin
        Arguments:
            servo: the servo to attach
            pin: the number of the pin to be attached to
            min: pulse width corresponging with the minimun angle on
            the servo (default = 544)
            max: pulse width corresponging with the max angel on the
            the servo (default = 2400)
        Returns:
            OK if servo attached to pin correctly, ERROR if else
        """
        servo = None
        if self.board is not None:
            servo = self.board.get_pin_element(pin)
            if servo is None:
                arm_robot = getattr(self.board, "arm_robot", None)
                if arm_robot is not None:
                    servo = arm_robot.attach_servo_to_pin(pin, self.instance_name)
        if servo is not None:
            self.servo = servo
            self.min = min
            self.max = max
            servo.min = min
            servo.max = max
            return self.OK
        return self.ERROR

    def write(self, angle):
        """
        Writes speed to servo.
        Our Servos, being rotation ones, will have their speed set by this
        method. If the angle is 0, the speed is full in one direction, if
        it is 180, is full speed in the oposite direction; with 90 being no
        movement done by the servo
        Arguments:
            servo: the servo to write to
            angle: the value to write [0-180]
        """
        if self.servo is not None:
            try:
                clamped = max(0, min(180, int(float(angle))))
            except (TypeError, ValueError):
                return
            self.servo.set_value(self.servo.pin, clamped)

    def write_microseconds(self, us):
        """
        Writes a pulse width to the servo and converts it to the equivalent
        Servo.write angle using the attach(min, max) calibration.

        Arduino Servo clamps writeMicroseconds() to the attached pulse range;
        the simulator stores the equivalent 0..180 control value so the robot
        model can keep using the same servo path as write().
        Arguments:
            servo: the servo to write to
            us: the value of the parameter in microseconds (int)
        """
        if self.servo is not None:
            try:
                pulse = int(float(us))
            except (TypeError, ValueError):
                return self.ERROR

            pulse_min = int(getattr(self.servo, "min", self.min))
            pulse_max = int(getattr(self.servo, "max", self.max))
            if pulse_max <= pulse_min:
                return self.ERROR

            clamped = max(pulse_min, min(pulse_max, pulse))
            angle = int((clamped - pulse_min) * 180 / (pulse_max - pulse_min))
            if hasattr(self.servo, "set_value"):
                self.servo.set_value(self.servo.pin, angle)
            else:
                self.servo.value = angle
            return self.OK
        return self.ERROR

    def read(self):
        """
        Reads the angle of the servo (being the last value passed to write)
        Arguments:
            The servo to read from
        Returns:
            The angle of the servo from 0 to 180 degrees
        """
        if self.servo is not None:
            return self.servo.value
        return None

    def attached(self):
        """
        Checks wether the Servo variable is attached or not
        Arguments:
            servo: the servo to check
        Returns:
            True if attached to pin, False if else
        """
        if self.servo is not None:
            return self.servo.pin != -1
        return False

    def detach(self):
        """
        Detach the Servo variable from its pin
        Arguments:
            servo: the servo to detach
        """
        if self.servo is not None:
            arm_robot = getattr(self.board, "arm_robot", None) if self.board is not None else None
            if arm_robot is not None:
                arm_robot.detach_servo(self.servo)
            elif self.board is not None and self.servo.pin != -1:
                self.board.detach_pin(self.servo.pin)
                self.servo.pin = -1
            self.servo = None
            return self.OK
        return self.ERROR
