"""
Librería Braccio para el simulador S4R.
Simula la librería Arduino TinkerKit Braccio.

Pines del Braccio:
    base         → pin 11
    shoulder     → pin 10
    elbow        → pin 9
    wrist_ver    → pin 6
    wrist_rot    → pin 5
    gripper      → pin 3
"""

# Pines estándar del Braccio
BRACCIO_PINS = {
    'base': 11,
    'shoulder': 10,
    'elbow': 9,
    'wrist_ver': 6,
    'wrist_rot': 5,
    'gripper': 3,
}

# Orden de los servos en servo_movement
_SERVO_ORDER = ['base', 'shoulder', 'elbow', 'wrist_ver', 'wrist_rot', 'gripper']


def get_name():
    return "Braccio"


def get_methods():
    """
    Retorna los métodos de la librería Braccio como diccionario.
    Formato: ("tipo_retorno", "nombre_python", [tipos_args], -1)
    """
    methods = {}
    methods["begin"] = ("void", "begin", [], -1)
    methods["ServoMovement"] = (
        "int", "servo_movement",
        ['int', 'int', 'int', 'int', 'int', 'int', 'int'],
        -1
    )
    return methods


def get_not_implemented():
    return []


class Braccio:
    """
    Clase que simula la librería Braccio de Arduino.
    Permite controlar los 6 servomotores del brazo TinkerKit Braccio.
    """

    OK = 1
    ERROR = 0

    def __init__(self, board=None):
        self.board = board
        self._servos = {}  # pin → elemento servo

    def set_board(self, board):
        """Establece la placa y resuelve los elementos servo por pin."""
        self.board = board
        self._resolve_servos()

    def _resolve_servos(self):
        """Busca los elementos servo en la placa por sus pines estándar."""
        self._servos = {}
        if self.board is None:
            return
        for name, pin in BRACCIO_PINS.items():
            elem = self.board.get_pin_element(pin)
            if elem is not None:
                self._servos[pin] = elem

    def begin(self):
        """
        Inicializa el Braccio.
        En el simulador, resuelve los servos y mueve al estado de reposo.
        """
        self._resolve_servos()
        # Posición de reposo estándar
        self.servo_movement(20, 90, 90, 90, 90, 90, 73)
        return self.OK

    def servo_movement(self, step_delay, v_base, v_shoulder, v_elbow,
                       v_wrist_ver, v_wrist_rot, v_gripper):
        """
        Mueve todos los servos del Braccio a las posiciones indicadas.

        Args:
            step_delay : retardo entre pasos (ignorado en simulador)
            v_base     : ángulo servo base (0-180°)
            v_shoulder : ángulo servo hombro (15-165°)
            v_elbow    : ángulo servo codo (0-180°)
            v_wrist_ver: ángulo servo muñeca vertical (0-180°)
            v_wrist_rot: ángulo servo muñeca rotación (0-180°)
            v_gripper  : ángulo servo pinza (10-73°)

        Returns:
            OK (1) si la operación fue correcta, ERROR (0) si hubo fallo.
        """
        if not self._servos and self.board is not None:
            self._resolve_servos()

        values = [v_base, v_shoulder, v_elbow, v_wrist_ver, v_wrist_rot, v_gripper]
        names = _SERVO_ORDER

        for name, value in zip(names, values):
            pin = BRACCIO_PINS[name]
            # Intentar escribir directamente al elemento servo
            elem = self._servos.get(pin)
            if elem is not None:
                elem.set_value(pin, int(value))
            elif self.board is not None:
                # Fallback: usar write_value del board
                self.board.write_value(pin, int(value))

        return self.OK
