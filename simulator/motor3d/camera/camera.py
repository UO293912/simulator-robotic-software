"""
Camera — Cámara virtual esférica para renderizado 3D.
"""
import math
import numpy as np


class Camera:

    DEFAULT_YAW = 45.0
    DEFAULT_PITCH = 30.0
    DEFAULT_DISTANCE = 700.0
    DEFAULT_FOCAL = 300.0
    DEFAULT_ZOOM = 1.0
    PITCH_MIN = -89.0
    PITCH_MAX = 89.0
    ZOOM_MIN = 0.01
    ZOOM_MAX = 100.0
    KEYBOARD_SPEED = 2.0  # grados por tick

    def __init__(self):
        self.yaw = self.DEFAULT_YAW
        self.pitch = self.DEFAULT_PITCH
        self.distance = self.DEFAULT_DISTANCE
        self.focal_length = self.DEFAULT_FOCAL
        self.zoom = self.DEFAULT_ZOOM
        self.screen_offset_x = 0.0
        self.screen_offset_y = 0.0

    # ------------------------------------------------------------------
    # Configuración
    # ------------------------------------------------------------------

    def set_zoom_from_scale(self, scale):
        """Convierte un porcentaje de escala en una distancia orbital real.

        A diferencia del zoom optico, aqui el porcentaje representa un dolly:
        200% acerca la camara a la mitad de distancia; 50% la aleja al doble.
        """
        if scale and scale > 0:
            zoom_factor = scale / 100.0
            zoom_factor = max(self.ZOOM_MIN, min(self.ZOOM_MAX, zoom_factor))
            self.distance = self.DEFAULT_DISTANCE / zoom_factor
        self._sync_zoom_from_distance()

    def set_distance(self, distance):
        """Fija la distancia orbital y actualiza el porcentaje de zoom mostrado."""
        min_distance = self.DEFAULT_DISTANCE / self.ZOOM_MAX
        max_distance = self.DEFAULT_DISTANCE / self.ZOOM_MIN
        self.distance = max(min_distance, min(max_distance, float(distance)))
        self._sync_zoom_from_distance()

    def set_orientation(self, yaw=None, pitch=None):
        if yaw is not None:
            self.yaw = float(yaw) % 360.0
        if pitch is not None:
            self.pitch = max(self.PITCH_MIN, min(self.PITCH_MAX, float(pitch)))

    def pan(self, dx, dy):
        self.screen_offset_x += dx
        self.screen_offset_y += dy

    def reset(self):
        self.yaw = self.DEFAULT_YAW
        self.pitch = self.DEFAULT_PITCH
        self.screen_offset_x = 0.0
        self.screen_offset_y = 0.0
        self.distance = self.DEFAULT_DISTANCE
        self.zoom = self.DEFAULT_ZOOM

    def keyboard_update(self, move_wasd):
        if not move_wasd:
            return
        if move_wasd.get('a') or move_wasd.get('A'):
            self.yaw = (self.yaw - self.KEYBOARD_SPEED) % 360.0
        if move_wasd.get('d') or move_wasd.get('D'):
            self.yaw = (self.yaw + self.KEYBOARD_SPEED) % 360.0
        if move_wasd.get('w') or move_wasd.get('W'):
            self.pitch = min(self.PITCH_MAX, self.pitch + self.KEYBOARD_SPEED)
        if move_wasd.get('s') or move_wasd.get('S'):
            self.pitch = max(self.PITCH_MIN, self.pitch - self.KEYBOARD_SPEED)

    # ------------------------------------------------------------------
    # Transformación geométrica
    # ------------------------------------------------------------------

    def get_position(self):
        """Posición de la cámara en coordenadas del mundo."""
        yaw_r = math.radians(self.yaw)
        pitch_r = math.radians(self.pitch)
        x = self.distance * math.cos(pitch_r) * math.sin(yaw_r)
        y = self.distance * math.cos(pitch_r) * math.cos(yaw_r)
        z = self.distance * math.sin(pitch_r)
        return np.array([x, y, z], dtype=float)

    def get_view_matrix(self):
        """
        Construye la matriz de vista (lookAt desde la posición de la cámara
        mirando al origen del mundo).
        """
        cam_pos = self.get_position()
        target = np.zeros(3)
        world_up = np.array([0.0, 0.0, 1.0])

        forward = target - cam_pos
        norm = np.linalg.norm(forward)
        if norm < 1e-9:
            forward = np.array([0.0, 0.0, -1.0])
        else:
            forward = forward / norm

        right = np.cross(forward, world_up)
        r_norm = np.linalg.norm(right)
        if r_norm < 1e-9:
            right = np.array([1.0, 0.0, 0.0])
        else:
            right = right / r_norm

        up = np.cross(right, forward)
        up = up / np.linalg.norm(up)

        # Matriz de rotación (view matrix 3x3)
        # Convención: R[2] = forward (hacia el target) → z_cam > 0 para puntos
        # delante de la cámara, compatible con el chequeo "z <= 0.01 → detrás".
        R = np.array([
            [right[0],    right[1],    right[2]  ],
            [up[0],       up[1],       up[2]     ],
            [forward[0],  forward[1],  forward[2]],
        ], dtype=float)
        return R, cam_pos

    def camera_space(self, point):
        """
        Transforma un punto 3D al espacio de cámara.
        Retorna [x_cam, y_cam, z_cam].
        """
        R, cam_pos = self.get_view_matrix()
        p = np.array(point, dtype=float) - cam_pos
        return R @ p

    def project(self, point, width, height):
        """
        Proyecta un punto 3D a coordenadas de pantalla 2D.
        Retorna (sx, sy) o None si está detrás de la cámara.
        """
        p_cs = self.camera_space(point)
        z = p_cs[2]
        if z <= 0.01:
            return None
        f = self.focal_length
        sx = (p_cs[0] / z) * f + width / 2.0 + self.screen_offset_x
        sy = (-p_cs[1] / z) * f + height / 2.0 + self.screen_offset_y
        return sx, sy

    def _sync_zoom_from_distance(self):
        if self.distance <= 1e-9:
            self.zoom = self.ZOOM_MAX
            return
        self.zoom = self.DEFAULT_DISTANCE / self.distance
        self.zoom = max(self.ZOOM_MIN, min(self.ZOOM_MAX, self.zoom))
