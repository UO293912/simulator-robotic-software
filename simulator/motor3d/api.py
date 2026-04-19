"""
Motor3DApi — Fachada principal del módulo de simulación 3D.
Arm3DLayer (en graphics) solo llama métodos de esta clase.
"""
import os
from pathlib import Path

from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics
from motor3d.kinematics.constraints_limits import clamp_model_joints
from motor3d.camera.camera import Camera
from motor3d.camera.camera_controller import CameraController
from motor3d.rendering.robot3d_drawing import Robot3DDrawing
from motor3d.rendering.scene3d import Scene3D
from motor3d.persistence.arm_config_repository import ArmConfigRepository
from motor3d.safety.safety_manager import SafetyManager


class Motor3DApi:

    DEFAULT_PRESET = 'braccio_tinkerkit'

    def __init__(self):
        self.model = ArmKinematicState()
        self.camera = Camera()

        # Rutas
        _assets_dir = Path(os.path.dirname(__file__)) / '..' / 'assets'
        _assets_dir = _assets_dir.resolve()
        _stl_dir = _assets_dir / 'stl'
        _presets_dir = _assets_dir / 'presets'
        _autosave = _assets_dir / 'arm3d_last_config.json'

        self.autosave_path = _autosave
        self.default_preset_name = self.DEFAULT_PRESET
        self.active_preset_name = self.DEFAULT_PRESET

        self.repository = ArmConfigRepository(
            default_path=_autosave,
            presets_dir=_presets_dir,
        )
        self.camera_controller = CameraController(self.camera)
        self.safety_manager = SafetyManager()

        renderer = Robot3DDrawing(stl_dir=str(_stl_dir))
        self.scene = Scene3D(self.model, self.camera, renderer)
        self.renderer = renderer

        # Cargar preset por defecto
        loaded = self.repository.load_model(self.model, silent=True)
        if not loaded:
            self.repository.load_builtin_preset(self.model, self.DEFAULT_PRESET, silent=True)
        self._sync_active_preset_name()
        if self.model.dof == 0:
            self._load_fallback_config()

        self.scene.update()

    # ------------------------------------------------------------------
    # Cámara
    # ------------------------------------------------------------------

    def set_zoom_from_scale(self, scale):
        self.camera.set_zoom_from_scale(scale)

    def keyboard_camera(self, move_wasd):
        self.camera_controller.keyboard_update(move_wasd)

    def pan_camera(self, dx, dy):
        self.camera_controller.pan(dx, dy)

    def drag_camera(self, dx, dy, pan=False):
        self.camera_controller.drag(dx, dy, pan=pan)

    def set_camera(self, yaw=None, pitch=None):
        self.camera.set_orientation(yaw=yaw, pitch=pitch)

    def reset_camera(self):
        self.camera.reset()

    # ------------------------------------------------------------------
    # Articulaciones
    # ------------------------------------------------------------------

    def set_joint(self, index, value):
        """Fija el ángulo articular con clamping por límites y actualiza FK."""
        self.model.set_joint(index, value)
        self.scene.update()

    # ------------------------------------------------------------------
    # Trayectoria
    # ------------------------------------------------------------------

    def set_show_trail(self, show):
        self.scene.set_show_trail(show)

    def set_show_joint_ranges(self, show):
        if show:
            self.model.visual['show_joint_ranges'] = True
        else:
            self.model.visual.pop('show_joint_ranges', None)

    def set_show_joint_axes(self, show):
        if show:
            self.model.visual['show_joint_axes'] = True
        else:
            self.model.visual.pop('show_joint_axes', None)

    # ------------------------------------------------------------------
    # Cinemática Inversa
    # ------------------------------------------------------------------

    def solve_ik(self, x, y, z):
        """
        Lanza la cinemática inversa hacia la posición (x, y, z).

        Returns:
            (converged: bool, message: str)
        """
        converged, error = solve_inverse_kinematics(
            self.model, [x, y, z], max_iter=150, tolerance=25.0, alpha=0.65
        )
        self.scene.update()
        if converged:
            return True, f"IK convergida (error={error:.1f} mm)"
        else:
            return False, f"IK no convergida (error={error:.1f} mm) — posición fuera del workspace"

    # ------------------------------------------------------------------
    # Configuración
    # ------------------------------------------------------------------

    def get_model_config(self):
        self.model.preset_name = self.active_preset_name
        return self.model.to_dict()

    def set_model_config(self, config):
        self.model.load_dict(config)
        self._sync_active_preset_name()
        self.renderer._canvas_image_id = None
        self.scene.clear_trail()
        self.scene.update()

    def save_model_config(self, path=None):
        self.model.preset_name = self.active_preset_name
        return self.repository.save_model(self.model, path=path or self.autosave_path)

    def load_model_config(self, path=None):
        ok = self.repository.load_model(self.model, path=path or self.autosave_path)
        if ok:
            self._sync_active_preset_name()
            self.scene.clear_trail()
            self.scene.update()
        return ok

    def uses_legacy_servo_degrees(self):
        """True solo para el Braccio predefinido con numeraciÃ³n servo 0..180."""
        return (
            self.model.visual.get('mode') == 'braccio_exact'
            and self.active_preset_name == self.DEFAULT_PRESET
        )

    # ------------------------------------------------------------------
    # Renderizado
    # ------------------------------------------------------------------

    def draw(self, canvas):
        """Renderiza la escena completa en el canvas Tkinter."""
        self.scene.draw(canvas)

    # ------------------------------------------------------------------
    # Evaluación de seguridad (usado por Arm3DLayer)
    # ------------------------------------------------------------------

    def evaluate_safety(self):
        """Evalúa el estado de seguridad de la pose actual."""
        points = self.scene.last_points
        max_reach = self.model.max_reach()
        return self.safety_manager.evaluate(points, max_reach)


    # ------------------------------------------------------------------
    # Helpers privados
    # ------------------------------------------------------------------

    def _load_fallback_config(self):
        """Configuración mínima si no hay preset disponible."""
        self.model.configure(
            dof=3,
            link_lengths=[200.0, 200.0, 120.0],
            joint_limits=[(-90.0, 90.0)] * 3,
            joint_types=['R', 'R', 'R'],
            joints=[0.0, 0.0, 0.0],
            dh_rows=[
                {'theta': 0.0, 'd': 212.0, 'a': 0.0, 'alpha': 90.0},
                {'theta': 0.0, 'd': 0.0, 'a': 200.0, 'alpha': 0.0},
                {'theta': 0.0, 'd': 0.0, 'a': 200.0, 'alpha': 0.0},
            ],
            visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        )
        self.active_preset_name = None
        self.model.preset_name = None

    def _sync_active_preset_name(self):
        preset_name = getattr(self.model, 'preset_name', None)
        if preset_name == self.DEFAULT_PRESET and self.model.visual.get('mode') == 'braccio_exact':
            self.active_preset_name = preset_name
            self.model.preset_name = preset_name
            return
        if self.model.visual.get('mode') == 'braccio_exact':
            # Compatibilidad con presets/autosaves antiguos sin preset_name persistido.
            self.active_preset_name = self.DEFAULT_PRESET
            self.model.preset_name = self.DEFAULT_PRESET
            return
        self.active_preset_name = None
        self.model.preset_name = None
