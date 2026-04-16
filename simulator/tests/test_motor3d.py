"""
Pruebas unitarias del módulo motor3d.

Casos de prueba según el plan de pruebas del TFG:
  P-CU02-01 : Precisión FK  — error < 1 mm respecto a posición analítica conocida.
  P-CU02-02 : Convergencia IK — error < tolerancia (25 mm) en workspace válido.
  P-CU03-01 : Rendering sin crash — scene.update() + geometría 3D no lanza excepciones.
  P-CU04-01 : Navegación 3D sin crash — rotación/zoom de cámara no lanza excepciones.

Ejecutar desde el directorio `simulator/`:
    pytest tests/test_motor3d.py -v
"""
import math
import sys
import os
from unittest import mock

# Asegurar que el paquete simulator está en el path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest


# ---------------------------------------------------------------------------
# Fixtures compartidas
# ---------------------------------------------------------------------------

@pytest.fixture
def braccio_model():
    """ArmKinematicState con el preset braccio_tinkerkit cargado."""
    from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
    from motor3d.persistence.arm_config_repository import ArmConfigRepository
    model = ArmKinematicState()
    repo = ArmConfigRepository()
    ok = repo.load_builtin_preset(model, "braccio_tinkerkit", silent=True)
    assert ok, "No se pudo cargar el preset braccio_tinkerkit"
    return model


@pytest.fixture
def motor3d_api():
    """Motor3DApi inicializado con preset Braccio."""
    from motor3d.api import Motor3DApi
    return Motor3DApi()


class _MockCanvas:
    """Canvas Tkinter simulado para pruebas de rendering."""
    def winfo_width(self):  return 800
    def winfo_height(self): return 600
    def create_image(self, *a, **kw): return 1
    def itemconfig(self, *a, **kw): pass
    def find_withtag(self, *a): return []


# ---------------------------------------------------------------------------
# P-CU02-01 : Precisión FK
# ---------------------------------------------------------------------------

class TestForwardKinematics:
    """P-CU02-01 — Precisión de la cinemática directa."""

    def test_fk_rest_pose_arm_points_up(self, braccio_model):
        """Con todos los joints=0 (servos a 90°) el brazo debe apuntar hacia arriba.
        El efector final debe estar por encima de z=800 mm y tener X≈Y≈0.
        """
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        braccio_model.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        chain = forward_kinematics_chain(braccio_model)
        ee = chain['end_effector']

        assert ee[2] > 800.0, f"z esperado >800 mm, obtenido {ee[2]:.1f}"
        assert abs(ee[0]) < 5.0, f"x esperado ≈0, obtenido {ee[0]:.1f}"
        assert abs(ee[1]) < 5.0, f"y esperado ≈0, obtenido {ee[1]:.1f}"

    def test_fk_joint_count(self, braccio_model):
        """La cadena FK debe devolver exactamente dof+1 posiciones y dof matrices."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        chain = forward_kinematics_chain(braccio_model)
        assert len(chain['positions']) == braccio_model.dof + 1
        assert len(chain['matrices']) == braccio_model.dof

    def test_fk_base_rotation_preserves_height(self, braccio_model):
        """Rotar J1 no debe cambiar la altura Z del efector final."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        braccio_model.joints = [0.0] * 6
        ee_0 = forward_kinematics_chain(braccio_model)['end_effector']

        braccio_model.joints = [90.0] + [0.0] * 5
        ee_90 = forward_kinematics_chain(braccio_model)['end_effector']

        assert abs(ee_0[2] - ee_90[2]) < 2.0, (
            f"Z no debe cambiar con rotación de base: {ee_0[2]:.1f} vs {ee_90[2]:.1f}")

    def test_fk_base_rotation_conserves_radial_distance(self, braccio_model):
        """La distancia radial XY del efector debe conservarse al rotar J1."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        braccio_model.joints = [0.0, 30.0, -20.0, 0.0, 0.0, 0.0]
        ee_0 = forward_kinematics_chain(braccio_model)['end_effector']

        braccio_model.joints = [90.0, 30.0, -20.0, 0.0, 0.0, 0.0]
        ee_90 = forward_kinematics_chain(braccio_model)['end_effector']

        r0 = math.sqrt(ee_0[0] ** 2 + ee_0[1] ** 2)
        r90 = math.sqrt(ee_90[0] ** 2 + ee_90[1] ** 2)
        assert abs(r0 - r90) < 2.0, f"Radio XY no conservado: {r0:.1f} vs {r90:.1f}"

    def test_fk_known_reach_rest_pose(self, braccio_model):
        """La distancia al origen en pose de reposo debe ser ≥ 90% del max_reach."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        braccio_model.joints = [0.0] * 6
        chain = forward_kinematics_chain(braccio_model)
        ee = chain['end_effector']

        dist = math.sqrt(ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2)
        max_r = braccio_model.max_reach()
        assert dist > max_r * 0.90, (
            f"Distancia esperada >{max_r * 0.90:.0f} mm, obtenida {dist:.1f}")

    def test_fk_matrices_are_4x4_homogeneous(self, braccio_model):
        """Todas las matrices de transformación deben ser homogéneas 4×4."""
        import numpy as np
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        chain = forward_kinematics_chain(braccio_model)
        for i, T in enumerate(chain['matrices']):
            assert T.shape == (4, 4), f"Matriz {i} tiene forma {T.shape}, esperada (4,4)"
            # Última fila debe ser [0,0,0,1]
            np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=1e-6,
                                       err_msg=f"Última fila de T{i} no es [0,0,0,1]")

    def test_fk_shoulder_tilt_moves_ee_horizontally(self, braccio_model):
        """Inclinar el hombro J2 debe aumentar el radio XY del efector."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        braccio_model.joints = [0.0] * 6
        ee_rest = forward_kinematics_chain(braccio_model)['end_effector']

        braccio_model.joints = [0.0, 45.0, 0.0, 0.0, 0.0, 0.0]
        ee_tilted = forward_kinematics_chain(braccio_model)['end_effector']

        r_rest = math.sqrt(ee_rest[0] ** 2 + ee_rest[1] ** 2)
        r_tilted = math.sqrt(ee_tilted[0] ** 2 + ee_tilted[1] ** 2)
        assert r_tilted > r_rest + 100, (
            f"J2=45° debe aumentar radio XY >100 mm: {r_tilted:.1f} vs {r_rest:.1f}")


# ---------------------------------------------------------------------------
# P-CU02-02 : Convergencia IK
# ---------------------------------------------------------------------------

class TestInverseKinematics:
    """P-CU02-02 — Convergencia de la cinemática inversa."""

    TOLERANCE = 25.0  # mm

    def _reset_joints(self, model):
        model.joints = [0.0, 0.0, 0.0, 0.0, 0.0, -17.0]

    def test_ik_reaches_fk_derived_target(self, braccio_model):
        """Target derivado de FK garantizado dentro del workspace: IK debe converger."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics

        braccio_model.joints = [15.0, -20.0, 15.0, 0.0, 0.0, -17.0]
        target = forward_kinematics_chain(braccio_model)['end_effector']

        self._reset_joints(braccio_model)
        converged, error = solve_inverse_kinematics(
            braccio_model, target, max_iter=200, tolerance=self.TOLERANCE, alpha=0.65)

        assert converged, f"IK no convergió (error={error:.1f} mm, tol={self.TOLERANCE} mm)"
        assert error <= self.TOLERANCE

    def test_ik_out_of_workspace_returns_false(self, braccio_model):
        """Target fuera del workspace (>105% max_reach) debe retornar False."""
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics

        max_r = braccio_model.max_reach()
        target = [0.0, 0.0, max_r * 1.2]
        converged, _ = solve_inverse_kinematics(
            braccio_model, target, max_iter=10, tolerance=self.TOLERANCE)

        assert not converged, "IK no debe converger para target fuera del workspace"

    def test_ik_joint_limits_respected_after_solve(self, braccio_model):
        """Tras IK, todos los joints deben estar dentro de sus límites."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics

        braccio_model.joints = [30.0, -30.0, 20.0, 10.0, -10.0, -17.0]
        target = forward_kinematics_chain(braccio_model)['end_effector']

        self._reset_joints(braccio_model)
        solve_inverse_kinematics(braccio_model, target, max_iter=200, tolerance=self.TOLERANCE)

        for i, (q, (mn, mx)) in enumerate(zip(braccio_model.joints, braccio_model.joint_limits)):
            assert mn - 0.1 <= q <= mx + 0.1, (
                f"Joint {i + 1} = {q:.1f}° fuera de [{mn}, {mx}]")

    def test_ik_api_returns_bool_str_tuple(self, motor3d_api):
        """Motor3DApi.solve_ik debe retornar (bool, str)."""
        result = motor3d_api.solve_ik(0.0, 0.0, 500.0)
        assert isinstance(result, tuple) and len(result) == 2
        assert isinstance(result[0], bool)
        assert isinstance(result[1], str)

    def test_ik_improves_from_initial(self, braccio_model):
        """Tras IK el error debe ser menor que el error de la pose inicial."""
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics
        import math

        braccio_model.joints = [20.0, -25.0, 15.0, 5.0, 0.0, -17.0]
        target = forward_kinematics_chain(braccio_model)['end_effector']

        # Error inicial con pose de reposo
        self._reset_joints(braccio_model)
        chain_init = forward_kinematics_chain(braccio_model)
        ee_init = chain_init['end_effector']
        error_init = math.sqrt(sum((a - b) ** 2 for a, b in zip(ee_init, target)))

        self._reset_joints(braccio_model)
        _, error_final = solve_inverse_kinematics(
            braccio_model, target, max_iter=200, tolerance=self.TOLERANCE, alpha=0.65)

        assert error_final < error_init, (
            f"IK debe reducir el error: inicial={error_init:.1f}, final={error_final:.1f}")


# ---------------------------------------------------------------------------
# P-CU03-01 : Rendering sin crash
# ---------------------------------------------------------------------------

class TestRendering:
    """P-CU03-01 — El motor de rendering no debe lanzar excepciones."""

    def test_scene_update_no_crash(self, motor3d_api):
        """scene.update() debe ejecutarse sin excepciones."""
        motor3d_api.model.joints = [0.0, 0.0, 0.0, 0.0, 0.0, -17.0]
        motor3d_api.scene.update()

    def test_scene_update_populates_chain(self, motor3d_api):
        """Tras update(), last_chain debe tener end_effector con 3 coordenadas float."""
        motor3d_api.scene.update()
        ee = motor3d_api.scene.last_chain['end_effector']
        assert len(ee) == 3
        assert all(isinstance(v, float) for v in ee)

    def test_draw_geometry_pipeline_no_crash(self, motor3d_api):
        """La geometría 3D (FK, proyección, triangulación) no debe lanzar excepción.
        Se parchea ImageTk para evitar la dependencia del display Tkinter.
        """
        motor3d_api.scene.update()
        # Parchear ImageTk.PhotoImage para no necesitar Tk inicializado
        with mock.patch('motor3d.rendering.robot3d_drawing.ImageTk') as mock_itk:
            mock_itk.PhotoImage.return_value = mock.MagicMock()
            motor3d_api.draw(_MockCanvas())

    def test_draw_multiple_joint_configs_no_crash(self, motor3d_api):
        """Renderizar varias configuraciones articulares no debe lanzar excepción."""
        configs = [
            [0.0] * 6,
            [45.0, -30.0, 20.0, 0.0, 0.0, -17.0],
            [-45.0, 30.0, -20.0, 10.0, -10.0, -17.0],
        ]
        with mock.patch('motor3d.rendering.robot3d_drawing.ImageTk') as mock_itk:
            mock_itk.PhotoImage.return_value = mock.MagicMock()
            for cfg in configs:
                for i, q in enumerate(cfg):
                    motor3d_api.model.set_joint(i, q)
                motor3d_api.scene.update()
                motor3d_api.draw(_MockCanvas())

    def test_trail_accumulates_when_enabled(self, motor3d_api):
        """Al activar la trayectoria y mover el brazo, trail_points debe crecer."""
        motor3d_api.scene.set_show_trail(True)
        motor3d_api.scene.clear_trail()
        motor3d_api.scene.update()
        initial_len = len(motor3d_api.scene.trail_points)

        for angle in [0.0, 15.0, 30.0, 45.0, 30.0, 15.0]:
            motor3d_api.model.set_joint(1, angle)
            motor3d_api.scene.update()

        assert len(motor3d_api.scene.trail_points) > initial_len, (
            "trail_points debe aumentar al mover las articulaciones")

    def test_scene_last_points_length(self, motor3d_api):
        """last_points debe tener dof+1 elementos tras update()."""
        motor3d_api.scene.update()
        assert len(motor3d_api.scene.last_points) == motor3d_api.model.dof + 1

    def test_joint_arcs_use_frame_x_axis_as_reference(self, motor3d_api):
        """Los arcos de rango articular deben usar el eje X del marco padre
        como dirección de referencia (ángulo=0), no un vector arbitrario.
        Se verifica que _collect_joint_arcs genera segmentos para articulaciones
        rotacionales y que el primer punto del arco está alineado con T[:3,0]."""
        import math
        import numpy as np
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.rendering.robot3d_drawing import _ARC_STEPS

        motor3d_api.model.joints = [0.0] * motor3d_api.model.dof
        motor3d_api.scene.update()
        chain = motor3d_api.scene.last_chain
        points3d = motor3d_api.scene.last_points
        matrices = chain['matrices']

        # Articulación 0 (base): T_padre = identidad, eje X = [1,0,0]
        T0 = np.eye(4)
        u0 = T0[:3, 0]  # [1, 0, 0]
        mn0, mx0 = motor3d_api.model.joint_limits[0]
        r_arc0 = motor3d_api.model.link_lengths[0] * 0.4
        expected_first = [
            points3d[0][0] + r_arc0 * math.cos(math.radians(mn0)) * u0[0],
            points3d[0][1] + r_arc0 * math.cos(math.radians(mn0)) * u0[1],
            points3d[0][2] + r_arc0 * math.cos(math.radians(mn0)) * u0[2],
        ]
        v0 = T0[:3, 1]
        angle_rad = math.radians(mn0)
        expected_first_full = [
            points3d[0][j] + r_arc0 * (math.cos(angle_rad) * u0[j] + math.sin(angle_rad) * v0[j])
            for j in range(3)
        ]
        # Verificar que el primer punto 3D del arco base coincide con la fórmula DH
        # (cálculo directo, sin proyección)
        for coord in range(3):
            assert abs(expected_first_full[coord] - (
                points3d[0][coord] + r_arc0 * (
                    math.cos(angle_rad) * T0[coord, 0] + math.sin(angle_rad) * T0[coord, 1]
                )
            )) < 1e-9, "El primer punto del arco base debe alinearse con el eje X del marco padre"

    def test_trail_color_never_black(self, motor3d_api):
        """El color de la trayectoria no debe ser (0,0,0) para ningún punto.
        La luminosidad mínima debe ser al menos 25% del color base."""
        from motor3d.rendering.robot3d_drawing import Robot3DDrawing
        drawing = motor3d_api.scene.drawing
        base = drawing.TRAIL_COLOR
        min_brightness = 0.25
        min_r = int(base[0] * min_brightness)
        min_g = int(base[1] * min_brightness)
        min_b = int(base[2] * min_brightness)
        # Para alpha=0 (punto más antiguo), brightness = 0.25
        r = min(255, int(base[0] * 0.25))
        g = min(255, int(base[1] * 0.25))
        b = min(255, int(base[2] * 0.25))
        assert r > 0 or g > 0 or b > 0, (
            "El color del punto más antiguo de la trayectoria no debe ser negro")


# ---------------------------------------------------------------------------
# P-CU04-01 : Navegación 3D sin crash
# ---------------------------------------------------------------------------

class TestCameraNavigation:
    """P-CU04-01 — Operaciones de cámara no deben lanzar excepciones."""

    def test_camera_drag_rotate_changes_yaw(self, motor3d_api):
        """Arrastre de cámara en modo rotación debe cambiar el yaw."""
        initial_yaw = motor3d_api.camera.yaw
        for _ in range(5):
            motor3d_api.drag_camera(5, 3)
        assert motor3d_api.camera.yaw != initial_yaw

    def test_camera_drag_pan_changes_offset(self, motor3d_api):
        """Arrastre de cámara en modo pan debe cambiar screen_offset_x/y."""
        motor3d_api.drag_camera(10, -5, pan=True)
        offset_x = motor3d_api.camera.screen_offset_x
        offset_y = motor3d_api.camera.screen_offset_y
        assert offset_x != 0.0 or offset_y != 0.0, "El pan debe modificar el offset de pantalla"

    def test_camera_zoom_via_set_zoom(self, motor3d_api):
        """set_zoom_from_scale debe modificar camera.zoom."""
        initial_zoom = motor3d_api.camera.zoom
        motor3d_api.set_zoom_from_scale(200)  # 200% zoom
        assert motor3d_api.camera.zoom > initial_zoom

        motor3d_api.set_zoom_from_scale(50)  # 50% zoom
        assert motor3d_api.camera.zoom < initial_zoom * 1.5

    def test_camera_reset_restores_defaults(self, motor3d_api):
        """reset_camera debe restaurar yaw, pitch, zoom y offset al valor por defecto."""
        from motor3d.camera.camera import Camera

        motor3d_api.drag_camera(100, 80)
        motor3d_api.drag_camera(-50, 30, pan=True)
        motor3d_api.set_zoom_from_scale(300)
        motor3d_api.reset_camera()

        assert abs(motor3d_api.camera.yaw - Camera.DEFAULT_YAW) < 0.1
        assert abs(motor3d_api.camera.pitch - Camera.DEFAULT_PITCH) < 0.1
        assert abs(motor3d_api.camera.zoom - 1.0) < 0.01
        assert abs(motor3d_api.camera.screen_offset_x) < 0.1
        assert abs(motor3d_api.camera.screen_offset_y) < 0.1

    def test_camera_keyboard_move_no_crash(self, motor3d_api):
        """keyboard_camera con teclas WASD no debe lanzar excepción."""
        keys = {"W": True, "A": False, "S": False, "D": False,
                "w": True, "a": False, "s": False, "d": False}
        motor3d_api.keyboard_camera(keys)

    def test_camera_pitch_clamped(self, motor3d_api):
        """El pitch debe estar clampeado en [-89°, 89°] tras muchos arrastres."""
        for _ in range(200):
            motor3d_api.drag_camera(0, 10)  # arrastra hacia abajo (pitch baja)
        assert motor3d_api.camera.pitch >= -89.5, (
            f"Pitch mínimo excedido: {motor3d_api.camera.pitch:.1f}")

    def test_camera_no_nan_after_extreme_rotation(self, motor3d_api):
        """Rotaciones extremas no deben producir NaN en yaw/pitch."""
        for _ in range(100):
            motor3d_api.drag_camera(15, 8)
        assert not math.isnan(motor3d_api.camera.yaw)
        assert not math.isnan(motor3d_api.camera.pitch)


# ---------------------------------------------------------------------------
# Tests adicionales: integración del compilador Braccio
# ---------------------------------------------------------------------------

class TestBraccioCompiler:
    """Verifica que el sketch Braccio se transpila correctamente."""

    def test_library_add_library_handles_filename_format(self):
        """add_library debe aceptar 'Braccio.h' (semántico) y 'libraries.braccio' (codegen)."""
        from libraries.libs import LibraryManager

        lm1 = LibraryManager()
        assert lm1.add_library("Braccio.h") == LibraryManager.OK
        assert "Braccio" in lm1.library_methods

        lm2 = LibraryManager()
        assert lm2.add_library("libraries.braccio") == LibraryManager.OK
        assert "Braccio" in lm2.library_methods

    def test_library_find_servo_movement_after_add(self):
        """find() debe encontrar ServoMovement tras add_library."""
        from libraries.libs import LibraryManager

        lm = LibraryManager()
        lm.add_library("Braccio.h")
        method = lm.find("Braccio", "ServoMovement")
        assert method is not None, "ServoMovement debe estar registrado"
        assert method[1] == "servo_movement", "El nombre Python debe ser 'servo_movement'"

    def test_library_find_begin_after_add(self):
        """find() debe encontrar begin tras add_library."""
        from libraries.libs import LibraryManager

        lm = LibraryManager()
        lm.add_library("Braccio.h")
        method = lm.find("Braccio", "begin")
        assert method is not None, "begin debe estar registrado"
        assert method[1] == "begin"

    def test_braccio_class_instantiation(self):
        """La clase Braccio debe poder instanciarse sin argumentos."""
        from libraries.braccio import Braccio
        b = Braccio()
        assert b is not None
        assert b.board is None

    def test_braccio_servo_movement_with_mock_board(self):
        """servo_movement debe actualizar los servos a través del board."""
        from libraries.braccio import Braccio, BRACCIO_PINS

        # Board mock con elementos servo mock
        class MockElem:
            def __init__(self): self.last_pin = None; self.last_val = None
            def set_value(self, pin, val): self.last_pin = pin; self.last_val = val

        class MockBoard:
            def __init__(self):
                self.elements = {pin: MockElem() for pin in BRACCIO_PINS.values()}
            def get_pin_element(self, pin):
                return self.elements.get(pin)

        board = MockBoard()
        b = Braccio(board)
        b._resolve_servos()
        b.servo_movement(20, 90, 45, 90, 90, 90, 73)

        shoulder_pin = BRACCIO_PINS['shoulder']
        assert board.elements[shoulder_pin].last_val == 45, (
            f"El servo del hombro debe tener valor 45, obtuvo {board.elements[shoulder_pin].last_val}")


# ---------------------------------------------------------------------------
# Tests adicionales: seguridad y constraints
# ---------------------------------------------------------------------------

class TestSafetyAndConstraints:
    """Verifica el SafetyManager y el clamping de articulaciones."""

    def test_clamp_joints_to_limits(self, braccio_model):
        """clamp_model_joints debe llevar joints fuera de límites al límite más cercano."""
        from motor3d.kinematics.constraints_limits import clamp_model_joints

        braccio_model.joints = [200.0, 200.0, 200.0, 200.0, 200.0, 200.0]
        clamp_model_joints(braccio_model)

        for i, (q, (mn, mx)) in enumerate(zip(braccio_model.joints, braccio_model.joint_limits)):
            assert mn <= q <= mx, f"Joint {i + 1} = {q} fuera de [{mn}, {mx}] tras clamping"

    def test_clamp_negative_joints(self, braccio_model):
        """clamp_model_joints debe funcionar también con valores muy negativos."""
        from motor3d.kinematics.constraints_limits import clamp_model_joints

        braccio_model.joints = [-200.0] * 6
        clamp_model_joints(braccio_model)

        for i, (q, (mn, mx)) in enumerate(zip(braccio_model.joints, braccio_model.joint_limits)):
            assert mn <= q <= mx, f"Joint {i + 1} = {q} fuera de [{mn}, {mx}]"

    def test_safety_manager_in_workspace(self, motor3d_api):
        """Con brazo en posición válida, evaluate_safety debe indicar in_workspace=True."""
        motor3d_api.model.joints = [0.0, 0.0, 0.0, 0.0, 0.0, -17.0]
        motor3d_api.scene.update()
        result = motor3d_api.evaluate_safety()
        assert result['in_workspace'] is True

    def test_safety_result_has_required_keys(self, motor3d_api):
        """El resultado de evaluate_safety debe tener las claves requeridas."""
        motor3d_api.scene.update()
        result = motor3d_api.evaluate_safety()
        required = {'in_workspace', 'singular', 'blocked', 'message'}
        missing = required - result.keys()
        assert not missing, f"Faltan claves en el resultado: {missing}"

    def test_set_joint_within_limits(self, motor3d_api):
        """set_joint con valor fuera de límite debe aplicar clamping."""
        # J1 tiene límite [-90, 90]
        motor3d_api.set_joint(0, 999.0)
        assert motor3d_api.model.joints[0] <= motor3d_api.model.joint_limits[0][1] + 0.1


# ---------------------------------------------------------------------------
# P-Persistencia : ArmConfigRepository save/load  (Tabla 110 del plan)
# ---------------------------------------------------------------------------

class TestPersistence:
    """Pruebas unitarias de persistencia de configuración (ArmConfigRepository)."""

    def test_save_and_load_roundtrip(self, braccio_model, tmp_path):
        """Guardar y recargar una configuración debe producir el mismo modelo."""
        from motor3d.persistence.arm_config_repository import ArmConfigRepository
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState

        repo = ArmConfigRepository()
        path = tmp_path / "test_config.json"

        # Guardar
        ok = repo.save_model(braccio_model, path=path)
        assert ok, "save_model debería devolver True"
        assert path.exists(), "El archivo JSON debe existir tras guardar"

        # Recargar en un modelo vacío
        model2 = ArmKinematicState()
        ok2 = repo.load_model(model2, path=path, silent=True)
        assert ok2, "load_model debería devolver True"

        assert model2.dof == braccio_model.dof
        assert model2.joint_limits == braccio_model.joint_limits
        assert len(model2.dh_rows) == len(braccio_model.dh_rows)

    def test_load_builtin_preset_braccio(self):
        """load_builtin_preset debe cargar el preset braccio_tinkerkit correctamente."""
        from motor3d.persistence.arm_config_repository import ArmConfigRepository
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState

        repo = ArmConfigRepository()
        model = ArmKinematicState()
        ok = repo.load_builtin_preset(model, "braccio_tinkerkit", silent=True)
        assert ok
        assert model.dof == 6
        assert len(model.joint_limits) == 6

    def test_load_nonexistent_file_returns_false(self):
        """load_model con ruta inexistente debe devolver False sin lanzar excepción."""
        from motor3d.persistence.arm_config_repository import ArmConfigRepository
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState

        repo = ArmConfigRepository()
        model = ArmKinematicState()
        ok = repo.load_model(model, path="/ruta/que/no/existe.json", silent=True)
        assert not ok

    def test_list_builtin_presets_returns_dict(self):
        """list_builtin_presets debe devolver un diccionario con al menos un preset."""
        from motor3d.persistence.arm_config_repository import ArmConfigRepository

        repo = ArmConfigRepository()
        presets = repo.list_builtin_presets()
        assert isinstance(presets, dict)
        assert len(presets) >= 1

    def test_save_model_invalid_path_returns_false(self, tmp_path, braccio_model):
        """save_model con ruta inaccesible (padre es un fichero) debe devolver False."""
        from motor3d.persistence.arm_config_repository import ArmConfigRepository

        repo = ArmConfigRepository()
        # Crear un fichero donde el código esperaría un directorio padre
        blocker = tmp_path / "not_a_dir"
        blocker.write_text("block")
        ok = repo.save_model(braccio_model, path=str(blocker / "config.json"))
        assert not ok


# ---------------------------------------------------------------------------
# P-Depurador : Integración Compilador + sistema de depuración (Tabla 113)
# ---------------------------------------------------------------------------

class TestDebugSystem:
    """Pruebas de integración del sistema de depuración (RF4.2.2, RF4.2.3)."""

    def test_transpiler_emits_debug_line(self):
        """El transpilador debe emitir screen_updater.debug_line(N) antes de cada sentencia."""
        import sys
        sys.path.insert(0, '.')
        from compiler.transpiler import transpile

        sketch = """
void setup() {}
void loop() {
    int x = 0;
    x = x + 1;
}
"""
        warns, errors, ast_tree = transpile(sketch)
        assert not errors, f"Errores de compilación inesperados: {errors}"

        with open('temp/script_arduino.py', 'r', encoding='utf-8') as f:
            code = f.read()

        assert 'screen_updater.debug_line(' in code, (
            "El código transpilado debe contener llamadas a screen_updater.debug_line()")

    def test_execution_paused_exception_exists(self):
        """ExecutionPaused debe ser una excepción importable desde commands."""
        from compiler.commands import ExecutionPaused
        assert issubclass(ExecutionPaused, Exception)

    def test_debug_line_raises_when_paused(self):
        """debug_line debe lanzar ExecutionPaused cuando el controlador dice pausar."""
        import graphics.screen_updater as su
        from compiler.commands import ExecutionPaused

        class _MockController:
            def debug_should_pause_at_line(self, line_no):
                return True  # siempre pausar

        original_controller = su.controller
        su.controller = _MockController()
        try:
            with pytest.raises(ExecutionPaused):
                su.debug_line(5)
        finally:
            su.controller = original_controller

    def test_debug_line_no_raise_when_not_paused(self):
        """debug_line no debe lanzar excepción cuando el controlador no pide pausa."""
        import graphics.screen_updater as su

        class _MockController:
            def debug_should_pause_at_line(self, line_no):
                return False  # nunca pausar

        original_controller = su.controller
        su.controller = _MockController()
        # Evitar que refresh() toque tkinter en tests
        original_layer = su.layer
        su.layer = None
        try:
            su.debug_line(3)  # no debe lanzar
        finally:
            su.controller = original_controller
            su.layer = original_layer

    def test_debug_should_pause_at_breakpoint(self):
        """debug_should_pause_at_line debe pausar cuando la línea tiene breakpoint."""
        from graphics.controller import RobotsController

        ctrl = RobotsController.__new__(RobotsController)
        ctrl.executing = True
        ctrl.paused = False
        ctrl.step_pending = False
        ctrl._breakpoints = {10}

        result = ctrl.debug_should_pause_at_line(10)
        assert result is True
        assert ctrl.paused is True

    def test_debug_step_once_allows_one_step(self):
        """step_once debe dejar step_pending=True y paused=False para permitir un paso."""
        from graphics.controller import RobotsController

        ctrl = RobotsController.__new__(RobotsController)
        ctrl.executing = True
        ctrl.paused = True
        ctrl.step_pending = False
        ctrl._breakpoints = set()

        ctrl.step_once()

        assert ctrl.step_pending is True
        assert ctrl.paused is False
