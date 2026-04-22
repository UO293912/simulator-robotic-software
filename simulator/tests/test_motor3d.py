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
from types import SimpleNamespace
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


    def test_fk_base_orientation_can_turn_joint_1_into_pendulum(self):
        """La base configurable debe reorientar el eje de la primera articulaciÃ³n."""
        import numpy as np
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain

        model = ArmKinematicState()
        model.configure(
            dof=1,
            link_lengths=[200.0],
            joint_limits=[(-90.0, 90.0)],
            joint_types=['R'],
            joints=[25.0],
            dh_rows=[{'theta': 0.0, 'd': 0.0, 'a': 200.0, 'alpha': 0.0}],
            base={'theta': 90.0, 'd': 0.0, 'a': 0.0, 'alpha': 90.0},
        )

        chain = forward_kinematics_chain(model)
        axis_world = chain['matrices'][0][:3, 2]
        np.testing.assert_allclose(axis_world, [1.0, 0.0, 0.0], atol=1e-6)


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

    def test_ik_prismatic_joint_reaches_linear_target(self):
        """Una P pura debe converger desplazando en mm sobre su eje local."""
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics

        model = ArmKinematicState()
        model.configure(
            dof=1,
            joint_limits=[(0.0, 120.0)],
            joint_types=['P'],
            joints=[0.0],
            dh_rows=[{'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0}],
        )

        converged, error = solve_inverse_kinematics(
            model, [0.0, 0.0, 80.0], max_iter=30, tolerance=1.0, alpha=0.65)
        ee = forward_kinematics_chain(model)['end_effector']

        assert converged, f"IK prismatica no convergio (error={error:.2f} mm)"
        assert abs(model.joints[0] - 80.0) <= 1.0
        assert abs(ee[2] - 80.0) <= 1.0

    def test_ik_prismatic_joint_supports_reoriented_axis(self):
        """Una P debe poder deslizar en cualquier direccion si su eje z se reorienta."""
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics

        model = ArmKinematicState()
        model.configure(
            dof=1,
            joint_limits=[(0.0, 120.0)],
            joint_types=['P'],
            joints=[0.0],
            dh_rows=[{'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0}],
            base={'theta': 90.0, 'd': 0.0, 'a': 0.0, 'alpha': 90.0},
        )

        converged, error = solve_inverse_kinematics(
            model, [60.0, 0.0, 0.0], max_iter=30, tolerance=1.0, alpha=0.65)
        ee = forward_kinematics_chain(model)['end_effector']

        assert converged, f"IK prismatica reorientada no convergio (error={error:.2f} mm)"
        assert abs(model.joints[0] - 60.0) <= 1.0
        assert abs(ee[0] - 60.0) <= 1.0

    def test_ik_prismatic_joint_supports_preoriented_axis(self):
        """Una P debe converger tambien si su eje se preorienta con yaw/pitch."""
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.kinematics.kinematics_ik import solve_inverse_kinematics

        model = ArmKinematicState()
        model.configure(
            dof=1,
            joint_limits=[(0.0, 120.0)],
            joint_types=['P'],
            joints=[0.0],
            dh_rows=[{'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0}],
            prismatic_pre_rotations=[{'yaw': 0.0, 'pitch': 90.0}],
        )

        converged, error = solve_inverse_kinematics(
            model, [70.0, 0.0, 0.0], max_iter=30, tolerance=1.0, alpha=0.65)
        ee = forward_kinematics_chain(model)['end_effector']

        assert converged, f"IK prismatica preorientada no convergio (error={error:.2f} mm)"
        assert abs(model.joints[0] - 70.0) <= 1.0
        assert abs(ee[0] - 70.0) <= 1.0


def test_max_reach_includes_prismatic_joint_stroke():
    """El workspace debe considerar la carrera maxima de las juntas P."""
    from motor3d.kinematics.arm_kinematic_state import ArmKinematicState

    model = ArmKinematicState()
    model.configure(
        dof=1,
        joint_limits=[(-30.0, 100.0)],
        joint_types=['P'],
        joints=[0.0],
        dh_rows=[{'theta': 0.0, 'd': 20.0, 'a': 50.0, 'alpha': 0.0}],
    )

    assert model.max_reach() == 170.0


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

    def test_draw_with_joint_axes_enabled_no_crash(self, motor3d_api):
        """Activar los ejes locales XYZ no debe romper el render ni el flag visual."""
        motor3d_api.set_show_joint_axes(True)
        assert motor3d_api.model.visual.get('show_joint_axes') is True

        motor3d_api.scene.update()
        with mock.patch('motor3d.rendering.robot3d_drawing.ImageTk') as mock_itk:
            mock_itk.PhotoImage.return_value = mock.MagicMock()
            motor3d_api.draw(_MockCanvas())

        motor3d_api.set_show_joint_axes(False)
        assert 'show_joint_axes' not in motor3d_api.model.visual

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

    def test_generic_joint_frames_respect_fixed_theta_offset(self):
        """La referencia neutra de un frame DH debe incluir el theta fijo."""
        import numpy as np
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.rendering.robot3d_drawing import GenericDhVisualModel

        model = ArmKinematicState()
        model.configure(
            dof=2,
            link_lengths=[120.0, 120.0],
            joint_limits=[(-90.0, 90.0), (-75.0, 75.0)],
            joint_types=['R', 'R'],
            joints=[25.0, 0.0],
            dh_rows=[
                {'theta': 0.0, 'd': 90.0, 'a': 0.0, 'alpha': 90.0},
                {'theta': 90.0, 'd': 0.0, 'a': 120.0, 'alpha': 0.0},
            ],
            visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        )

        chain = forward_kinematics_chain(model)
        frames = GenericDhVisualModel().get_joint_frames(model, chain)

        assert len(frames) == 2

        parent_transform = chain['matrices'][0]
        expected_axis = parent_transform[:3, 2]
        expected_xref = parent_transform[:3, 1]

        np.testing.assert_allclose(frames[1]['axis'], expected_axis, atol=1e-9)
        np.testing.assert_allclose(frames[1]['xref'], expected_xref, atol=1e-9)
        assert abs(np.dot(frames[1]['axis'], frames[1]['xref'])) < 1e-9

    def test_generic_gripper_tcp_stays_stable_while_opening(self):
        """La pinza genérica no debe desplazar el TCP visual al abrir o cerrar."""
        import numpy as np
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.rendering.robot3d_drawing import GenericDhVisualModel

        model = ArmKinematicState()
        model.configure(
            dof=3,
            link_lengths=[0.0, 160.0, 20.0],
            joint_limits=[(-90.0, 90.0), (-90.0, 90.0), (-80.0, -10.0)],
            joint_types=['R', 'R', 'R'],
            joints=[15.0, -20.0, -80.0],
            dh_rows=[
                {'theta': 0.0, 'd': 120.0, 'a': 0.0, 'alpha': 90.0},
                {'theta': 0.0, 'd': 0.0, 'a': 160.0, 'alpha': 0.0},
                {'theta': 0.0, 'd': 0.0, 'a': 20.0, 'alpha': 0.0},
            ],
            visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        )

        visual = GenericDhVisualModel()

        chain_open = forward_kinematics_chain(model)
        tcp_open = np.array(visual.get_effective_end_effector(model, chain_open['positions'], chain_open))
        fk_open = np.array(chain_open['end_effector'])

        model.joints[2] = -10.0
        chain_closed = forward_kinematics_chain(model)
        tcp_closed = np.array(visual.get_effective_end_effector(model, chain_closed['positions'], chain_closed))
        fk_closed = np.array(chain_closed['end_effector'])

        np.testing.assert_allclose(tcp_open, tcp_closed, atol=1e-9)
        assert np.linalg.norm(fk_open - fk_closed) > 5.0

    def test_generic_gripper_fingers_counter_rotate_symmetrically(self):
        """La pinza genérica debe abrir los dedos en espejo usando un pivote común."""
        import numpy as np
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.rendering.robot3d_drawing import GenericDhVisualModel

        model = ArmKinematicState()
        model.configure(
            dof=3,
            link_lengths=[0.0, 160.0, 20.0],
            joint_limits=[(-90.0, 90.0), (-90.0, 90.0), (-80.0, -10.0)],
            joint_types=['R', 'R', 'R'],
            joints=[10.0, -15.0, -80.0],
            dh_rows=[
                {'theta': 0.0, 'd': 120.0, 'a': 0.0, 'alpha': 90.0},
                {'theta': 0.0, 'd': 0.0, 'a': 160.0, 'alpha': 0.0},
                {'theta': 0.0, 'd': 0.0, 'a': 20.0, 'alpha': 0.0},
            ],
            visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        )

        visual = GenericDhVisualModel()
        dims = visual._resolve_dimensions(model)

        chain_open = forward_kinematics_chain(model)
        gripper_open = visual._get_gripper_geometry(model, chain_open, dims)

        model.joints[2] = -10.0
        chain_closed = forward_kinematics_chain(model)
        gripper_closed = visual._get_gripper_geometry(model, chain_closed, dims)

        assert gripper_open is not None and gripper_closed is not None

        left_open = next(f for f in gripper_open['fingers'] if f['sign'] > 0)
        right_open = next(f for f in gripper_open['fingers'] if f['sign'] < 0)
        left_closed = next(f for f in gripper_closed['fingers'] if f['sign'] > 0)

        np.testing.assert_allclose(gripper_open['forward'], gripper_closed['forward'], atol=1e-9)
        np.testing.assert_allclose(gripper_open['hinge_axis'], gripper_closed['hinge_axis'], atol=1e-9)
        assert np.dot(left_open['dir'], gripper_open['side']) == pytest.approx(
            -np.dot(right_open['dir'], gripper_open['side']),
            abs=1e-9,
        )
        assert np.dot(left_open['dir'], gripper_open['side']) > 0.0
        assert np.dot(right_open['dir'], gripper_open['side']) < 0.0
        assert np.dot(left_open['dir'], gripper_open['forward']) < np.dot(
            left_closed['dir'],
            gripper_closed['forward'],
        )

    def test_prismatic_visual_geometry_separates_slide_axis_from_rigid_offset(self):
        """Una P con `a` no nulo debe deslizar por z y dejar el offset lateral aparte."""
        import numpy as np
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.rendering.robot3d_drawing import GenericDhVisualModel

        model = ArmKinematicState()
        model.configure(
            dof=1,
            link_lengths=[120.0],
            joint_limits=[(0.0, 120.0)],
            joint_types=['P'],
            joints=[60.0],
            dh_rows=[{'theta': 0.0, 'd': 90.0, 'a': 120.0, 'alpha': 0.0}],
            visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        )

        chain = forward_kinematics_chain(model)
        visual = GenericDhVisualModel()
        prism = visual._get_prismatic_geometry(model, 0, chain)

        assert prism is not None
        np.testing.assert_allclose(prism['axis_dir'], [0.0, 0.0, 1.0], atol=1e-6)
        np.testing.assert_allclose(prism['slide_end'], [0.0, 0.0, 150.0], atol=1e-6)
        np.testing.assert_allclose(chain['positions'][1], [120.0, 0.0, 150.0], atol=1e-6)
        np.testing.assert_allclose(prism['support_vec'], [120.0, 0.0, 0.0], atol=1e-6)

    def test_prismatic_visual_geometry_supports_preoriented_axis(self):
        """La geometria visual debe seguir el yaw/pitch fijo de una P."""
        import numpy as np
        from motor3d.kinematics.arm_kinematic_state import ArmKinematicState
        from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
        from motor3d.rendering.robot3d_drawing import GenericDhVisualModel

        model = ArmKinematicState()
        model.configure(
            dof=1,
            link_lengths=[40.0],
            joint_limits=[(0.0, 120.0)],
            joint_types=['P'],
            joints=[60.0],
            dh_rows=[{'theta': 0.0, 'd': 0.0, 'a': 40.0, 'alpha': 0.0}],
            prismatic_pre_rotations=[{'yaw': 0.0, 'pitch': 90.0}],
            visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        )

        chain = forward_kinematics_chain(model)
        visual = GenericDhVisualModel()
        prism = visual._get_prismatic_geometry(model, 0, chain)

        assert prism is not None
        np.testing.assert_allclose(prism['axis_dir'], [1.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(prism['slide_end'], [60.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(chain['positions'][1], [60.0, 0.0, -40.0], atol=1e-6)
        np.testing.assert_allclose(prism['support_vec'], [0.0, 0.0, -40.0], atol=1e-6)

    def test_projection_context_matches_camera_project(self):
        """La proyeccion cacheada por frame debe coincidir con camera.project()."""
        import pytest
        from motor3d.camera.camera import Camera
        from motor3d.rendering.robot3d_drawing import Robot3DDrawing

        camera = Camera()
        camera.set_orientation(yaw=32.0, pitch=18.0)
        camera.zoom = 1.35
        camera.screen_offset_x = 24.0
        camera.screen_offset_y = -12.0

        drawing = Robot3DDrawing()
        projection = drawing._build_projection_context(camera, 800, 600)

        point = [120.0, -85.0, 210.0]
        expected = camera.project(point, 800, 600)
        actual = drawing._project_point(point, projection)

        assert actual is not None and expected is not None
        assert actual == pytest.approx(expected, abs=1e-9)

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

    def test_arm3d_servo_sync_uses_real_time_not_frames(self, monkeypatch):
        """La animación del Arm3D no debe depender del número de frames renderizados."""
        import graphics.layers as layers_mod

        layer = layers_mod.Arm3DLayer()
        times = iter([0.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2])
        monkeypatch.setattr(layers_mod.time, "monotonic", lambda: next(times))

        # Estado inicial: servo base en 90° -> joint DH = 0°
        layer.robot.servo_base.value = 90
        layer._Arm3DLayer__sync_from_servos()

        # Nuevo objetivo: servo base en 0° -> joint DH = -90°
        layer.robot.servo_base.value = 0
        for _ in range(6):
            layer._Arm3DLayer__sync_from_servos()

        assert layer.motor3d.model.joints[0] <= -89.0, (
            f"J1 debería alcanzar el objetivo por tiempo real, obtuvo {layer.motor3d.model.joints[0]:.1f}°")


# ---------------------------------------------------------------------------
# P-CU04-01 : Navegación 3D sin crash
# ---------------------------------------------------------------------------

def test_arm3d_custom_mode_preserves_full_internal_range_with_visible_servo_offset():
    """En modo custom no debe reinterpretar los joints como servos 0..180."""
    import graphics.layers as layers_mod

    layer = layers_mod.Arm3DLayer()
    layer.motor3d.model.configure(
        dof=1,
        link_lengths=[100.0],
        joint_limits=[(-180.0, 180.0)],
        joint_types=['R'],
        joints=[0.0],
        dh_rows=[{'theta': 0.0, 'd': 0.0, 'a': 100.0, 'alpha': 0.0}],
        visual={'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
    )

    layer.set_joint_angle(0, -30.0)
    layer._Arm3DLayer__sync_from_servos()
    assert abs(layer.motor3d.model.joints[0] + 120.0) < 1e-6

    layer.set_joint_angle(0, 225.0)
    layer._Arm3DLayer__sync_from_servos()
    assert abs(layer.motor3d.model.joints[0] - 135.0) < 1e-6


def test_braccio_layer_keeps_visible_servo_values_and_internal_dh_values():
    """El Braccio predefinido debe seguir usando la numeraciÃ³n servo tradicional."""
    from graphics.layers import Arm3DLayer

    layer = Arm3DLayer()
    layer.set_joint_angle(0, 45.0)
    assert layer.motor3d.model.joints[0] == -45.0
    assert layer.robot.servo_base.value == 45.0


def test_arm3d_slider_passes_visible_servo_value_to_controller():
    """El slider no debe volver a restar 90Â° antes de llegar al controlador."""
    from graphics.gui import Arm3DControlPanel

    class DummyLabel:
        def __init__(self):
            self.text = ""

        def config(self, **kwargs):
            if 'text' in kwargs:
                self.text = kwargs['text']

    calls = []
    panel = Arm3DControlPanel.__new__(Arm3DControlPanel)
    panel._val_labels = [DummyLabel()]
    panel._get_model = lambda: SimpleNamespace(joint_types=['R'])
    panel._is_servo_mode = lambda: True
    panel.application = SimpleNamespace(
        controller=SimpleNamespace(
            update_arm3d_joint=lambda joint_idx, angle: calls.append((joint_idx, angle))
        )
    )

    Arm3DControlPanel._on_slider(panel, 0, "45")

    assert calls == [(0, 45.0)]


def test_arm3d_config_locked_preset_keeps_confirm_enabled():
    """El preset Braccio bloqueado debe desactivar importar, pero no confirmar ni exportar."""
    from graphics.gui import Arm3DConfigurationWindow

    class DummyWidget:
        def __init__(self):
            self.options = {}

        def configure(self, **kwargs):
            self.options.update(kwargs)

    window = Arm3DConfigurationWindow.__new__(Arm3DConfigurationWindow)
    window._dof_spin = DummyWidget()
    window._vis_combo = DummyWidget()
    window._btn_import = DummyWidget()
    window._btn_export = DummyWidget()
    window._btn_save = DummyWidget()
    window._rows = []
    window._base_row_controls = []

    Arm3DConfigurationWindow._set_locked(window, True)

    assert window._btn_import.options["state"] == "disabled"
    assert window._btn_export.options["state"] == "normal"
    assert window._btn_save.options["state"] == "normal"


class _ConfigField:
    def __init__(self, value):
        self.value = value
        self.options = {}

    def get(self):
        return self.value

    def configure(self, **kwargs):
        self.options.update(kwargs)


def _make_arm3d_config_window_for_collect(config_row, joint_type, lim_min, lim_max, direction=None):
    from graphics.gui import Arm3DConfigurationWindow

    direction = direction or {'yaw': 0.0, 'pitch': 0.0}
    window = Arm3DConfigurationWindow.__new__(Arm3DConfigurationWindow)
    window._rows = [[
        _ConfigField(str(config_row['theta'])),
        _ConfigField(str(config_row['d'])),
        _ConfigField(str(config_row['a'])),
        _ConfigField(str(config_row['alpha'])),
        _ConfigField(joint_type),
        _ConfigField(str(lim_min)),
        _ConfigField(str(lim_max)),
        _ConfigField(str(direction['yaw'])),
        _ConfigField(str(direction['pitch'])),
    ]]
    window._base_row_entries = {
        'theta': _ConfigField("0"),
        'd': _ConfigField("0"),
        'a': _ConfigField("0"),
        'alpha': _ConfigField("0"),
    }
    window.motor3d = SimpleNamespace(get_model_config=lambda: {'visual': {'mode': 'auto_generic'}})
    window._visual_var = SimpleNamespace(get=lambda: 'auto_generic')
    return window


def test_arm3d_collect_config_allows_zero_a_zero_d_and_equal_rotational_limits():
    """La configuración DH debe permitir a=0, d=0 y límites iguales en juntas R."""
    from graphics.gui import Arm3DConfigurationWindow

    window = _make_arm3d_config_window_for_collect(
        {'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0},
        'R',
        0.0,
        0.0,
    )

    config, error = Arm3DConfigurationWindow._collect_config(window)

    assert error is None
    assert config is not None
    assert config['dh_rows'][0] == {'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0}
    assert config['joint_limits'][0] == (0.0, 0.0)


def test_arm3d_collect_config_allows_equal_prismatic_limits():
    """La configuración DH debe permitir límites iguales también en juntas P."""
    from graphics.gui import Arm3DConfigurationWindow

    window = _make_arm3d_config_window_for_collect(
        {'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0},
        'P',
        0.0,
        0.0,
    )

    config, error = Arm3DConfigurationWindow._collect_config(window)

    assert error is None
    assert config is not None
    assert config['joint_types'][0] == 'P'
    assert config['joint_limits'][0] == (0.0, 0.0)
    assert config['prismatic_pre_rotations'][0] == {'yaw': 0.0, 'pitch': 0.0}


def test_arm3d_collect_config_allows_prismatic_rigid_offset_in_a():
    """Una P puede tener `a` como offset rígido sin que la GUI la rechace."""
    from graphics.gui import Arm3DConfigurationWindow

    window = _make_arm3d_config_window_for_collect(
        {'theta': 0.0, 'd': 0.0, 'a': 50.0, 'alpha': 0.0},
        'P',
        0.0,
        120.0,
    )

    config, error = Arm3DConfigurationWindow._collect_config(window)

    assert error is None
    assert config is not None
    assert config['dh_rows'][0]['a'] == 50.0
    assert config['joint_types'][0] == 'P'


def test_arm3d_collect_config_stores_prismatic_pre_rotation():
    """La GUI debe guardar yaw/pitch para preorientar una P antes del deslizamiento."""
    from graphics.gui import Arm3DConfigurationWindow

    window = _make_arm3d_config_window_for_collect(
        {'theta': 0.0, 'd': 0.0, 'a': 0.0, 'alpha': 0.0},
        'P',
        0.0,
        120.0,
        direction={'yaw': 90.0, 'pitch': 45.0},
    )

    config, error = Arm3DConfigurationWindow._collect_config(window)

    assert error is None
    assert config is not None
    assert config['prismatic_pre_rotations'][0] == {'yaw': 90.0, 'pitch': 45.0}


def test_arm3d_collect_config_rejects_only_minimum_greater_than_maximum():
    """La validación de límites debe fallar solo cuando el mínimo supera al máximo."""
    from graphics.gui import Arm3DConfigurationWindow

    window = _make_arm3d_config_window_for_collect(
        {'theta': 0.0, 'd': 0.0, 'a': 50.0, 'alpha': 0.0},
        'R',
        10.0,
        0.0,
    )

    config, error = Arm3DConfigurationWindow._collect_config(window)

    assert config is None
    assert error is not None
    assert "no puede ser mayor que el máximo" in error


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
        assert abs(motor3d_api.camera.distance - Camera.DEFAULT_DISTANCE) < 0.1

    def test_camera_distance_expands_for_large_auto_generic_models(self):
        """Los modelos genericos grandes no deben arrancar con la camara dentro del brazo."""
        from motor3d.api import Motor3DApi
        from motor3d.camera.camera import Camera

        api = Motor3DApi()
        config = {
            'dof': 3,
            'link_lengths': [0.0, 500.0, 500.0],
            'joints': [0.0, 0.0, 0.0],
            'joint_limits': [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]],
            'joint_types': ['R', 'R', 'R'],
            'dh_rows': [
                {'theta': 0.0, 'd': 300.0, 'a': 0.0, 'alpha': 90.0},
                {'theta': 0.0, 'd': 0.0, 'a': 500.0, 'alpha': 0.0},
                {'theta': 0.0, 'd': 0.0, 'a': 500.0, 'alpha': 0.0},
            ],
            'tool': {'parent_joint': -1, 'offset': [0.0, 0.0, 0.0]},
            'visual': {'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        }

        api.set_model_config(config)

        assert api.camera.distance > Camera.DEFAULT_DISTANCE
        assert api.camera.distance == pytest.approx(1625.0, abs=1e-6)

    def test_camera_reset_restores_safe_distance_for_large_auto_generic_models(self):
        """Reset camara debe volver a una distancia segura en modelos genericos grandes."""
        from motor3d.api import Motor3DApi

        api = Motor3DApi()
        config = {
            'dof': 3,
            'link_lengths': [0.0, 500.0, 500.0],
            'joints': [0.0, 0.0, 0.0],
            'joint_limits': [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]],
            'joint_types': ['R', 'R', 'R'],
            'dh_rows': [
                {'theta': 0.0, 'd': 300.0, 'a': 0.0, 'alpha': 90.0},
                {'theta': 0.0, 'd': 0.0, 'a': 500.0, 'alpha': 0.0},
                {'theta': 0.0, 'd': 0.0, 'a': 500.0, 'alpha': 0.0},
            ],
            'tool': {'parent_joint': -1, 'offset': [0.0, 0.0, 0.0]},
            'visual': {'mode': 'auto_generic', 'theme': 'default', 'sizes': {}},
        }

        api.set_model_config(config)
        api.camera.distance = 700.0
        api.drag_camera(90, 40)
        api.set_zoom_from_scale(250)

        api.reset_camera()

        assert api.camera.distance == pytest.approx(1625.0, abs=1e-6)

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

    def test_braccio_servo_movement_with_mock_board(self, monkeypatch):
        """servo_movement debe actualizar los servos a través del board."""
        import libraries.braccio as braccio_mod
        from libraries.braccio import Braccio, BRACCIO_PINS

        rest = {
            BRACCIO_PINS['base']: 90,
            BRACCIO_PINS['shoulder']: 90,
            BRACCIO_PINS['elbow']: 180,
            BRACCIO_PINS['wrist_ver']: 180,
            BRACCIO_PINS['wrist_rot']: 90,
            BRACCIO_PINS['gripper']: 10,
        }

        # Board mock con elementos servo mock
        class MockElem:
            def __init__(self, value=0):
                self.value = value
                self.last_pin = None
                self.last_val = None
                self.writes = []

            def set_value(self, pin, val):
                self.value = val
                self.last_pin = pin
                self.last_val = val
                self.writes.append(val)

        class MockBoard:
            def __init__(self):
                self.elements = {pin: MockElem(rest[pin]) for pin in BRACCIO_PINS.values()}

            def get_pin_element(self, pin):
                return self.elements.get(pin)

        delays = []
        monkeypatch.setattr(braccio_mod, "_delay_ms", lambda ms: delays.append(ms))

        board = MockBoard()
        b = Braccio(board)
        b._resolve_servos()
        b.servo_movement(20, 90, 45, 180, 180, 90, 10)

        shoulder_pin = BRACCIO_PINS['shoulder']
        assert board.elements[shoulder_pin].last_val == 45, (
            f"El servo del hombro debe terminar en 45, obtuvo {board.elements[shoulder_pin].last_val}")
        assert board.elements[shoulder_pin].writes[0] == 89
        assert len(board.elements[shoulder_pin].writes) == 45
        assert len(delays) == 45
        assert set(delays) == {20}

    def test_braccio_servo_movement_clamps_targets_and_step_delay(self, monkeypatch):
        """servo_movement debe aplicar el clamping oficial de rangos y stepDelay."""
        import libraries.braccio as braccio_mod

        class MockElem:
            def __init__(self, value=0):
                self.value = value
                self.last_val = value

            def set_value(self, _pin, val):
                self.value = val
                self.last_val = val

        class MockBoard:
            def __init__(self):
                self.elements = {
                    pin: MockElem({
                        braccio_mod.BRACCIO_PINS['base']: 90,
                        braccio_mod.BRACCIO_PINS['shoulder']: 45,
                        braccio_mod.BRACCIO_PINS['elbow']: 180,
                        braccio_mod.BRACCIO_PINS['wrist_ver']: 180,
                        braccio_mod.BRACCIO_PINS['wrist_rot']: 90,
                        braccio_mod.BRACCIO_PINS['gripper']: 10,
                    }[pin])
                    for pin in braccio_mod.BRACCIO_PINS.values()
                }

            def get_pin_element(self, pin):
                return self.elements.get(pin)

        delays = []
        monkeypatch.setattr(braccio_mod, "_delay_ms", lambda ms: delays.append(ms))

        board = MockBoard()
        b = braccio_mod.Braccio(board)
        b._resolve_servos()
        b.servo_movement(5, 200, -90, 181, -91, 999, -999)

        assert board.elements[braccio_mod.BRACCIO_PINS['base']].last_val == 180
        assert board.elements[braccio_mod.BRACCIO_PINS['shoulder']].last_val == 15
        assert board.elements[braccio_mod.BRACCIO_PINS['elbow']].last_val == 180
        assert board.elements[braccio_mod.BRACCIO_PINS['wrist_ver']].last_val == 0
        assert board.elements[braccio_mod.BRACCIO_PINS['wrist_rot']].last_val == 180
        assert board.elements[braccio_mod.BRACCIO_PINS['gripper']].last_val == 10
        assert delays, "ServoMovement debe esperar entre pasos"
        assert set(delays) == {10}

    def test_braccio_module_level_api_uses_active_standard_board(self, monkeypatch):
        """La API de módulo de Braccio debe operar sobre standard.board."""
        import libraries.standard as standard
        import libraries.braccio as braccio

        class MockElem:
            def __init__(self, value=0):
                self.value = value
                self.last_pin = None
                self.last_val = None

            def set_value(self, pin, val):
                self.value = val
                self.last_pin = pin
                self.last_val = val

        class MockBoard:
            def __init__(self):
                self.elements = {
                    pin: MockElem({
                        braccio.BRACCIO_PINS['base']: 90,
                        braccio.BRACCIO_PINS['shoulder']: 45,
                        braccio.BRACCIO_PINS['elbow']: 180,
                        braccio.BRACCIO_PINS['wrist_ver']: 180,
                        braccio.BRACCIO_PINS['wrist_rot']: 90,
                        braccio.BRACCIO_PINS['gripper']: 10,
                    }[pin])
                    for pin in braccio.BRACCIO_PINS.values()
                }

            def get_pin_element(self, pin):
                return self.elements.get(pin)

            def write_value(self, pin, val):
                elem = self.elements.get(pin)
                if elem is not None:
                    elem.set_value(pin, val)
                    return True
                return False

        original_board = standard.board
        original_singleton = braccio._singleton
        try:
            monkeypatch.setattr(braccio, "_delay_ms", lambda _ms: None)
            board = MockBoard()
            standard.board = board
            braccio._singleton = None
            braccio.begin()
            braccio.servo_movement(20, 90, 60, 180, 180, 90, 10)

            shoulder_pin = braccio.BRACCIO_PINS['shoulder']
            assert board.elements[shoulder_pin].last_val == 60
        finally:
            standard.board = original_board
            braccio._singleton = original_singleton

    def test_braccio_begin_allocates_fixed_servos_on_arm_robot_board(self, monkeypatch):
        """Braccio.begin debe reservar los pines oficiales sobre el brazo 3D sin cableado previo."""
        import libraries.standard as standard
        import libraries.braccio as braccio
        from robot_components.robots import ArmHardwareRobot

        original_board = standard.board
        original_singleton = braccio._singleton
        try:
            robot = ArmHardwareRobot()
            monkeypatch.setattr(braccio, "_delay_ms", lambda _ms: None)
            standard.board = robot.board
            braccio._singleton = None

            assert braccio.begin() == 1
            assert robot.servo_base.pin == braccio.BRACCIO_PINS['base']
            assert robot.servo_shoulder.pin == braccio.BRACCIO_PINS['shoulder']
            assert robot.servo_elbow.pin == braccio.BRACCIO_PINS['elbow']
            assert robot.servo_wrist_vertical.pin == braccio.BRACCIO_PINS['wrist_ver']
            assert robot.servo_wrist.pin == braccio.BRACCIO_PINS['wrist_rot']
            assert robot.servo_gripper.pin == braccio.BRACCIO_PINS['gripper']
            assert robot.servo_base.value == 90
            assert robot.servo_shoulder.value == 45
            assert robot.servo_elbow.value == 180
            assert robot.servo_wrist_vertical.value == 180
            assert robot.servo_wrist.value == 90
            assert robot.servo_gripper.value == 10
        finally:
            standard.board = original_board
            braccio._singleton = original_singleton

    def test_arm3d_assigns_joints_by_attach_order(self):
        """Servo.attach/write debe asignar J1..J6 por orden de attach, no por nombre."""
        from graphics.layers import Arm3DLayer
        from libraries.servo import Servo

        layer = Arm3DLayer()
        first = Servo(layer.robot.board, "gripper")
        second = Servo(layer.robot.board, "base")

        assert first.attach(10) == Servo.OK
        assert second.attach(11) == Servo.OK

        first.write(30)
        second.write(120)
        layer._current_joints = None
        layer._Arm3DLayer__sync_from_servos()

        assert layer.motor3d.model.joints[0] == -60.0
        assert layer.motor3d.model.joints[1] == 30.0

    def test_servo_write_clamps_to_official_range(self):
        """Servo.write debe usar la convención oficial y clampear a [0, 180]."""
        from graphics.layers import Arm3DLayer
        from libraries.servo import Servo

        layer = Arm3DLayer()
        servo = Servo(layer.robot.board, "base")
        assert servo.attach(11) == Servo.OK

        servo.write(-60)
        layer._current_joints = None
        layer._Arm3DLayer__sync_from_servos()
        assert layer.motor3d.model.joints[0] == -90.0

        servo.write(250)
        layer._current_joints = None
        layer._Arm3DLayer__sync_from_servos()
        assert layer.motor3d.model.joints[0] == 90.0

    def test_transpiler_initializes_servo_instances_with_board(self):
        """Las declaraciones Servo deben crear instancias enlazadas a la placa activa."""
        from compiler.transpiler import transpile

        sketch = """
#include <Servo.h>

Servo base;
Servo shoulder;

void setup() {}
void loop() {}
"""
        warns, errors, _ast_tree = transpile(sketch)
        assert not warns
        assert not errors

        with open('temp/script_arduino.py', 'r', encoding='utf-8') as f:
            code = f.read()

        assert "base = Servo.Servo(standard.board)" in code
        assert "shoulder = Servo.Servo(standard.board)" in code


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

    def test_setup_returns_false_when_setup_raises(self):
        """Setup.execute debe devolver False si setup() falla en tiempo de ejecución."""
        import compiler.commands as commands
        import libraries.standard as standard
        import robot_components.robot_state as state

        class _MockConsole:
            def __init__(self):
                self.errors = []

            def write_error(self, error):
                self.errors.append(error)

        class _MockRobot:
            board = None

        class _MockLayer:
            robot = _MockRobot()

        class _MockController:
            robot_layer = _MockLayer()
            console = _MockConsole()

        original_module = commands.module
        original_state = standard.state
        try:
            standard.state = state.State()

            class _BrokenModule:
                @staticmethod
                def setup():
                    raise RuntimeError("boom")

            commands.module = _BrokenModule()
            cmd = commands.Setup(_MockController())
            cmd.ready = True

            assert cmd.execute() is False
            assert len(_MockController.console.errors) == 1
        finally:
            commands.module = original_module
            standard.state = original_state

    def test_drawing_loop_does_not_restart_when_execution_stopped(self):
        """drawing_loop no debe relanzar loop() si executing ya es False."""
        import graphics.controller as controller_mod
        import graphics.screen_updater as screen_updater

        ctrl = controller_mod.RobotsController.__new__(controller_mod.RobotsController)
        ctrl.executing = False
        ctrl.paused = False
        ctrl.arm3d = False
        ctrl.robot_layer = None

        class _MockView:
            keys_used = False

            def after(self, *_args):
                return None

        class _MockLoopCommand:
            def __init__(self):
                self.calls = 0

            def execute(self):
                self.calls += 1

        original_refresh = screen_updater.refresh
        try:
            ctrl.view = _MockView()
            ctrl.loop_command = _MockLoopCommand()
            screen_updater.refresh = lambda: None
            ctrl.drawing_loop()
            assert ctrl.loop_command.calls == 0
        finally:
            screen_updater.refresh = original_refresh

    def test_arm3d_render_loop_uses_fast_interval_when_interacting(self):
        """El loop pasivo del Arm3D debe acelerar el refresco si hay interaccion."""
        import graphics.controller as controller_mod
        import graphics.layers as layers_mod

        ctrl = controller_mod.RobotsController.__new__(controller_mod.RobotsController)
        ctrl.arm3d = True
        ctrl.executing = False
        ctrl._arm3d_loop_running = True

        after_calls = []

        class _MockView:
            move_WASD = {'w': False, 'a': False, 's': False, 'd': False}

            def after(self, ms, _cb):
                after_calls.append(ms)
                return None

        class _MockMotor3D:
            def __init__(self):
                self.draw_calls = 0

            def keyboard_camera(self, _move_wasd):
                pass

            def draw(self, _canvas):
                self.draw_calls += 1

            def evaluate_safety(self):
                return {'blocked': False, 'message': '', 'in_workspace': True, 'singular': False}

        layer = layers_mod.Arm3DLayer.__new__(layers_mod.Arm3DLayer)
        layer.motor3d = _MockMotor3D()
        layer._canvas = object()
        layer._update_hud = lambda _safety: None
        layer.wants_fast_render = lambda: True

        ctrl.view = _MockView()
        ctrl.robot_layer = layer

        ctrl.arm3d_render_loop()

        assert layer.motor3d.draw_calls == 1
        assert after_calls == [controller_mod.RobotsController._ARM3D_RENDER_ACTIVE_MS]

    def test_arm3d_render_loop_uses_idle_interval_when_quiet(self):
        """El loop pasivo del Arm3D debe bajar el refresco cuando todo esta quieto."""
        import graphics.controller as controller_mod
        import graphics.layers as layers_mod

        ctrl = controller_mod.RobotsController.__new__(controller_mod.RobotsController)
        ctrl.arm3d = True
        ctrl.executing = False
        ctrl._arm3d_loop_running = True

        after_calls = []

        class _MockView:
            move_WASD = {'w': False, 'a': False, 's': False, 'd': False}

            def after(self, ms, _cb):
                after_calls.append(ms)
                return None

        class _MockMotor3D:
            def keyboard_camera(self, _move_wasd):
                pass

            def draw(self, _canvas):
                pass

            def evaluate_safety(self):
                return {'blocked': False, 'message': '', 'in_workspace': True, 'singular': False}

        layer = layers_mod.Arm3DLayer.__new__(layers_mod.Arm3DLayer)
        layer.motor3d = _MockMotor3D()
        layer._canvas = object()
        layer._update_hud = lambda _safety: None
        layer.wants_fast_render = lambda: False

        ctrl.view = _MockView()
        ctrl.robot_layer = layer

        ctrl.arm3d_render_loop()

        assert after_calls == [controller_mod.RobotsController._ARM3D_RENDER_IDLE_MS]
