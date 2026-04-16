"""
Cinemática Inversa (Inverse Kinematics) — Método Jacobiano-transpuesto iterativo.
Jacobiano calculado por diferencias finitas con delta = 0.5° por articulación rotacional.
El cálculo se realiza en radianes internamente para mantener las unidades consistentes
(J en mm/rad, error en mm, paso en rad). Los ángulos del modelo se almacenan en grados.
"""
import math

from motor3d.kinematics.kinematics_fk import forward_kinematics_chain
from motor3d.kinematics.constraints_limits import clamp_model_joints

_DELTA_DEG = 0.5                       # perturbación para diferencias finitas (grados)
_DELTA_RAD = math.radians(_DELTA_DEG)  # equivalente en radianes ≈ 0.008727


def solve_inverse_kinematics(model, target, max_iter=150, tolerance=25.0, alpha=0.65):
    """
    Calcula los ángulos articulares para alcanzar la posición objetivo.

    Internamente trabaja en radianes para consistencia de unidades:
      J (mm/rad) * alpha (1) * e (mm)  →  Δq (mm²/rad)  convertido a grados.

    Args:
        model     : ArmKinematicState — se modifica in-place si converge.
        target    : [x, y, z] — posición objetivo del efector final (mm).
        max_iter  : número máximo de iteraciones.
        tolerance : error aceptable en mm.
        alpha     : tasa de aprendizaje (adimensional, escala en mm⁻² típica 0.3-1.0).

    Returns:
        (converged: bool, final_error: float)
    """
    tx, ty, tz = float(target[0]), float(target[1]), float(target[2])

    # Comprobar si el punto está dentro del workspace
    max_reach = model.max_reach()
    dist_target = math.sqrt(tx ** 2 + ty ** 2 + tz ** 2)
    if dist_target > max_reach * 1.05:
        chain = forward_kinematics_chain(model)
        ee = chain['end_effector']
        err = math.sqrt((ee[0] - tx) ** 2 + (ee[1] - ty) ** 2 + (ee[2] - tz) ** 2)
        return False, err

    dof = model.dof
    limits = model.joint_limits
    types = model.joint_types

    # Trabajar en radianes internamente
    joints_rad = [math.radians(q) for q in model.joints]
    limits_rad = [(math.radians(mn), math.radians(mx)) for mn, mx in limits]

    def _get_ee(qs_rad):
        """FK con joints temporales en radianes."""
        orig = list(model.joints)
        model.joints[:] = [math.degrees(q) for q in qs_rad]
        chain = forward_kinematics_chain(model)
        model.joints[:] = orig
        return chain['end_effector']

    # Factor alpha reescalado: convierte mm²/rad en radianes válidos.
    # norm_scale ≈ 1/max_reach² da pasos de ~α radianes cuando error ≈ max_reach.
    norm_scale = alpha / (max_reach ** 2 + 1e-6)

    for _ in range(max_iter):
        model.joints[:] = [math.degrees(q) for q in joints_rad]
        chain = forward_kinematics_chain(model)
        ee = chain['end_effector']

        ex = tx - ee[0]
        ey = ty - ee[1]
        ez = tz - ee[2]
        error = math.sqrt(ex ** 2 + ey ** 2 + ez ** 2)

        if error <= tolerance:
            clamp_model_joints(model)
            return True, error

        # Jacobiano por diferencias finitas (mm/rad)
        J = []
        for i in range(dof):
            if types[i] == 'P':
                J.append([0.0, 0.0, 0.0])
                continue
            q_plus = list(joints_rad)
            q_plus[i] += _DELTA_RAD
            ee_plus = _get_ee(q_plus)
            J.append([
                (ee_plus[0] - ee[0]) / _DELTA_RAD,
                (ee_plus[1] - ee[1]) / _DELTA_RAD,
                (ee_plus[2] - ee[2]) / _DELTA_RAD,
            ])

        # Paso Jacobiano-transpuesto: Δq = norm_scale * J^T * e  (en radianes)
        e_vec = [ex, ey, ez]
        for i in range(dof):
            if types[i] == 'P':
                continue
            grad = J[i][0] * e_vec[0] + J[i][1] * e_vec[1] + J[i][2] * e_vec[2]
            joints_rad[i] += norm_scale * grad
            mn, mx = limits_rad[i]
            joints_rad[i] = max(mn, min(mx, joints_rad[i]))

    model.joints[:] = [math.degrees(q) for q in joints_rad]
    clamp_model_joints(model)
    chain = forward_kinematics_chain(model)
    ee = chain['end_effector']
    final_err = math.sqrt((ee[0] - tx) ** 2 + (ee[1] - ty) ** 2 + (ee[2] - tz) ** 2)
    return final_err <= tolerance, final_err
