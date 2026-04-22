"""
Cinematica Inversa (Inverse Kinematics) - metodo Jacobiano-transpuesto iterativo.

Usa un Jacobiano posicional analitico y mantiene unidades internas mixtas:
  - juntas R: radianes
  - juntas P: milimetros

De esta forma preservamos la convergencia previa en articulaciones rotacionales
y anadimos soporte correcto para prismatica sin mezclar grados con milimetros.
"""
import math

from motor3d.kinematics.constraints_limits import clamp_model_joints
from motor3d.kinematics.kinematics_fk import (
    forward_kinematics_chain,
    get_base_transform,
    get_joint_axis_world,
)

_RAD_TO_DEG = 180.0 / math.pi
_COL_EPS = 1e-9


def _position_jacobian_native(model, chain):
    """
    Jacobiano posicional expresado en las unidades internas del solver.

    Para juntas R:
        d p / d q(rad) = z x (p_ee - p_joint)

    Para juntas P:
        d p / d q(mm) = z
    donde z es el eje local de la articulacion en coordenadas del mundo.
    """
    matrices = chain.get('matrices', [])
    positions = chain.get('positions', [])
    ee = chain.get('end_effector', [0.0, 0.0, 0.0])
    base_T = get_base_transform(model)
    J = []

    for i in range(model.dof):
        T_prev = matrices[i - 1] if i > 0 else base_T
        axis = get_joint_axis_world(model, i, T_prev)

        if model.joint_types[i] == 'P':
            J.append(axis)
            continue

        joint_pos = positions[i] if i < len(positions) else [0.0, 0.0, 0.0]
        rx = float(ee[0]) - float(joint_pos[0])
        ry = float(ee[1]) - float(joint_pos[1])
        rz = float(ee[2]) - float(joint_pos[2])

        J.append([
            axis[1] * rz - axis[2] * ry,
            axis[2] * rx - axis[0] * rz,
            axis[0] * ry - axis[1] * rx,
        ])

    return J


def _max_step_native(jtype, limits):
    """Cota suave por iteracion para evitar saltos grandes e inestables."""
    mn, mx = float(limits[0]), float(limits[1])
    span = abs(mx - mn)
    if jtype == 'P':
        return max(5.0, min(40.0, span * 0.25 if span > 0.0 else 10.0))
    return math.radians(max(2.0, min(15.0, span * 0.25 if span > 0.0 else 5.0)))


def solve_inverse_kinematics(model, target, max_iter=150, tolerance=25.0, alpha=0.65):
    """
    Calcula los angulos/desplazamientos articulares para alcanzar la posicion objetivo.

    Trabaja con unidades internas mixtas:
      - juntas R en radianes
      - juntas P en milimetros

    Para las R conserva el escalado previo basado en alcance maximo.
    Para las P usa un paso normalizado por columna, adecuado a su unidad lineal.

    Args:
        model     : ArmKinematicState - se modifica in-place si converge.
        target    : [x, y, z] - posicion objetivo del efector final (mm).
        max_iter  : numero maximo de iteraciones.
        tolerance : error aceptable en mm.
        alpha     : tasa de aprendizaje adimensional.

    Returns:
        (converged: bool, final_error: float)
    """
    tx, ty, tz = float(target[0]), float(target[1]), float(target[2])

    # Comprobar si el punto esta claramente fuera del workspace.
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
    joints_internal = []
    limits_internal = []
    for q, (mn, mx), jtype in zip(model.joints, limits, types):
        if jtype == 'P':
            joints_internal.append(float(q))
            limits_internal.append((float(mn), float(mx)))
        else:
            joints_internal.append(math.radians(float(q)))
            limits_internal.append((math.radians(float(mn)), math.radians(float(mx))))

    rot_norm_scale = alpha / (max_reach ** 2 + 1e-6)

    for _ in range(max_iter):
        model.joints[:] = [
            q if jtype == 'P' else q * _RAD_TO_DEG
            for q, jtype in zip(joints_internal, types)
        ]
        chain = forward_kinematics_chain(model)
        ee = chain['end_effector']

        ex = tx - ee[0]
        ey = ty - ee[1]
        ez = tz - ee[2]
        error = math.sqrt(ex ** 2 + ey ** 2 + ez ** 2)

        if error <= tolerance:
            clamp_model_joints(model)
            return True, error

        J = _position_jacobian_native(model, chain)
        e_vec = [ex, ey, ez]
        deltas = [0.0] * dof

        for i in range(dof):
            col = J[i]
            grad = col[0] * e_vec[0] + col[1] * e_vec[1] + col[2] * e_vec[2]
            if types[i] == 'P':
                col_norm_sq = col[0] * col[0] + col[1] * col[1] + col[2] * col[2]
                if col_norm_sq <= _COL_EPS:
                    continue
                step = alpha * grad / (col_norm_sq + _COL_EPS)
            else:
                step = rot_norm_scale * grad

            max_step = _max_step_native(types[i], limits[i])
            deltas[i] = max(-max_step, min(max_step, step))

        for i in range(dof):
            mn, mx = limits_internal[i]
            joints_internal[i] = max(mn, min(mx, joints_internal[i] + deltas[i]))

    model.joints[:] = [
        q if jtype == 'P' else q * _RAD_TO_DEG
        for q, jtype in zip(joints_internal, types)
    ]
    clamp_model_joints(model)
    chain = forward_kinematics_chain(model)
    ee = chain['end_effector']
    final_err = math.sqrt((ee[0] - tx) ** 2 + (ee[1] - ty) ** 2 + (ee[2] - tz) ** 2)
    return final_err <= tolerance, final_err
