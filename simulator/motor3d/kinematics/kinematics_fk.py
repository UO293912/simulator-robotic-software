"""
Cinemática Directa (Forward Kinematics) mediante parametrización Denavit-Hartenberg.

Convención modificada DH:
    Articulación R: T_i = Rz(theta_i + q_i) * Tz(d_i)   * Tx(a_i) * Rx(alpha_i)
    Articulación P: T_i = Rz(theta_i)       * Tz(d_i+qi) * Tx(a_i) * Rx(alpha_i)
"""
import math
import numpy as np


def get_base_transform(model):
    """Retorna la transformacion fija del joint 0 implicito de la base."""
    base = getattr(model, 'base_row', None)
    if isinstance(base, dict):
        theta = math.radians(float(base.get('theta', 0.0)))
        d = float(base.get('d', 0.0))
        a = float(base.get('a', 0.0))
        alpha = math.radians(float(base.get('alpha', 0.0)))
        return _dh_transform(theta, d, a, alpha)

    raw_rpy = getattr(model, 'base_rpy', [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
    roll = math.radians(float(raw_rpy[0]) if len(raw_rpy) > 0 else 0.0)
    pitch = math.radians(float(raw_rpy[1]) if len(raw_rpy) > 1 else 0.0)
    yaw = math.radians(float(raw_rpy[2]) if len(raw_rpy) > 2 else 0.0)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ], dtype=float)
    Ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ], dtype=float)
    Rz = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)

    T = np.eye(4, dtype=float)
    T[:3, :3] = Rz @ Ry @ Rx
    return T


def forward_kinematics_chain(model):
    """
    Calcula la cadena cinemática completa.

    Args:
        model: ArmKinematicState con los parámetros DH y ángulos articulares.

    Returns:
        Dict con:
            'matrices'    — lista de matrices 4x4 acumuladas (una por articulación)
            'positions'   — lista de [x, y, z] del origen de cada articulación
            'end_effector'— [x, y, z] del efector final (con tool_offset)
    """
    T = get_base_transform(model)
    matrices = []
    positions = [[float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]]

    for i in range(model.dof):
        row = model.dh_rows[i]
        jtype = model.joint_types[i] if i < len(model.joint_types) else 'R'
        theta = math.radians(float(row.get('theta', 0.0)))
        d = float(row.get('d', 0.0))
        a = float(row.get('a', 0.0))
        alpha = math.radians(float(row.get('alpha', 0.0)))

        if jtype == 'P':
            Ti = _dh_transform(theta, d + model.joints[i], a, alpha)
        else:
            q = math.radians(model.joints[i])
            Ti = _dh_transform(theta + q, d, a, alpha)
        T = T @ Ti
        matrices.append(T.copy())
        positions.append([float(T[0, 3]), float(T[1, 3]), float(T[2, 3])])

    # Efector final con offset de herramienta
    to = model.tool_offset
    if to and any(v != 0 for v in to):
        t_local = np.array([float(to[0]), float(to[1]), float(to[2]), 1.0])
        t_world = T @ t_local
        end_effector = [float(t_world[0]), float(t_world[1]), float(t_world[2])]
    else:
        end_effector = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]

    return {
        'matrices': matrices,
        'positions': positions,
        'end_effector': end_effector,
    }


def _dh_transform(theta, d, a, alpha):
    """
    Calcula la matriz de transformación DH para un eslabón.

    T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)

    Resultado explícito:
        [cos(θ), -sin(θ)*cos(α),  sin(θ)*sin(α),  a*cos(θ)]
        [sin(θ),  cos(θ)*cos(α), -cos(θ)*sin(α),  a*sin(θ)]
        [0,       sin(α),         cos(α),           d      ]
        [0,       0,              0,                1      ]
    """
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d     ],
        [0.0, 0.0,      0.0,     1.0   ]
    ], dtype=float)
