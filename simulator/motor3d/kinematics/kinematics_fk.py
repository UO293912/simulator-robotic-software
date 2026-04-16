"""
Cinemática Directa (Forward Kinematics) mediante parametrización Denavit-Hartenberg.

Convención modificada DH:
    T_i = Rz(theta_i + q_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
"""
import math
import numpy as np


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
    T = np.eye(4)
    matrices = []
    positions = [[0.0, 0.0, 0.0]]

    for i in range(model.dof):
        row = model.dh_rows[i]
        q = math.radians(model.joints[i])
        theta = math.radians(float(row.get('theta', 0.0)))
        d = float(row.get('d', 0.0))
        a = float(row.get('a', 0.0))
        alpha = math.radians(float(row.get('alpha', 0.0)))

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
