"""
Funciones de comprobación de workspace y singularidades.
"""
import math


def in_workspace(points, max_reach):
    """
    Verifica si la posición del efector final está dentro del workspace del robot.

    Args:
        points   : lista de [x, y, z] de las articulaciones (posiciones FK).
        max_reach: alcance máximo del brazo en mm.

    Returns:
        True si la posición es alcanzable, False si está fuera del workspace.
    """
    if not points or len(points) < 2:
        return True

    end = points[-1]
    dist = math.sqrt(end[0] ** 2 + end[1] ** 2 + end[2] ** 2)
    return dist <= max_reach * 1.05  # 5% de tolerancia


def near_singularity(points, threshold=5.0):
    """
    Detecta si la cadena cinemática está cerca de una configuración singular
    (dos o más eslabones consecutivos alineados).

    Args:
        points   : lista de [x, y, z] de los orígenes articulares (posiciones FK).
        threshold: umbral en grados para considerar alineación.

    Returns:
        True si se detecta singularidad, False si no.
    """
    if len(points) < 3:
        return False

    threshold_rad = math.radians(threshold)

    for i in range(len(points) - 2):
        p0 = points[i]
        p1 = points[i + 1]
        p2 = points[i + 2]

        # Vector del eslabón anterior y siguiente
        v1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
        v2 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]

        n1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
        n2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2)

        if n1 < 1e-6 or n2 < 1e-6:
            continue

        dot = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (n1 * n2)
        dot = max(-1.0, min(1.0, dot))
        angle = math.acos(abs(dot))

        if angle < threshold_rad:
            return True

    return False
