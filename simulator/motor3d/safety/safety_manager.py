"""
SafetyManager — Servicio de cómputo puro para evaluación de seguridad cinemática.
Sin estado propio; delega en workspace_singularity.
"""
from motor3d.safety.workspace_singularity import in_workspace, near_singularity


class SafetyManager:

    def evaluate(self, points, max_reach):
        """
        Evalúa el estado de seguridad del brazo.

        Args:
            points   : lista de posiciones articulares [[x,y,z], ...]
            max_reach: alcance máximo del brazo en mm.

        Returns:
            Dict con claves:
                'in_workspace' : bool
                'singular'     : bool
                'blocked'      : bool  (True si hay que bloquear la simulación)
                'message'      : str   (mensaje de aviso o cadena vacía)
        """
        workspace_ok = in_workspace(points, max_reach)
        singular = near_singularity(points)

        blocked = not workspace_ok or singular
        message = ""

        if not workspace_ok:
            message = "Posición fuera del espacio de trabajo del robot."
        elif singular:
            message = "Singularidad detectada: eslabones alineados. Ajuste la posición manualmente."

        return {
            'in_workspace': workspace_ok,
            'singular': singular,
            'blocked': blocked,
            'message': message,
        }
