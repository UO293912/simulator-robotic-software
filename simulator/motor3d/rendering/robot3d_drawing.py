"""
Robot3DDrawing — Motor de renderizado 3D.
Pipeline: NumPy vectorizado → PIL rasterización → Tkinter PhotoImage (un único blit).

BraccioVisualModel  — Carga mallas STL del Braccio; anima con matrices FK.
GenericDhVisualModel — Geometría procedural para cualquier cadena DH.
MeshAsset           — Contenedor de triángulos + color base.
"""
import math
import os
import struct
import numpy as np
from motor3d.kinematics.kinematics_fk import get_base_transform

try:
    from PIL import Image, ImageDraw, ImageTk
    _PIL_OK = True
except ImportError:
    _PIL_OK = False


# ---------------------------------------------------------------------------
# Constantes de renderizado
# ---------------------------------------------------------------------------
_LIGHT_DIR = np.array([0.35, -0.45, 1.0], dtype=float)
_LIGHT_DIR = _LIGHT_DIR / np.linalg.norm(_LIGHT_DIR)

_ARC_COLOR = (0, 150, 130)
_ARC_STEPS = 32
_AMBIENT = 0.30


# ---------------------------------------------------------------------------
# MeshAsset
# ---------------------------------------------------------------------------
class MeshAsset:
    """Contenedor de triángulos en coordenadas de modelo + color base."""

    def __init__(self, triangles, color=(180, 180, 180)):
        """
        Args:
            triangles: array numpy (N, 3, 3) — N triángulos, cada uno con 3 vértices xyz.
            color    : (R, G, B) entero 0-255.
        """
        self.triangles = np.asarray(triangles, dtype=float)
        self.color = color

    def is_empty(self):
        return self.triangles.size == 0


# ---------------------------------------------------------------------------
# Lectura de STL
# ---------------------------------------------------------------------------
def _load_stl(path):
    """
    Lee un archivo STL (binario o ASCII) y retorna array numpy (N, 3, 3).
    Retorna array vacío si el archivo no existe o hay error.
    """
    if not os.path.isfile(path):
        return np.zeros((0, 3, 3), dtype=float)
    try:
        with open(path, 'rb') as f:
            header = f.read(80)
            if header is None:
                return np.zeros((0, 3, 3), dtype=float)
            # Intentar STL binario
            count_data = f.read(4)
            if len(count_data) < 4:
                return np.zeros((0, 3, 3), dtype=float)
            n_tris = struct.unpack('<I', count_data)[0]
            expected = n_tris * 50
            remaining = f.read(expected)
            if len(remaining) == expected and n_tris > 0:
                # Es STL binario
                tris = []
                offset = 0
                for _ in range(n_tris):
                    # 3 floats normal + 9 floats verts + 2 bytes attr
                    vals = struct.unpack_from('<12f', remaining, offset)
                    v0 = [vals[3], vals[4], vals[5]]
                    v1 = [vals[6], vals[7], vals[8]]
                    v2 = [vals[9], vals[10], vals[11]]
                    tris.append([v0, v1, v2])
                    offset += 50
                return np.array(tris, dtype=float)
    except Exception:
        pass

    # Intentar STL ASCII
    try:
        tris = []
        with open(path, 'r', errors='replace') as f:
            content = f.read()
        lines = content.splitlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if line.startswith('facet normal'):
                verts = []
                i += 1
                while i < len(lines) and len(verts) < 3:
                    vl = lines[i].strip()
                    if vl.startswith('vertex'):
                        parts = vl.split()
                        verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
                    i += 1
                if len(verts) == 3:
                    tris.append(verts)
            else:
                i += 1
        if tris:
            return np.array(tris, dtype=float)
    except Exception:
        pass

    return np.zeros((0, 3, 3), dtype=float)


# ---------------------------------------------------------------------------
# BraccioVisualModel
# ---------------------------------------------------------------------------
class BraccioVisualModel:
    """
    Modelo visual específico del TinkerKit Braccio.
    Carga 7 mallas STL y las anima mediante transformaciones de articulación.
    """

    REQUIRED_DOF = 6
    STL_NAMES = [
        'braccio_base.stl',
        'braccio_shoulder.stl',
        'braccio_elbow.stl',
        'braccio_wrist_pitch.stl',
        'braccio_wrist_roll.stl',
        'braccio_left_gripper.stl',
        'braccio_right_gripper.stl',
    ]
    # Colores por pieza (RGB)
    PIECE_COLORS = [
        (80, 80, 80),    # base
        (60, 120, 200),  # shoulder
        (60, 120, 200),  # elbow
        (60, 120, 200),  # wrist pitch
        (60, 120, 200),  # wrist roll
        (220, 80, 60),   # gripper L
        (220, 80, 60),   # gripper R
    ]

    # Los STL del Braccio ya están en mm — no se aplica escala.
    STL_SCALE = 1.0

    # Offsets visuales extraídos del URDF braccio_arm.urdf.
    # Formato: (xyz_mm [x, y, z], rpy [roll, pitch, yaw] en radianes).
    # Las coordenadas xyz están en mm (valores URDF en metros × 1000).
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll) — convención URDF estándar.
    VISUAL_OFFSETS = [
        # piece 0 — braccio_base_link: rpy="0 0 π"  xyz="0 0.004 0"
        ([0.0, 4.0, 0.0],   [0.0, 0.0, math.pi]),
        # piece 1 — shoulder_link:    rpy="0 0 0"   xyz="-0.0045 0.0055 -0.026"
        ([-4.5, 5.5, -26.0], [0.0, 0.0, 0.0]),
        # piece 2 — elbow_link:       rpy="0 0 0"   xyz="-0.0045 0.005 -0.025"
        ([-4.5, 5.0, -25.0], [0.0, 0.0, 0.0]),
        # piece 3 — wrist_pitch_link: rpy="0 0 0"   xyz="0.003 -0.0004 -0.024"
        ([3.0, -0.4, -24.0], [0.0, 0.0, 0.0]),
        # piece 4 — wrist_roll_link:  rpy="0 0 0"   xyz="0.006 0 0"
        ([6.0, 0.0, 0.0],   [0.0, 0.0, 0.0]),
        # piece 5 — left_gripper_link:  rpy="0 π/2 0"  xyz="0 -0.012 0"
        ([0.0, -12.0, 0.0], [0.0, math.pi / 2, 0.0]),
        # piece 6 — right_gripper_link: rpy="0 π/2 0"  xyz="0 -0.012 0.010"
        ([0.0, -12.0, 10.0], [0.0, math.pi / 2, 0.0]),
    ]

    def __init__(self, stl_dir):
        self._stl_dir = stl_dir
        self._assets = []
        self._loaded = False
        self._offset_matrices = [
            self._build_offset_matrix(xyz, rpy)
            for xyz, rpy in self.VISUAL_OFFSETS
        ]
        self._load_assets()

    @staticmethod
    def _Raxis(axis, angle):
        """
        Matriz de rotación 4×4 alrededor de un eje arbitrario por angle radianes.
        Rodrigues' rotation formula.
        """
        ax = np.array(axis, dtype=float)
        norm = np.linalg.norm(ax)
        if norm < 1e-9:
            return np.eye(4, dtype=float)
        ax = ax / norm
        c, s = math.cos(angle), math.sin(angle)
        t = 1.0 - c
        x, y, z = ax
        R = np.array([
            [t*x*x + c,     t*x*y - s*z,  t*x*z + s*y],
            [t*x*y + s*z,   t*y*y + c,    t*y*z - s*x],
            [t*x*z - s*y,   t*y*z + s*x,  t*z*z + c  ],
        ], dtype=float)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        return T

    @staticmethod
    def _build_offset_matrix(xyz_mm, rpy):
        """
        Construye una matriz homogénea 4×4 desde un origen visual URDF.
        También usada para orígenes articulares (joint origins).

        Args:
            xyz_mm : [x, y, z] desplazamiento en mm.
            rpy    : [roll, pitch, yaw] en radianes.
                     Convención URDF: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

        Returns:
            np.ndarray shape (4, 4).
        """
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        Rx = np.array([[1, 0,   0  ],
                       [0, cr, -sr ],
                       [0, sr,  cr ]], dtype=float)
        Ry = np.array([[ cp, 0, sp],
                       [  0, 1,  0],
                       [-sp, 0, cp]], dtype=float)
        Rz = np.array([[cy, -sy, 0],
                       [sy,  cy, 0],
                       [ 0,   0, 1]], dtype=float)

        R = Rz @ Ry @ Rx
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3]  = xyz_mm
        return T

    def _load_assets(self):
        self._assets = []
        for i, name in enumerate(self.STL_NAMES):
            path = os.path.join(self._stl_dir, name)
            tris = _load_stl(path)
            if tris.size > 0:
                tris = tris * self.STL_SCALE
            color = self.PIECE_COLORS[i] if i < len(self.PIECE_COLORS) else (150, 150, 150)
            self._assets.append(MeshAsset(tris, color))
        self._loaded = any(not a.is_empty() for a in self._assets)

    def supports_model(self, model):
        return model.dof >= self.REQUIRED_DOF and self._loaded

    # Offset del TCP en el frame local de T_wrist_r (en mm), calculado como
    # el punto medio entre las puntas de ambas garras cuando el gripper está
    # totalmente cerrado (servo=73° → q_urdf=1.274 rad).
    # Derivado del centroide de los vértices extremos de los STL de la pinza.
    _TCP_IN_WRIST_R = np.array([1.57, -0.50, 126.55, 1.0])

    def get_effective_end_effector(self, model, points, chain):
        """
        Devuelve el TCP (tool center point) calculado con la cadena URDF:
        la posición de la punta de las garras cuando están cerradas,
        independiente del estado real de apertura del gripper (joint[5]).

        El TCP se define como el punto medio de las puntas de ambas garras
        en posición cerrada, expresado en el frame de T_wrist_r:
        offset = [1.57, -0.50, 126.55] mm.
        """
        try:
            q = [(model.joints[i] + 90.0) * math.pi / 180.0
                 for i in range(min(5, model.dof))]   # solo joints 0-4; gripper no afecta TCP
            while len(q) < 5:
                q.append(math.pi / 2)

            _T = self._build_offset_matrix
            _R = self._Raxis
            base_T = get_base_transform(model)

            T_base    = base_T @ _R([0, 0, 1], q[0])
            T_shoulder = T_base @ _T([0.0, -2.0, 72.0], [-math.pi / 2, 0.0, 0.0]) \
                         @ _R([1, 0, 0], q[1])
            T_elbow   = T_shoulder @ _T([0.0, 0.0, 125.0], [-math.pi / 2, 0.0, 0.0]) \
                        @ _R([1, 0, 0], q[2])
            T_wrist_p = T_elbow @ _T([0.0, 0.0, 125.0], [-math.pi / 2, 0.0, 0.0]) \
                        @ _R([1, 0, 0], q[3])
            T_pre_wr  = T_wrist_p @ _T([0.0, 0.0, 60.0], [0.0, 0.0, math.pi / 2])
            T_wrist_r = T_pre_wr @ _R([0, 0, -1], q[4])

            # TCP = punta de las garras cerradas en frame T_wrist_r
            tcp = T_wrist_r @ self._TCP_IN_WRIST_R
            return [float(tcp[0]), float(tcp[1]), float(tcp[2])]
        except Exception:
            # Fallback al efector DH si algo falla
            if chain and chain.get('end_effector'):
                return chain['end_effector']
            return list(points[-1]) if points else [0.0, 0.0, 0.0]

    def get_joint_frames(self, model, chain=None):
        """
        Retorna la lista de marcos articulares en el sistema URDF
        (idéntico al de iter_triangles) para que _collect_joint_arcs dibuje
        los arcos en las posiciones correctas del modelo renderizado.

        Cada elemento: {'pos': [x,y,z], 'axis': np.array, 'xref': np.array, 'r_arc': float}

        La dirección de referencia (xref) corresponde a joints[i]=0 (posición neutro DH),
        calculada como el eje del marco AFTER transform @ Rjoint(q_urdf=π/2) que es
        perpendicular al eje de rotación — equivalente a -T_pre[:3,1] para las
        articulaciones con eje X, y [1,0,0] para la rotación de base (eje Z).
        """
        q = [(model.joints[i] + 90.0) * math.pi / 180.0
             for i in range(min(6, model.dof))]
        while len(q) < 6:
            q.append(math.pi / 2)

        _T = self._build_offset_matrix
        _R = self._Raxis
        base_T = get_base_transform(model)

        frames = []

        # Joint 0 — base, rotación alrededor de Z del mundo
        T_base = base_T @ _R([0, 0, 1], q[0])
        frames.append({
            'pos': base_T[:3, 3].tolist(),
            'axis': base_T[:3, 2].copy(),
            'xref': np.array([1.0, 0.0, 0.0]),   # X world = dirección a q0=0
            'r_arc': 80.0,
        })
        frames[0]['xref'] = base_T[:3, 0].copy()

        # Joint 1 — hombro, rotación alrededor de X
        T_pre_shoulder = T_base @ _T([0.0, -2.0, 72.0], [-math.pi / 2, 0.0, 0.0])
        T_shoulder = T_pre_shoulder @ _R([1, 0, 0], q[1])
        frames.append({
            'pos': T_pre_shoulder[:3, 3].tolist(),
            'axis': T_pre_shoulder[:3, 0].copy(),
            'xref': -T_pre_shoulder[:3, 1].copy(),   # dirección del eslabón a q1=0
            'r_arc': 60.0,
        })

        # Joint 2 — codo, rotación alrededor de X
        T_pre_elbow = T_shoulder @ _T([0.0, 0.0, 125.0], [-math.pi / 2, 0.0, 0.0])
        T_elbow = T_pre_elbow @ _R([1, 0, 0], q[2])
        frames.append({
            'pos': T_pre_elbow[:3, 3].tolist(),
            'axis': T_pre_elbow[:3, 0].copy(),
            'xref': -T_pre_elbow[:3, 1].copy(),
            'r_arc': 55.0,
        })

        # Joint 3 — muñeca pitch, rotación alrededor de X
        T_pre_wrist_p = T_elbow @ _T([0.0, 0.0, 125.0], [-math.pi / 2, 0.0, 0.0])
        T_wrist_p = T_pre_wrist_p @ _R([1, 0, 0], q[3])
        frames.append({
            'pos': T_pre_wrist_p[:3, 3].tolist(),
            'axis': T_pre_wrist_p[:3, 0].copy(),
            'xref': -T_pre_wrist_p[:3, 1].copy(),
            'r_arc': 45.0,
        })

        # Joint 4 — muñeca roll, rotación alrededor de -Z
        T_pre_wrist_r = T_wrist_p @ _T([0.0, 0.0, 60.0], [0.0, 0.0, math.pi / 2])
        T_wrist_r = T_pre_wrist_r @ _R([0, 0, -1], q[4])
        frames.append({
            'pos': T_pre_wrist_r[:3, 3].tolist(),
            'axis': -T_pre_wrist_r[:3, 2].copy(),   # -Z de T_pre_wrist_r
            'xref': -T_pre_wrist_r[:3, 1].copy(),   # dirección a q4=0
            'r_arc': 30.0,
        })

        # Joint 5 — pinza (gripper_joint URDF), rotación alrededor de −Y
        # Se usa el origen articular del dedo derecho (gripper_joint):
        #   xyz=(10,0,30)mm, rpy=(0,−0.2967,0) desde wrist_roll
        # La dirección de referencia (DH joint=0 → q_urdf=π/2) se obtiene
        # aplicando _R([0,-1,0], π/2) al eje Z de T_pre_right_g, que resulta
        # en −T_pre_right_g[:,0].
        T_pre_right_g = T_wrist_r @ _T([10.0, 0.0, 30.0], [0.0, -0.2967, 0.0])
        frames.append({
            'pos': T_pre_right_g[:3, 3].tolist(),
            'axis': -T_pre_right_g[:3, 1].copy(),   # eje −Y (URDF axis=-Y)
            'xref': -T_pre_right_g[:3, 0].copy(),   # dirección del dedo a DH joint[5]=0
            'r_arc': 50.0,
        })

        return frames[:model.dof]

    def iter_triangles(self, model, points, chain):
        """
        Genera (triangles_world, color) para cada pieza.

        Construye su PROPIA cadena cinemática directamente desde los datos del
        URDF braccio_arm.urdf, independiente de las matrices FK del DH.
        (Según §8.9 de la Guía: "NO depende de las matrices DH de la cadena FK")

        Conversión de ángulos:
            q_urdf = (joint_deg + 90.0) * pi / 180
            → servo=0  → q=0,    servo=90 → q=π/2,  servo=180 → q=π
            Validado con límites URDF: shoulder [0.262, 2.880] ≡ servo [15°,165°]

        Yields: (np.array shape (N,3,3), (R,G,B))
        """
        if model.dof < self.REQUIRED_DOF:
            return

        # Ángulos URDF a partir de los ángulos DH del modelo
        q = [(model.joints[i] + 90.0) * math.pi / 180.0
             for i in range(min(6, model.dof))]
        while len(q) < 6:
            q.append(math.pi / 2)

        # ------------------------------------------------------------------
        # Cadena cinemática URDF (todos los offsets xyz en mm, rpy en rad)
        # ------------------------------------------------------------------
        _T = self._build_offset_matrix   # alias corto
        _R = self._Raxis
        base_T = get_base_transform(model)

        # Pieza 0: braccio_base_link
        #   base_joint: parent=base_link(world), origin=(0,0,0), axis=+Z
        T_base = base_T @ _R([0, 0, 1], q[0])

        # Pieza 1: shoulder_link
        #   shoulder_joint: origin xyz=(0,-2,72)mm rpy=(-π/2,0,0), axis=+X
        T_shoulder = T_base \
                     @ _T([0.0, -2.0, 72.0], [-math.pi / 2, 0.0, 0.0]) \
                     @ _R([1, 0, 0], q[1])

        # Pieza 2: elbow_link
        #   elbow_joint: origin xyz=(0,0,125)mm rpy=(-π/2,0,0), axis=+X
        T_elbow = T_shoulder \
                  @ _T([0.0, 0.0, 125.0], [-math.pi / 2, 0.0, 0.0]) \
                  @ _R([1, 0, 0], q[2])

        # Pieza 3: wrist_pitch_link
        #   wrist_pitch_joint: origin xyz=(0,0,125)mm rpy=(-π/2,0,0), axis=+X
        T_wrist_p = T_elbow \
                    @ _T([0.0, 0.0, 125.0], [-math.pi / 2, 0.0, 0.0]) \
                    @ _R([1, 0, 0], q[3])

        # Pieza 4: wrist_roll_link
        #   wrist_roll_joint: origin xyz=(0,0,60)mm rpy=(0,0,π/2), axis=-Z
        T_wrist_r = T_wrist_p \
                    @ _T([0.0, 0.0, 60.0], [0.0, 0.0, math.pi / 2]) \
                    @ _R([0, 0, -1], q[4])

        # Pieza 5: left_gripper_link (sub_gripper_joint, mimic de gripper_joint)
        #   origin xyz=(-10,0,30)mm rpy=(0,3.4383,0), axis=+Y
        T_left_g = T_wrist_r \
                   @ _T([-10.0, 0.0, 30.0], [0.0, 3.4383, 0.0]) \
                   @ _R([0, 1, 0], q[5])

        # Pieza 6: right_gripper_link (gripper_joint)
        #   origin xyz=(10,0,30)mm rpy=(0,-0.2967,0), axis=-Y
        T_right_g = T_wrist_r \
                    @ _T([10.0, 0.0, 30.0], [0.0, -0.2967, 0.0]) \
                    @ _R([0, -1, 0], q[5])

        link_transforms = [
            T_base, T_shoulder, T_elbow, T_wrist_p,
            T_wrist_r, T_left_g, T_right_g,
        ]

        for piece_idx, asset in enumerate(self._assets):
            if asset.is_empty() or piece_idx >= len(link_transforms):
                continue

            # T_final = T_link_world @ T_visual_origin (URDF <visual><origin>)
            T_final = link_transforms[piece_idx] @ self._offset_matrices[piece_idx]

            tris = asset.triangles          # (N, 3, 3) ya en mm
            N = tris.shape[0]
            verts = tris.reshape(-1, 3)
            verts_h = np.hstack([verts, np.ones((verts.shape[0], 1))])
            verts_w = (T_final @ verts_h.T).T
            yield verts_w[:, :3].reshape(N, 3, 3), asset.color


# ---------------------------------------------------------------------------
# Utilidades de geometría
# ---------------------------------------------------------------------------

def _axis_aligned_transform(direction):
    """Matriz 4×4 identidad con columna Z = normalize(direction).
    Se usa para orientar cilindros a lo largo de un vector arbitrario."""
    z = np.array(direction, dtype=float)
    n = np.linalg.norm(z)
    if n < 1e-9:
        return np.eye(4)
    z = z / n
    ref = np.array([0.0, 1.0, 0.0]) if abs(z[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(ref, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4, dtype=float)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    return T


# ---------------------------------------------------------------------------
# GenericDhVisualModel
# ---------------------------------------------------------------------------
class GenericDhVisualModel:
    """
    Modelo visual genérico para cualquier cadena DH (1-6 DOF).
    Cada articulación se representa como una carcasa discoidea + núcleo cilíndrico
    (similar a un cuerpo de servo), y cada eslabón como una sección rectangular.
    """

    # Dimensiones mínimas en mm — aseguran visibilidad a zoom normal
    MIN_RADIUS = 25.0
    MAX_RADIUS = 55.0
    MIN_LINK_W = 18.0
    MAX_LINK_W = 44.0
    JOINT_STEPS = 12

    def supports_model(self, model):
        return model.dof >= 1

    def get_effective_end_effector(self, model, points, chain):
        if chain and chain.get('end_effector'):
            return chain['end_effector']
        if points:
            return points[-1]
        return [0.0, 0.0, 0.0]

    def _joint_neutral_xref(self, model, index, parent_transform):
        """Dirección de referencia para q=0 respetando el theta fijo DH."""
        row = model.dh_rows[index] if index < len(model.dh_rows) else {}
        theta_deg = float(row.get('theta', 0.0))
        theta_rad = math.radians(theta_deg)

        x_parent = np.asarray(parent_transform[:3, 0], dtype=float)
        y_parent = np.asarray(parent_transform[:3, 1], dtype=float)
        return math.cos(theta_rad) * x_parent + math.sin(theta_rad) * y_parent

    def get_joint_frames(self, model, chain=None):
        frames = []
        matrices = chain.get('matrices', []) if chain else []
        positions = chain.get('positions', []) if chain else []
        base_T = get_base_transform(model)
        for i in range(model.dof):
            T = matrices[i - 1] if i > 0 else base_T
            pos = positions[i] if i < len(positions) else [0.0, 0.0, 0.0]
            r_arc = model.link_lengths[i] * 0.4 if i < len(model.link_lengths) else 40.0
            frames.append({
                'pos': pos,
                'axis': T[:3, 2].copy(),
                'xref': self._joint_neutral_xref(model, i, T),
                'r_arc': r_arc,
            })
        return frames

    def iter_triangles(self, model, points, chain):
        """
        Genera triángulos de la geometría procedural del brazo.

        Cada articulación:
          - R: disco exterior (eje de rotación = col-Z del frame anterior) + hub cilíndrico.
          - P: housing fijo en p0 + vástago delgado que se extiende hasta p1 (long. variable).
          - Último joint + eslabón muy corto → pinza de dos dedos que abre/cierra.
        Yields: (np.array shape (N,3,3), (R,G,B))
        """
        if not chain or 'matrices' not in chain:
            return

        matrices = chain['matrices']
        positions = chain.get('positions', [])
        if len(positions) < 2:
            return

        dims = self._resolve_dimensions(model)
        r  = dims['radius']
        lw = dims['link_w']
        base_T = get_base_transform(model)

        for i in range(model.dof):
            p0 = np.array(positions[i],     dtype=float)
            p1 = np.array(positions[i + 1], dtype=float)

            # Frame anterior: define el eje de rotación REAL del joint i (col Z)
            T_prev = matrices[i - 1] if i > 0 else base_T
            # Transform alineado con la dirección del eslabón (p0→p1)
            T_link = _axis_aligned_transform(p1 - p0)

            jtype    = model.joint_types[i] if i < len(model.joint_types) else 'R'
            at_limit = model.is_at_limit(i)
            seg_len  = float(np.linalg.norm(p1 - p0))

            if at_limit:
                actuator_color = (230, 80, 30)
            elif jtype == 'P':
                actuator_color = (55, 185, 90)
            else:
                actuator_color = (55, 115, 205)

            housing_color = (58, 62, 75)
            link_color    = (140, 145, 162)

            # ---- Pinza: último joint con eslabón muy corto ----
            is_gripper = (i == model.dof - 1 and jtype != 'P' and seg_len < r * 2.8)

            if is_gripper:
                # Frame real del EE (acumulado hasta el último joint).
                # Cuando seg_len≈0 (a=0), T_link es identidad — inútil.
                # matrices[i] = frame acumulado TRAS aplicar el joint i.
                T_ee = matrices[i] if i < len(matrices) else T_prev

                # Palma (carcasa del último joint) — perpendicular al eje de rotación
                palm = _make_cylinder(p0, T_prev, r * 0.65, r * 0.50, self.JOINT_STEPS)
                if palm.size > 0:
                    yield palm, housing_color

                jval = model.joints[i] if i < len(model.joints) else 0.0
                mn, mx = model.joint_limits[i] if i < len(model.joint_limits) else (0.0, 90.0)
                t = (jval - mn) / (mx - mn) if mx > mn else 0.5
                t = max(0.0, min(1.0, t))

                # Usar el frame EE real para orientar los dedos:
                #   col-Z = dirección de avance (hacia el objeto)
                #   col-X = dirección de apertura lateral
                forward    = T_ee[:3, 2]
                side       = T_ee[:3, 0]
                finger_len = r * 2.4
                finger_w   = r * 0.42
                # min → abierto (t=0 → gap grande), max → cerrado (t=1 → gap pequeño)
                half_gap   = r * (0.72 - 0.56 * t)

                for sign in (+1.0, -1.0):
                    f_root = p0 + side * sign * half_gap
                    f_tip  = f_root + forward * finger_len
                    prism  = _make_link_prism(f_root, f_tip, finger_w)
                    if prism.size > 0:
                        yield prism, actuator_color

            elif jtype == 'P':
                # Brida de conexión (flange) en p0
                flange = _make_cylinder(p0, T_prev, r, r * 0.30, self.JOINT_STEPS)
                if flange.size > 0:
                    yield flange, (90, 110, 130)
                # Housing cilíndrico fijo centrado en p0
                housing_h = r * 1.6
                housing = _make_cylinder(p0, T_link, r * 0.58, housing_h, self.JOINT_STEPS)
                if housing.size > 0:
                    yield housing, housing_color
                # Vástago delgado que se extiende de p0 a p1 (longitud = extensión actual)
                if seg_len > r * 0.4:
                    rod_center = (p0 + p1) * 0.5
                    rod = _make_cylinder(rod_center, T_link, r * 0.20, seg_len, self.JOINT_STEPS)
                    if rod.size > 0:
                        yield rod, actuator_color
                # No se dibuja link_prism — el vástago ya representa la conexión

            else:
                # R: carcasa discoidea + hub actuador
                disc = _make_cylinder(p0, T_prev, r,        r * 0.40, self.JOINT_STEPS)
                if disc.size > 0:
                    yield disc, housing_color
                hub_h = min(r * 1.6, max(r * 0.8, seg_len * 0.25))
                hub = _make_cylinder(p0, T_link, r * 0.38, hub_h, self.JOINT_STEPS)
                if hub.size > 0:
                    yield hub, actuator_color
                # Sección de eslabón
                prism = _make_link_prism(p0, p1, lw)
                if prism.size > 0:
                    yield prism, link_color

        # Indicador del efector final
        ee = np.array(chain.get('end_effector', positions[-1]), dtype=float)
        sphere = _make_sphere_approx(ee, r * 0.58)
        if sphere.size > 0:
            yield sphere, (220, 180, 50)

    def _resolve_dimensions(self, model):
        """Calcula radio y ancho de eslabón usando |a| + |d| de cada fila DH."""
        seg_lengths = []
        for row in model.dh_rows:
            a = abs(float(row.get('a', 0)))
            d = abs(float(row.get('d', 0)))
            seg = math.sqrt(a * a + d * d)
            if seg > 1.0:
                seg_lengths.append(seg)
        avg_len = sum(seg_lengths) / len(seg_lengths) if seg_lengths else 100.0
        r  = max(self.MIN_RADIUS, min(self.MAX_RADIUS, avg_len * 0.12))
        lw = max(self.MIN_LINK_W, min(self.MAX_LINK_W, avg_len * 0.09))
        return {'radius': r, 'link_w': lw}


# ---------------------------------------------------------------------------
# Funciones de geometría procedural
# ---------------------------------------------------------------------------

def _make_cylinder(center, transform_mat, radius, height, steps):
    """Genera triángulos de un cilindro orientado según los ejes de transform_mat."""
    tris = []
    angles = [2 * math.pi * k / steps for k in range(steps)]
    # Usar la columna Z de la matriz de transformación como eje del cilindro
    ax = transform_mat[:3, 2]
    norm = np.linalg.norm(ax)
    if norm < 1e-9:
        ax = np.array([0.0, 0.0, 1.0])
    else:
        ax = ax / norm

    # Ortogonal al eje
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ax, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(ax, ref)
    u = u / np.linalg.norm(u)
    v = np.cross(ax, u)

    c = np.array(center, dtype=float)
    top = c + ax * height * 0.5
    bot = c - ax * height * 0.5

    for k in range(steps):
        a0, a1 = angles[k], angles[(k + 1) % steps]
        p0b = bot + radius * (math.cos(a0) * u + math.sin(a0) * v)
        p1b = bot + radius * (math.cos(a1) * u + math.sin(a1) * v)
        p0t = top + radius * (math.cos(a0) * u + math.sin(a0) * v)
        p1t = top + radius * (math.cos(a1) * u + math.sin(a1) * v)
        tris.append(np.array([[p0b, p1b, p0t]]))
        tris.append(np.array([[p1b, p1t, p0t]]))

    if tris:
        result = np.vstack(tris)
        return result
    return np.zeros((0, 3, 3))


def _make_link_prism(p0, p1, width):
    """Genera triángulos de un prisma cuadrado entre dos puntos."""
    p0 = np.array(p0, dtype=float)
    p1 = np.array(p1, dtype=float)
    axis = p1 - p0
    length = np.linalg.norm(axis)
    if length < 1e-6:
        return np.zeros((0, 3, 3))
    axis = axis / length

    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(axis, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u = u / np.linalg.norm(u) * (width / 2)
    v = np.cross(axis, u)
    v_n = np.linalg.norm(v)
    if v_n > 1e-9:
        v = v / v_n * (width / 2)

    corners_b = [p0 + u + v, p0 - u + v, p0 - u - v, p0 + u - v]
    corners_t = [p1 + u + v, p1 - u + v, p1 - u - v, p1 + u - v]

    faces = []
    for i in range(4):
        j = (i + 1) % 4
        faces.append([corners_b[i], corners_b[j], corners_t[i]])
        faces.append([corners_b[j], corners_t[j], corners_t[i]])

    return np.array(faces, dtype=float)


def _make_sphere_approx(center, radius, steps=6):
    """Aproximación de esfera con un icosaedro simplificado."""
    c = np.array(center, dtype=float)
    tris = []
    for i in range(steps):
        phi0 = math.pi * i / steps - math.pi / 2
        phi1 = math.pi * (i + 1) / steps - math.pi / 2
        for j in range(steps * 2):
            theta0 = 2 * math.pi * j / (steps * 2)
            theta1 = 2 * math.pi * (j + 1) / (steps * 2)

            def pt(phi, theta):
                return c + radius * np.array([
                    math.cos(phi) * math.cos(theta),
                    math.cos(phi) * math.sin(theta),
                    math.sin(phi)
                ])

            p00 = pt(phi0, theta0)
            p01 = pt(phi0, theta1)
            p10 = pt(phi1, theta0)
            p11 = pt(phi1, theta1)
            tris.append([p00, p01, p10])
            tris.append([p01, p11, p10])

    return np.array(tris, dtype=float)


def _tint_color(color, factor):
    return (
        min(255, int(color[0] * factor)),
        min(255, int(color[1] * factor)),
        min(255, int(color[2] * factor)),
    )


# ---------------------------------------------------------------------------
# Robot3DDrawing
# ---------------------------------------------------------------------------
class Robot3DDrawing:
    """
    Motor de renderizado 3D principal.
    Pipeline: NumPy vectorizado → PIL rasterización → Tkinter PhotoImage.
    """

    STATIC_TAG = "arm3d_static"
    DYNAMIC_TAG = "arm3d_dynamic"
    TRAIL_TAG = "arm3d_trail"

    BG_COLOR = (30, 30, 35)
    GRID_COLOR = (60, 60, 70)
    AXIS_COLORS = {'x': (220, 60, 60), 'y': (60, 200, 60), 'z': (60, 60, 220)}
    TRAIL_COLOR = (0, 220, 180)

    drawing_width = 800
    drawing_height = 600

    def __init__(self, stl_dir=None):
        if stl_dir is None:
            stl_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'assets', 'stl')
            stl_dir = os.path.normpath(stl_dir)

        self.braccio_visual = BraccioVisualModel(stl_dir)
        self.generic_visual = GenericDhVisualModel()
        self.visual_model = None
        self._last_draw_key = None
        self._photo = None
        self._canvas_image_id = None

    def resolve_visual_model(self, model):
        """Selecciona el modelo visual según la configuración (Patrón Estrategia)."""
        visual_mode = model.visual.get('mode', 'auto_generic')
        if visual_mode == 'braccio_exact' and self.braccio_visual.supports_model(model):
            return self.braccio_visual
        return self.generic_visual

    def get_effective_end_effector(self, model, points3d, chain):
        vm = self.resolve_visual_model(model)
        return vm.get_effective_end_effector(model, points3d, chain)

    def draw(self, canvas, camera, points3d, model, chain, trail=None):
        """
        Renderiza la escena completa en el canvas Tkinter.

        Args:
            canvas : tk.Canvas
            camera : Camera
            points3d: lista de [x,y,z] posiciones articulares
            model  : ArmKinematicState
            chain  : resultado de forward_kinematics_chain
            trail  : lista de [x,y,z] puntos de trayectoria o None
        """
        if not _PIL_OK or canvas is None:
            return

        try:
            w = canvas.winfo_width()
            h = canvas.winfo_height()
        except Exception:
            w, h = self.drawing_width, self.drawing_height

        if w < 10 or h < 10:
            w, h = self.drawing_width, self.drawing_height

        img = Image.new('RGB', (w, h), self.BG_COLOR)
        draw = ImageDraw.Draw(img)

        # Rejilla y ejes
        self._draw_grid(draw, camera, w, h)
        self._draw_axes(draw, camera, w, h)

        # Colectar mallas agrupadas por color (evita crear 10k+ tuplas Python)
        vm = self.resolve_visual_model(model)
        self.visual_model = vm
        mesh_groups = []  # lista de (np.array (N,3,3), color)
        for tris_world, color in vm.iter_triangles(model, points3d, chain):
            if tris_world.size == 0:
                continue
            mesh_groups.append((tris_world, color))

        # Trayectoria — dibujada ANTES del mesh para que el brazo la ocluya correctamente
        if trail:
            self._draw_trail(draw, camera, trail, w, h)

        # Renderizado vectorizado del mesh (NumPy pipeline)
        self._render_mesh_vectorized(draw, camera, mesh_groups, w, h)

        # Arcos de rango articular — encima del mesh para que sean visibles
        if points3d and chain and model.visual.get('show_joint_ranges', False):
            for line in self._collect_joint_arcs(model, points3d, chain, camera, w, h):
                draw.line(line, fill=_ARC_COLOR, width=1)

        # Esqueleto de puntos
        if model.visual.get('mode') == 'skeleton':
            self._render_skeleton(draw, camera, points3d, model, w, h)

        # Ejes locales XYZ de cada articulaciÃ³n
        if points3d and chain and model.visual.get('show_joint_axes', False):
            self._draw_joint_axes(draw, model, chain, camera, w, h)

        # Blit a canvas
        self._blit(canvas, img)

    def _blit(self, canvas, img):
        """Sube la imagen PIL al canvas Tkinter con un único create_image."""
        photo = ImageTk.PhotoImage(img)
        self._photo = photo
        if self._canvas_image_id is None:
            try:
                self._canvas_image_id = canvas.create_image(0, 0, anchor='nw', image=photo)
            except Exception:
                self._canvas_image_id = None
        else:
            try:
                canvas.itemconfig(self._canvas_image_id, image=photo)
            except Exception:
                try:
                    self._canvas_image_id = canvas.create_image(0, 0, anchor='nw', image=photo)
                except Exception:
                    pass

    def _render_mesh_vectorized(self, draw, camera, mesh_groups, w, h):
        """
        Pipeline de renderizado vectorizado con NumPy.

        Recibe mesh_groups = lista de (np.array (N,3,3), color_rgb).
        Proyecta todos los vértices de una vez, calcula normales y shading
        en bulk, aplica back-face culling y ordena por profundidad (painter's)
        antes de llamar a draw.polygon — eliminando los bucles Python por vértice.
        """
        if not mesh_groups:
            return

        # ------------------------------------------------------------------ 1. Concatenar
        group_tris = [tris for tris, _ in mesh_groups]      # lista de (Ni,3,3)
        group_colors = []
        for tris, color in mesh_groups:
            group_colors.extend([color] * tris.shape[0])     # un color por triángulo

        all_tris = np.vstack(group_tris)  # (N, 3, 3)
        N = all_tris.shape[0]
        if N == 0:
            return

        # ------------------------------------------------------------------ 2. View matrix (UNA VEZ)
        R_view, cam_pos = camera.get_view_matrix()
        f = camera.focal_length * camera.zoom
        cx = w / 2.0 + camera.screen_offset_x
        cy = h / 2.0 + camera.screen_offset_y

        # ------------------------------------------------------------------ 3. Proyectar TODOS los vértices
        verts = all_tris.reshape(N * 3, 3)               # (N*3, 3)
        verts_cam = (verts - cam_pos) @ R_view.T          # (N*3, 3)

        z_cam = verts_cam[:, 2]                           # (N*3,)
        behind = (z_cam <= 0.01).reshape(N, 3).any(axis=1)  # triángulos con vértice detrás

        # Proyección perspectiva
        z_safe = np.where(z_cam > 0.01, z_cam, 1.0)
        sx = (verts_cam[:, 0] / z_safe) * f + cx         # (N*3,)
        sy = (-verts_cam[:, 1] / z_safe) * f + cy        # (N*3,)
        pts2d = np.stack([sx, sy], axis=1).reshape(N, 3, 2)  # (N, 3, 2)

        # ------------------------------------------------------------------ 4. Normales (vectorizado)
        v0 = all_tris[:, 0, :]    # (N, 3)
        v1 = all_tris[:, 1, :]
        v2 = all_tris[:, 2, :]
        e1 = v1 - v0
        e2 = v2 - v0
        normals = np.cross(e1, e2)                        # (N, 3)
        n_norms = np.linalg.norm(normals, axis=1)         # (N,)
        degen = n_norms < 1e-9
        n_norms_safe = np.where(degen, 1.0, n_norms)
        normals = normals / n_norms_safe[:, np.newaxis]   # (N, 3)

        # ------------------------------------------------------------------ 5. Back-face culling (vectorizado)
        view_dirs = cam_pos - v0                          # (N, 3)
        vd_norms = np.linalg.norm(view_dirs, axis=1)
        vd_safe = np.where(vd_norms < 1e-9, 1.0, vd_norms)
        view_dirs = view_dirs / vd_safe[:, np.newaxis]    # (N, 3)
        facing = np.einsum('ij,ij->i', normals, view_dirs) > 0  # (N,)

        # ------------------------------------------------------------------ 6. Profundidad (centroide)
        centroids_cam = ((v0 + v1 + v2) / 3.0 - cam_pos) @ R_view.T  # (N, 3)
        depths = centroids_cam[:, 2]                      # (N,)

        # ------------------------------------------------------------------ 7. Máscara final
        mask = ~behind & ~degen & facing                  # (N,)
        visible = np.where(mask)[0]
        if visible.size == 0:
            return

        # ------------------------------------------------------------------ 8. Iluminación Lambertiana (vectorizado)
        light_dot = np.einsum('ij,j->i', normals[visible], _LIGHT_DIR)
        shade = _AMBIENT + (1.0 - _AMBIENT) * np.maximum(0.0, light_dot)  # (|visible|,)

        # ------------------------------------------------------------------ 9. Ordenar de lejos a cerca
        sort_order = np.argsort(-depths[visible])
        draw_idx = visible[sort_order]
        shade_sorted = shade[sort_order]

        # ------------------------------------------------------------------ 10. Dibujar
        for k, i in enumerate(draw_idx):
            pts = pts2d[i]           # (3, 2)
            s = float(shade_sorted[k])
            c = group_colors[i]
            r = min(255, int(c[0] * s))
            g = min(255, int(c[1] * s))
            b = min(255, int(c[2] * s))
            try:
                draw.polygon(
                    [pts[0, 0], pts[0, 1], pts[1, 0], pts[1, 1], pts[2, 0], pts[2, 1]],
                    fill=(r, g, b)
                )
            except Exception:
                pass

    def _render_mesh(self, draw, camera, all_tris, w, h):
        """Wrapper de compatibilidad — redirige al pipeline vectorizado."""
        if not all_tris:
            return
        # Agrupar la lista legacy (tri, color) en el formato que espera el pipeline
        groups = {}
        order = []
        for tri, color in all_tris:
            key = color
            if key not in groups:
                groups[key] = []
                order.append(key)
            groups[key].append(tri)
        mesh_groups = []
        for key in order:
            arr = np.array(groups[key])
            mesh_groups.append((arr, key))
        self._render_mesh_vectorized(draw, camera, mesh_groups, w, h)

    def _render_skeleton(self, draw, camera, points3d, model, w, h):
        """Renderizado simplificado con líneas y círculos."""
        if not points3d or len(points3d) < 2:
            return
        prev = None
        for i, p in enumerate(points3d):
            proj = camera.project(p, w, h)
            if proj is None:
                prev = None
                continue
            if prev is not None:
                draw.line([prev, proj], fill=(200, 200, 200), width=3)
            r = 6 if i > 0 else 10
            color = (220, 80, 30) if model.is_at_limit(i - 1) and i > 0 else (60, 180, 220)
            draw.ellipse([proj[0] - r, proj[1] - r, proj[0] + r, proj[1] + r], fill=color)
            prev = proj

    def _draw_grid(self, draw, camera, w, h):
        """Dibuja la rejilla del plano Z=0."""
        size = 400
        step = 100
        color = self.GRID_COLOR
        for x in range(-size, size + step, step):
            p0 = camera.project([float(x), float(-size), 0.0], w, h)
            p1 = camera.project([float(x), float(size), 0.0], w, h)
            if p0 and p1:
                draw.line([p0, p1], fill=color, width=1)
        for y in range(-size, size + step, step):
            p0 = camera.project([float(-size), float(y), 0.0], w, h)
            p1 = camera.project([float(size), float(y), 0.0], w, h)
            if p0 and p1:
                draw.line([p0, p1], fill=color, width=1)

    def _draw_axes(self, draw, camera, w, h):
        """Dibuja los ejes de coordenadas XYZ con etiquetas."""
        origin = [0.0, 0.0, 0.0]
        length = 120.0
        axes = {
            'x': ([length, 0.0, 0.0], self.AXIS_COLORS['x']),
            'y': ([0.0, length, 0.0], self.AXIS_COLORS['y']),
            'z': ([0.0, 0.0, length], self.AXIS_COLORS['z']),
        }
        p0 = camera.project(origin, w, h)
        if p0 is None:
            return
        for label, (end, color) in axes.items():
            p1 = camera.project(end, w, h)
            if p1:
                draw.line([p0, p1], fill=color, width=2)
                try:
                    draw.text((p1[0] + 3, p1[1] - 8), label.upper(), fill=color)
                except Exception:
                    pass

    def _draw_trail(self, draw, camera, trail, w, h):
        """Dibuja la trayectoria del efector final.

        Proyecta cada punto 3D al espacio pantalla.  El segmento entre un punto
        que proyecta y el siguiente que NO proyecta (detrás de la cámara) se
        descarta, evitando líneas erróneas que cruzan la pantalla.

        El color varía de teal tenue (puntos más antiguos) a teal brillante
        (puntos más recientes).  Se usa un mínimo de brillo del 25 % para que
        los puntos antiguos siempre sean visibles.
        """
        if not trail or len(trail) < 2:
            return

        # Proyectar con índice original para respetar la continuidad
        projected = []  # lista de (screen_xy | None)
        for p in trail:
            projected.append(camera.project(p, w, h))

        n = len(projected)
        for i in range(n - 1):
            p0 = projected[i]
            p1 = projected[i + 1]
            # Descartar segmentos con algún extremo detrás de la cámara
            if p0 is None or p1 is None:
                continue
            # alpha = 0 → punto más antiguo, alpha = 1 → punto más reciente
            alpha = i / max(1, n - 1)
            brightness = 0.25 + 0.75 * alpha
            r = min(255, int(self.TRAIL_COLOR[0] * brightness))
            g = min(255, int(self.TRAIL_COLOR[1] * brightness))
            b = min(255, int(self.TRAIL_COLOR[2] * brightness))
            width = 1 if brightness < 0.6 else 2
            try:
                draw.line([p0, p1], fill=(r, g, b), width=width)
            except Exception:
                pass

    def _resolve_joint_basis(self, frame):
        """Construye una base ortonormal local (X, Y, Z) a partir del frame articular."""
        center = np.asarray(frame['pos'], dtype=float)
        axis = np.asarray(frame['axis'], dtype=float)
        u_raw = np.asarray(frame['xref'], dtype=float)

        axis_n = np.linalg.norm(axis)
        if axis_n < 1e-9:
            return None
        axis = axis / axis_n

        u_raw = u_raw - np.dot(u_raw, axis) * axis
        u_n = np.linalg.norm(u_raw)
        if u_n < 1e-9:
            return None
        u = u_raw / u_n

        v_raw = np.cross(axis, u)
        v_n = np.linalg.norm(v_raw)
        if v_n < 1e-9:
            return None
        v = v_raw / v_n
        u = np.cross(v, axis)

        return center, axis, u, v

    def _draw_joint_axes(self, draw, model, chain, camera, w, h):
        """Dibuja los ejes XYZ locales de cada articulaciÃ³n usando sus frames renderizados."""
        vm = self.resolve_visual_model(model)
        frames = vm.get_joint_frames(model, chain)

        for frame in frames[:model.dof]:
            basis = self._resolve_joint_basis(frame)
            if basis is None:
                continue

            center, axis_z, axis_x, axis_y = basis
            p0 = camera.project(center.tolist(), w, h)
            if p0 is None:
                continue

            axis_len = float(np.clip(frame.get('r_arc', 40.0) * 0.65, 24.0, 72.0))
            axes = (
                (axis_x, self.AXIS_COLORS['x']),
                (axis_y, self.AXIS_COLORS['y']),
                (axis_z, self.AXIS_COLORS['z']),
            )
            for axis_dir, color in axes:
                p1 = camera.project((center + axis_len * axis_dir).tolist(), w, h)
                if p1 is None:
                    continue
                try:
                    draw.line([p0, p1], fill=color, width=2)
                except Exception:
                    pass

    def _collect_joint_arcs(self, model, points3d, chain, camera, w, h):
        """Retorna lista de segmentos 2D que representan los arcos de rango articular.

        Delega la obtención de las posiciones y ejes articulares al modelo visual
        activo (vm.get_joint_frames), de modo que los arcos coincidan con el
        modelo 3D renderizado — tanto para el Braccio exacto (URDF) como para
        el modelo genérico DH.

        Para cada articulación rotacional i:
          - center = pivote en coordenadas mundo (del modelo visual)
          - axis   = eje de rotación en coordenadas mundo
          - u/v    = base ortonormal en el plano del arco;
                     u apunta en la dirección neutro (joints[i]=0)
          - Segmentos de arco [mn, mx] grados + líneas radiales.
        """
        lines = []
        vm = self.resolve_visual_model(model)
        frames = vm.get_joint_frames(model, chain)

        for i in range(model.dof):
            if model.joint_types[i] != 'R':
                continue
            if i >= len(frames):
                continue

            frame = frames[i]
            basis = self._resolve_joint_basis(frame)
            if basis is None:
                continue
            center, axis, u, v = basis
            u_raw = u

            # Normalizar eje
            axis_n = np.linalg.norm(axis)
            if axis_n < 1e-9:
                continue
            axis = axis / axis_n

            # Calcular v = cross(axis, u), luego re-ortonormalizar u
            v_raw = np.cross(axis, u_raw)
            v_n = np.linalg.norm(v_raw)
            if v_n < 1e-9:
                continue
            v = v_raw / v_n
            u = np.cross(v, axis)   # re-ortonormalización: u ⟂ axis ⟂ v

            mn, mx = model.joint_limits[i]
            r_arc = frame['r_arc']

            # Puntos del arco con índice original (para descartar saltos si algún punto
            # queda detrás de la cámara sin crear segmentos erróneos que cruzan la pantalla)
            projected = []
            for k in range(_ARC_STEPS + 1):
                angle_deg = mn + (mx - mn) * k / _ARC_STEPS
                angle_rad = math.radians(angle_deg)
                c_a = math.cos(angle_rad)
                s_a = math.sin(angle_rad)
                p3d = [
                    center[0] + r_arc * (c_a * u[0] + s_a * v[0]),
                    center[1] + r_arc * (c_a * u[1] + s_a * v[1]),
                    center[2] + r_arc * (c_a * u[2] + s_a * v[2]),
                ]
                projected.append(camera.project(p3d, w, h))

            for k in range(len(projected) - 1):
                p0 = projected[k]
                p1 = projected[k + 1]
                if p0 is not None and p1 is not None:
                    lines.append([p0, p1])

            # Líneas radiales del centro al inicio y fin del arco
            c_proj = camera.project(list(center), w, h)
            if c_proj:
                if projected[0] is not None:
                    lines.append([c_proj, projected[0]])
                if projected[-1] is not None:
                    lines.append([c_proj, projected[-1]])

        return lines
