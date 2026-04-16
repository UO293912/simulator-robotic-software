"""
ArmKinematicState — Estado cinemático completo del brazo robótico.
Parametrización Denavit-Hartenberg, configurable para 1-6 DOF.
"""


class ArmKinematicState:

    MIN_DOF = 1
    MAX_DOF = 6
    VISUAL_MODES = ["auto_generic", "braccio_exact", "skeleton"]

    def __init__(self):
        self.dof = 0
        self.link_lengths = []
        self.joints = []
        self.joint_limits = []
        self.joint_types = []
        self.dh_rows = []
        self.tool_parent_joint = -1
        self.tool_offset = [0.0, 0.0, 0.0]
        self.visual = {"mode": "auto_generic", "theme": "default", "sizes": {}}

    def configure(self, dof, link_lengths=None, joint_limits=None,
                  joint_types=None, joints=None, dh_rows=None,
                  tool=None, visual=None):
        self.dof = max(self.MIN_DOF, min(self.MAX_DOF, int(dof)))
        n = self.dof

        self.link_lengths = list(link_lengths) if link_lengths else [100.0] * n
        self.link_lengths = self.link_lengths[:n]
        while len(self.link_lengths) < n:
            self.link_lengths.append(100.0)

        if joint_limits:
            self.joint_limits = [tuple(lim) for lim in joint_limits][:n]
        else:
            self.joint_limits = [(-90.0, 90.0)] * n
        while len(self.joint_limits) < n:
            self.joint_limits.append((-90.0, 90.0))

        self.joint_types = list(joint_types) if joint_types else ['R'] * n
        self.joint_types = self.joint_types[:n]
        while len(self.joint_types) < n:
            self.joint_types.append('R')

        if joints:
            self.joints = [float(j) for j in joints][:n]
        else:
            self.joints = [0.0] * n
        while len(self.joints) < n:
            self.joints.append(0.0)

        if dh_rows:
            self.dh_rows = [dict(row) for row in dh_rows][:n]
        else:
            self.dh_rows = [
                {'theta': 0.0, 'd': 0.0, 'a': float(self.link_lengths[i]), 'alpha': 0.0}
                for i in range(n)
            ]
        while len(self.dh_rows) < n:
            i = len(self.dh_rows)
            ll = self.link_lengths[i] if i < len(self.link_lengths) else 100.0
            self.dh_rows.append({'theta': 0.0, 'd': 0.0, 'a': float(ll), 'alpha': 0.0})

        if tool:
            self.tool_parent_joint = tool.get('parent_joint', -1)
            self.tool_offset = tool.get('offset', [0.0, 0.0, 0.0])
        else:
            self.tool_parent_joint = -1
            self.tool_offset = [0.0, 0.0, 0.0]

        if visual:
            self.visual = dict(visual)
        else:
            self.visual = {"mode": "auto_generic", "theme": "default", "sizes": {}}

        self._sync_link_lengths_from_dh()
        self._clamp_joints()

    def set_joint(self, index, value):
        if 0 <= index < self.dof:
            mn, mx = self.joint_limits[index]
            self.joints[index] = max(mn, min(mx, float(value)))

    def set_dh(self, dh_rows):
        n = self.dof
        self.dh_rows = [dict(row) for row in dh_rows][:n]
        while len(self.dh_rows) < n:
            self.dh_rows.append({'theta': 0.0, 'd': 0.0, 'a': 100.0, 'alpha': 0.0})
        self._sync_link_lengths_from_dh()

    def set_limits(self, limits):
        n = self.dof
        self.joint_limits = [tuple(lim) for lim in limits][:n]
        while len(self.joint_limits) < n:
            self.joint_limits.append((-90.0, 90.0))
        self._clamp_joints()

    def is_at_limit(self, index, eps=1.0):
        if 0 <= index < self.dof:
            mn, mx = self.joint_limits[index]
            return self.joints[index] <= mn + eps or self.joints[index] >= mx - eps
        return False

    def max_reach(self):
        return sum(abs(row.get('a', 0.0)) + abs(row.get('d', 0.0))
                   for row in self.dh_rows)

    def to_dict(self):
        return {
            'dof': self.dof,
            'link_lengths': list(self.link_lengths),
            'joints': list(self.joints),
            'joint_limits': [list(lim) for lim in self.joint_limits],
            'joint_types': list(self.joint_types),
            'dh_rows': [dict(row) for row in self.dh_rows],
            'tool': {
                'parent_joint': self.tool_parent_joint,
                'offset': list(self.tool_offset),
            },
            'visual': dict(self.visual),
        }

    def load_dict(self, data):
        self.configure(
            dof=data.get('dof', 1),
            link_lengths=data.get('link_lengths'),
            joint_limits=data.get('joint_limits'),
            joint_types=data.get('joint_types'),
            joints=data.get('joints'),
            dh_rows=data.get('dh_rows'),
            tool=data.get('tool'),
            visual=data.get('visual'),
        )

    def _sync_link_lengths_from_dh(self):
        for i, row in enumerate(self.dh_rows):
            if i < len(self.link_lengths):
                self.link_lengths[i] = abs(row.get('a', self.link_lengths[i]))

    def _clamp_joints(self):
        for i in range(self.dof):
            mn, mx = self.joint_limits[i]
            self.joints[i] = max(mn, min(mx, self.joints[i]))
