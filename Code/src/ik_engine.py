"""IKEngine — Pure math module for robot arm inverse/forward kinematics.

Converts between 3D Cartesian coordinates, joint angles (degrees),
and servo positions (0-1000) for a 2-link planar arm with base rotation.
"""

import math


class IKEngine:
    """Inverse / forward kinematics for a 3-axis robot arm.

    Parameters
    ----------
    l1 : float
        Length from shoulder joint to elbow joint in mm (default 75.0).
    l2 : float
        Length from elbow joint to tip in mm (default 85.0).
    """

    def __init__(self, l1: float = 75.0, l2: float = 85.0) -> None:
        self.l1 = l1
        self.l2 = l2

    # ------------------------------------------------------------------
    # Inverse Kinematics
    # ------------------------------------------------------------------

    def calculate_ik(
        self, x: float, y: float, z: float
    ) -> tuple[float, float, float]:
        """Translate 3D coordinates into joint angles (degrees).

        Parameters
        ----------
        x, y, z : float
            Target position in mm relative to the shoulder joint origin.

        Returns
        -------
        (base_deg, shoulder_deg, elbow_deg) : tuple[float, float, float]
            Joint angles in degrees.

        Notes
        -----
        * base_deg is computed as atan2(y, x).
        * If the target is unreachable (distance > L1+L2), the reach
          distance is clamped to L1+L2 − 0.1 mm.
        """
        # Base pan angle (top-down view)
        base_rad = math.atan2(y, x)

        # 2-D distances in the shoulder plane
        r = math.hypot(x, y)  # horizontal distance
        D = math.hypot(r, z)  # diagonal distance shoulder → target

        # Clamp unreachable targets
        if D > (self.l1 + self.l2):
            D = self.l1 + self.l2 - 0.1

        # Elbow angle via law of cosines
        cos_elbow = (self.l1 ** 2 + self.l2 ** 2 - D ** 2) / (
            2 * self.l1 * self.l2
        )
        # Clamp for floating-point safety
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_rad = math.acos(cos_elbow)

        # Shoulder angle
        alpha = math.atan2(z, r)
        cos_beta = (self.l1 ** 2 + D ** 2 - self.l2 ** 2) / (
            2 * self.l1 * D
        )
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        shoulder_rad = alpha + beta

        return (
            math.degrees(base_rad),
            math.degrees(shoulder_rad),
            math.degrees(elbow_rad),
        )

    # ------------------------------------------------------------------
    # Forward Kinematics
    # ------------------------------------------------------------------

    def forward_kinematics(
        self, base_deg: float, shoulder_deg: float, elbow_deg: float
    ) -> tuple[float, float, float]:
        """Compute 3D tip position from joint angles.

        Parameters
        ----------
        base_deg, shoulder_deg, elbow_deg : float
            Joint angles in degrees.

        Returns
        -------
        (x, y, z) : tuple[float, float, float]
            Tip position in mm relative to the shoulder joint origin.
        """
        base_rad = math.radians(base_deg)
        shoulder_rad = math.radians(shoulder_deg)
        elbow_rad = math.radians(elbow_deg)

        # Elbow angle is the interior angle between L1 and L2.
        # Total angle from horizontal for the second link:
        total_angle = shoulder_rad - (math.pi - elbow_rad)

        # Tip in the shoulder's 2-D plane
        r = self.l1 * math.cos(shoulder_rad) + self.l2 * math.cos(total_angle)
        z = self.l1 * math.sin(shoulder_rad) + self.l2 * math.sin(total_angle)

        # Rotate around the base axis
        x = r * math.cos(base_rad)
        y = r * math.sin(base_rad)

        return x, y, z

    # ------------------------------------------------------------------
    # Angle ↔ Servo Position Conversion
    # ------------------------------------------------------------------

    def angles_to_servo_positions(
        self, base_deg: float, shoulder_deg: float, elbow_deg: float
    ) -> tuple[int, int, int]:
        """Convert joint angles to servo positions on the [0, 1000] scale.

        Formulas (from hardware calibration):
            base_pos     = 500 + (base_deg / 0.24)
            shoulder_pos = 500 + (shoulder_deg − 90) / 0.24
            elbow_pos    = 500 − ((180 − elbow_deg) / 0.24)

        All results are clamped to [0, 1000].
        """
        base_pos = int(500 + (base_deg / 0.24))
        shoulder_pos = int(500 + (shoulder_deg - 90) / 0.24)
        elbow_pos = int(500 - ((180 - elbow_deg) / 0.24))

        return (
            self._clamp(base_pos, 0, 1000),
            self._clamp(shoulder_pos, 0, 1000),
            self._clamp(elbow_pos, 0, 1000),
        )

    def servo_positions_to_angles(
        self, base_pos: int, shoulder_pos: int, elbow_pos: int
    ) -> tuple[float, float, float]:
        """Convert servo positions back to joint angles (degrees).

        Inverse of :meth:`angles_to_servo_positions`.
        """
        base_deg = (base_pos - 500) * 0.24
        shoulder_deg = (shoulder_pos - 500) * 0.24 + 90
        elbow_deg = 180 - (500 - elbow_pos) * 0.24

        return base_deg, shoulder_deg, elbow_deg

    # ------------------------------------------------------------------
    # Reachability
    # ------------------------------------------------------------------

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """Return True iff the target is within the arm's reach envelope.

        A point is reachable when ``hypot(hypot(x, y), z) ≤ L1 + L2``.
        """
        return math.hypot(math.hypot(x, y), z) <= self.l1 + self.l2

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp(value: int, lo: int, hi: int) -> int:
        """Clamp *value* to [lo, hi]. Idempotent."""
        return max(lo, min(hi, value))
