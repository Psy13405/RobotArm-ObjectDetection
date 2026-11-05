"""ArmState — Centralized model for the robot arm's joint configuration.

Holds joint angles in degrees, validates against mechanical limits,
delegates all math (IK, FK, servo conversion) to IKEngine, and
notifies registered listeners on every state change.
"""

from __future__ import annotations

from typing import Callable

from src.ik_engine import IKEngine


class ArmState:
    """Single source of truth for the arm's joint configuration.

    Parameters
    ----------
    ik_engine : IKEngine, optional
        IKEngine instance used for all math operations.
        A default ``IKEngine(75.0, 85.0)`` is created when *None*.
    """

    # Physical constants (mm)
    L1: float = 75.0
    L2: float = 85.0

    # Mechanical limits (degrees)
    LIMITS: dict[str, tuple[float, float]] = {
        "base_deg": (-120.0, 120.0),
        "shoulder_deg": (0.0, 180.0),
        "elbow_deg": (0.0, 180.0),
    }

    def __init__(self, ik_engine: IKEngine | None = None) -> None:
        self._ik = ik_engine or IKEngine(self.L1, self.L2)

        # Joint angles — start at a safe neutral pose
        self._base_deg: float = 0.0
        self._shoulder_deg: float = 90.0
        self._elbow_deg: float = 90.0

        # Registered change listeners
        self._listeners: list[Callable[[], None]] = []

    # ------------------------------------------------------------------
    # Properties (read-only access to joint angles)
    # ------------------------------------------------------------------

    @property
    def base_deg(self) -> float:
        return self._base_deg

    @property
    def shoulder_deg(self) -> float:
        return self._shoulder_deg

    @property
    def elbow_deg(self) -> float:
        return self._elbow_deg

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_joint_angle(self, joint: str, degrees: float) -> None:
        """Set a single joint angle, validate, and notify listeners.

        Parameters
        ----------
        joint : str
            One of ``"base_deg"``, ``"shoulder_deg"``, or ``"elbow_deg"``.
        degrees : float
            Desired angle in degrees.  Clamped to the joint's mechanical
            limits before storing.

        Raises
        ------
        ValueError
            If *joint* is not a recognised joint name.
        """
        if joint not in self.LIMITS:
            raise ValueError(
                f"Unknown joint '{joint}'. "
                f"Expected one of {list(self.LIMITS.keys())}."
            )

        lo, hi = self.LIMITS[joint]
        clamped = max(lo, min(hi, degrees))

        setattr(self, f"_{joint}", clamped)
        self._notify()

    def set_from_xyz(self, x: float, y: float, z: float) -> None:
        """Move the arm to a target XYZ position via inverse kinematics.

        If the target is unreachable, the arm moves to the closest
        reachable point along the target direction (IKEngine handles
        the clamping internally).

        After computing angles, each is clamped to mechanical limits
        before storing.
        """
        base, shoulder, elbow = self._ik.calculate_ik(x, y, z)

        # Clamp each angle to mechanical limits
        lo_b, hi_b = self.LIMITS["base_deg"]
        lo_s, hi_s = self.LIMITS["shoulder_deg"]
        lo_e, hi_e = self.LIMITS["elbow_deg"]

        self._base_deg = max(lo_b, min(hi_b, base))
        self._shoulder_deg = max(lo_s, min(hi_s, shoulder))
        self._elbow_deg = max(lo_e, min(hi_e, elbow))

        self._notify()

    def get_servo_positions(self) -> tuple[int, int, int]:
        """Return current servo positions as ``(base, shoulder, elbow)``."""
        return self._ik.angles_to_servo_positions(
            self._base_deg, self._shoulder_deg, self._elbow_deg
        )

    def get_tip_position(self) -> tuple[float, float, float]:
        """Return the current 3D tip position via forward kinematics."""
        return self._ik.forward_kinematics(
            self._base_deg, self._shoulder_deg, self._elbow_deg
        )

    def on_change(self, callback: Callable[[], None]) -> None:
        """Register a listener that is called on every state change."""
        self._listeners.append(callback)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _notify(self) -> None:
        """Invoke every registered listener."""
        for cb in self._listeners:
            cb()
