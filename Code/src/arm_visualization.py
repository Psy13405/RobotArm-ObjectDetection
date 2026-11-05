"""ArmVisualization — PyVista-based 3D view for the robot arm.

Renders the 3-axis arm as connected cylinders with interactive controls.
Uses PyVista sphere widgets for draggable tip and elbow joints.
"""

from __future__ import annotations

import math
import signal
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
import pyvista as pv

if TYPE_CHECKING:
    from src.arm_state import ArmState
    from src.serial_controller import SerialController


class ArmVisualization:
    """3D visualization of the robot arm using PyVista."""

    L1: float = 75.0
    L2: float = 85.0

    _BASE_RADIUS: float = 20.0
    _BASE_HEIGHT: float = 10.0
    _LINK_RADIUS: float = 6.0
    _JOINT_RADIUS: float = 8.0

    def __init__(
        self,
        arm_state: ArmState,
        serial_controller: SerialController | None = None,
    ) -> None:
        self._arm_state = arm_state
        self._serial_controller = serial_controller
        self._plotter: pv.Plotter | None = None

        # Mesh actors
        self._base_actor = None
        self._shoulder_actor = None
        self._elbow_actor = None
        self._shoulder_joint_actor = None
        self._status_actor = None
        self._telemetry_actor = None

        self._updating = False
        self._closed = False

        # Telemetry polling
        self._telemetry_thread: threading.Thread | None = None
        self._telemetry_stop = threading.Event()
        self._telemetry_data: dict = {}

        # Stored 3D positions
        self._shoulder_origin = np.array([0.0, 0.0, self._BASE_HEIGHT])
        self._elbow_pos_3d = np.zeros(3)
        self._tip_pos_3d = np.zeros(3)

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------

    def setup_scene(self) -> None:
        self._plotter = pv.Plotter(title="Robot Arm 3D Controller")
        self._plotter.set_background("white")

        grid = pv.Plane(
            center=(0, 0, 0), direction=(0, 0, 1),
            i_size=400, j_size=400, i_resolution=20, j_resolution=20,
        )
        self._plotter.add_mesh(
            grid, style="wireframe", color="lightgray", opacity=0.5
        )

        self._draw_arm()
        self._plotter.camera_position = "iso"
        self._plotter.reset_camera()

    # ------------------------------------------------------------------
    # Arm mesh rendering (links + shoulder joint only, not tip/elbow)
    # ------------------------------------------------------------------

    def _compute_positions(self):
        """Compute shoulder_end and elbow_end from current arm state."""
        base_rad = math.radians(self._arm_state.base_deg)
        shoulder_rad = math.radians(self._arm_state.shoulder_deg)
        elbow_rad = math.radians(self._arm_state.elbow_deg)

        shoulder_end = self._shoulder_origin + np.array([
            self.L1 * math.cos(shoulder_rad) * math.cos(base_rad),
            self.L1 * math.cos(shoulder_rad) * math.sin(base_rad),
            self.L1 * math.sin(shoulder_rad),
        ])

        total_angle = shoulder_rad - (math.pi - elbow_rad)
        elbow_end = shoulder_end + np.array([
            self.L2 * math.cos(total_angle) * math.cos(base_rad),
            self.L2 * math.cos(total_angle) * math.sin(base_rad),
            self.L2 * math.sin(total_angle),
        ])

        self._elbow_pos_3d = shoulder_end.copy()
        self._tip_pos_3d = elbow_end.copy()
        return shoulder_end, elbow_end

    def _draw_arm(self) -> None:
        """Draw the arm links and shoulder joint (not the draggable spheres)."""
        if self._plotter is None or self._closed:
            return

        for actor in (
            self._base_actor, self._shoulder_actor,
            self._elbow_actor, self._shoulder_joint_actor,
        ):
            if actor is not None:
                self._plotter.remove_actor(actor)

        shoulder_end, elbow_end = self._compute_positions()

        # Base platform
        base_mesh = pv.Cylinder(
            center=(0, 0, self._BASE_HEIGHT / 2), direction=(0, 0, 1),
            radius=self._BASE_RADIUS, height=self._BASE_HEIGHT, resolution=30,
        )
        self._base_actor = self._plotter.add_mesh(
            base_mesh, color="slategray", smooth_shading=True
        )

        def _cyl(p0, p1, r):
            d = p1 - p0
            length = max(float(np.linalg.norm(d)), 1e-6)
            return pv.Cylinder(
                center=((p0 + p1) / 2).tolist(), direction=d.tolist(),
                radius=r, height=length, resolution=20,
            )

        # Shoulder link
        self._shoulder_actor = self._plotter.add_mesh(
            _cyl(self._shoulder_origin, shoulder_end, self._LINK_RADIUS),
            color="steelblue", smooth_shading=True,
        )
        # Elbow link
        self._elbow_actor = self._plotter.add_mesh(
            _cyl(shoulder_end, elbow_end, self._LINK_RADIUS),
            color="coral", smooth_shading=True,
        )
        # Shoulder joint (fixed, not draggable)
        self._shoulder_joint_actor = self._plotter.add_mesh(
            pv.Sphere(radius=self._JOINT_RADIUS, center=self._shoulder_origin.tolist()),
            color="gold", smooth_shading=True,
        )

    # ------------------------------------------------------------------
    # Draggable sphere widgets for tip and elbow
    # ------------------------------------------------------------------

    def enable_drag_interaction(self) -> None:
        """Add draggable sphere widgets at the tip and elbow positions."""
        if self._plotter is None:
            return

        shoulder_end, elbow_end = self._compute_positions()

        # Tip sphere widget (red) — drives IK
        self._plotter.add_sphere_widget(
            callback=self._on_tip_dragged,
            center=elbow_end.tolist(),
            radius=self._JOINT_RADIUS * 1.2,
            color="red",
            style="surface",
            test_callback=False,
        )

        # Elbow sphere widget (gold) — drives shoulder angle
        self._plotter.add_sphere_widget(
            callback=self._on_elbow_dragged,
            center=shoulder_end.tolist(),
            radius=self._JOINT_RADIUS * 1.2,
            color="gold",
            style="surface",
            test_callback=False,
        )

    def _on_tip_dragged(self, point: np.ndarray) -> None:
        """Called when the tip sphere widget is moved."""
        if self._updating or self._closed:
            return
        x, y, z = float(point[0]), float(point[1]), float(point[2]) - self._BASE_HEIGHT
        self._arm_state.set_from_xyz(x, y, z)

    def _on_elbow_dragged(self, point: np.ndarray) -> None:
        """Called when the elbow sphere widget is moved."""
        if self._updating or self._closed:
            return
        rel = np.array(point) - self._shoulder_origin
        base_rad = math.radians(self._arm_state.base_deg)
        cos_b, sin_b = math.cos(base_rad), math.sin(base_rad)
        r = float(rel[0]) * cos_b + float(rel[1]) * sin_b
        z = float(rel[2])
        dist = math.hypot(r, z)
        if dist < 1e-6:
            return
        if dist > self.L1:
            scale = self.L1 / dist
            r *= scale
            z *= scale
        shoulder_deg = math.degrees(math.atan2(z, r))
        shoulder_deg = max(0.0, min(180.0, shoulder_deg))
        self._arm_state.set_joint_angle("shoulder_deg", shoulder_deg)

    # ------------------------------------------------------------------
    # Joint angle sliders
    # ------------------------------------------------------------------

    def add_joint_sliders(self) -> None:
        if self._plotter is None:
            return

        def _on_base(v):
            if not self._updating:
                self._arm_state.set_joint_angle("base_deg", v)

        def _on_shoulder(v):
            if not self._updating:
                self._arm_state.set_joint_angle("shoulder_deg", v)

        def _on_elbow(v):
            if not self._updating:
                self._arm_state.set_joint_angle("elbow_deg", v)

        self._base_slider = self._plotter.add_slider_widget(
            _on_base, rng=(-120, 120), value=self._arm_state.base_deg,
            title="Base", pointa=(0.02, 0.90), pointb=(0.35, 0.90), style="modern")
        self._shoulder_slider = self._plotter.add_slider_widget(
            _on_shoulder, rng=(0, 180), value=self._arm_state.shoulder_deg,
            title="Shoulder", pointa=(0.02, 0.82), pointb=(0.35, 0.82), style="modern")
        self._elbow_slider = self._plotter.add_slider_widget(
            _on_elbow, rng=(0, 180), value=self._arm_state.elbow_deg,
            title="Elbow", pointa=(0.02, 0.74), pointb=(0.35, 0.74), style="modern")

        self._arm_state.on_change(self._on_arm_state_changed)

    def _on_arm_state_changed(self) -> None:
        if self._updating or self._closed:
            return
        self._updating = True
        try:
            self._draw_arm()
            self._sync_sliders()
            self._sync_xyz_sliders()
            self._update_sphere_widgets()
            self._update_status_indicator()
            self._update_telemetry_display()
            if self._plotter is not None:
                self._plotter.render()
        finally:
            self._updating = False

    def _sync_sliders(self) -> None:
        try:
            self._base_slider.GetRepresentation().SetValue(self._arm_state.base_deg)
            self._shoulder_slider.GetRepresentation().SetValue(self._arm_state.shoulder_deg)
            self._elbow_slider.GetRepresentation().SetValue(self._arm_state.elbow_deg)
        except Exception:
            pass

    def _sync_xyz_sliders(self) -> None:
        try:
            tip = self._arm_state.get_tip_position()
            self._x_slider.GetRepresentation().SetValue(tip[0])
            self._y_slider.GetRepresentation().SetValue(tip[1])
            self._z_slider.GetRepresentation().SetValue(tip[2])
        except Exception:
            pass

    def _update_sphere_widgets(self) -> None:
        """Move the sphere widgets to match the current arm state."""
        if self._plotter is None:
            return
        try:
            shoulder_end, elbow_end = self._compute_positions()
            # Sphere widgets are stored in plotter.sphere_widgets list
            widgets = self._plotter.sphere_widgets
            if len(widgets) >= 2:
                widgets[0].SetCenter(elbow_end.tolist())  # tip widget
                widgets[1].SetCenter(shoulder_end.tolist())  # elbow widget
        except Exception:
            pass

    # ------------------------------------------------------------------
    # XYZ coordinate input
    # ------------------------------------------------------------------

    def add_xyz_input(self) -> None:
        if self._plotter is None:
            return
        max_reach = self.L1 + self.L2
        tip = self._arm_state.get_tip_position()

        def _on_xyz(_v):
            if self._updating:
                return
            try:
                x = self._x_slider.GetRepresentation().GetValue()
                y = self._y_slider.GetRepresentation().GetValue()
                z = self._z_slider.GetRepresentation().GetValue()
            except Exception:
                return
            self._arm_state.set_from_xyz(x, y, z)

        self._x_slider = self._plotter.add_slider_widget(
            _on_xyz, rng=(-max_reach, max_reach), value=tip[0],
            title="X (mm)", pointa=(0.65, 0.90), pointb=(0.98, 0.90), style="modern")
        self._y_slider = self._plotter.add_slider_widget(
            _on_xyz, rng=(-max_reach, max_reach), value=tip[1],
            title="Y (mm)", pointa=(0.65, 0.82), pointb=(0.98, 0.82), style="modern")
        self._z_slider = self._plotter.add_slider_widget(
            _on_xyz, rng=(-max_reach, max_reach), value=tip[2],
            title="Z (mm)", pointa=(0.65, 0.74), pointb=(0.98, 0.74), style="modern")

    # ------------------------------------------------------------------
    # Serial connection status indicator
    # ------------------------------------------------------------------

    def _update_status_indicator(self) -> None:
        if self._plotter is None or self._closed:
            return
        if self._status_actor is not None:
            self._plotter.remove_actor(self._status_actor)
            self._status_actor = None

        if self._serial_controller is not None:
            connected = self._serial_controller.is_connected()
            label = "Serial: Connected" if connected else "Serial: Disconnected"
            color = "green" if connected else "red"
        else:
            label = "Serial: N/A"
            color = "gray"

        self._status_actor = self._plotter.add_text(
            label, position="lower_left", font_size=10, color=color,
            name="serial_status")

    # ------------------------------------------------------------------
    # Servo telemetry display
    # ------------------------------------------------------------------

    def start_telemetry(self) -> None:
        """Start a background thread that polls servo telemetry every 2s."""
        if self._serial_controller is None:
            return
        self._telemetry_stop.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True
        )
        self._telemetry_thread.start()

    def _telemetry_loop(self) -> None:
        """Background loop: read position + temperature from all servos."""
        while not self._telemetry_stop.is_set():
            if self._serial_controller is not None and self._serial_controller.is_connected():
                try:
                    self._telemetry_data = self._serial_controller.read_all_telemetry()
                except Exception:
                    pass
            self._telemetry_stop.wait(timeout=2.0)

    def _update_telemetry_display(self) -> None:
        """Refresh the servo telemetry text overlay."""
        if self._plotter is None or self._closed:
            return
        if self._telemetry_actor is not None:
            self._plotter.remove_actor(self._telemetry_actor)
            self._telemetry_actor = None

        if not self._telemetry_data:
            return

        lines = []
        for name in ("base", "shoulder", "elbow"):
            info = self._telemetry_data.get(name, {})
            sid = info.get("id", "?")
            pos = info.get("position")
            temp = info.get("temperature")
            pos_str = str(pos) if pos is not None else "--"
            temp_str = f"{round(((temp*1.8)+32), 2)}°F" if temp is not None else "--"
            lines.append(f"{name.capitalize()} (ID {sid}): pos={pos_str}  temp={temp_str}")

        text = "\n".join(lines)
        self._telemetry_actor = self._plotter.add_text(
            text, position=(10, 40), font_size=20, color="black",
            name="telemetry_info",
        )

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    def run(self) -> None:
        if self._plotter is None:
            raise RuntimeError("Call setup_scene() before run().")

        self._update_status_indicator()
        self._update_telemetry_display()
        self.start_telemetry()

        # Ctrl+C closes the window gracefully
        original_sigint = signal.getsignal(signal.SIGINT)

        def _sigint_handler(sig, frame):
            self._closed = True
            self._telemetry_stop.set()
            if self._plotter is not None:
                try:
                    self._plotter.close()
                except Exception:
                    pass

        signal.signal(signal.SIGINT, _sigint_handler)

        try:
            self._plotter.show()
        finally:
            self._telemetry_stop.set()
            signal.signal(signal.SIGINT, original_sigint)
