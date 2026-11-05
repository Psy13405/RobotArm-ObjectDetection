"""Robot Arm 3D Controller — application entry point.

Wires together IKEngine, ArmState, SerialController, and
ArmVisualization, then starts the interactive 3D event loop.
"""

from __future__ import annotations

import argparse
import logging

from src.ik_engine import IKEngine
from src.arm_state import ArmState
from src.serial_controller import SerialController
from src.arm_visualization import ArmVisualization

DEFAULT_PORT = "/dev/ttyACM0"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="Robot Arm 3D Controller")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port for BusLinker V3.0 (default: {DEFAULT_PORT})",
    )
    args = parser.parse_args()

    # 1. Core math engine
    ik_engine = IKEngine(75.0, 85.0)

    # 2. Centralized arm state
    arm_state = ArmState(ik_engine)

    # 3. Serial connection (graceful failure → visualization-only mode)
    serial_controller = SerialController(args.port)
    serial_controller.connect()

    # 4. Wire state changes → servo commands
    def on_state_change() -> None:
        base_pos, shoulder_pos, elbow_pos = arm_state.get_servo_positions()
        serial_controller.move_all(base_pos, shoulder_pos, elbow_pos, time_ms=100)

    arm_state.on_change(on_state_change)

    # 5. Visualization
    viz = ArmVisualization(arm_state, serial_controller)
    viz.setup_scene()
    viz.add_joint_sliders()
    viz.add_xyz_input()
    viz.enable_drag_interaction()

    try:
        viz.run()  # blocks until window closed
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    finally:
        serial_controller.disconnect()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()
