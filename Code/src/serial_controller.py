"""SerialController — Hardware communication with BusLinker V3.0.

Manages serial connection to Hiwonder HX-35H servos, builds and sends
10-byte servo packets (0x55 0x55 header + checksum), throttles commands
to prevent bus saturation, and handles disconnection with auto-reconnect.
"""

from __future__ import annotations

import logging
import threading
import time

import serial

logger = logging.getLogger(__name__)


class SerialController:
    """Manages serial communication with the BusLinker V3.0 hardware.

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``"/dev/cu.usbmodem5B3E1210981"``).
    baud : int
        Baud rate (default 115200).
    """

    SERVO_IDS: dict[str, int] = {"base": 2, "shoulder": 3, "elbow": 1}
    MIN_INTERVAL_S: float = 0.050  # 50 ms throttle interval

    def __init__(self, port: str, baud: int = 115200) -> None:
        self._port_path = port
        self._baud = baud
        self._serial: serial.Serial | None = None
        self._connected = False
        self._lock = threading.Lock()  # serialize all serial port access

        # Throttling state (Task 4.2)
        self._last_send_time: float = 0.0

        # Auto-reconnect state (Task 4.3)
        self._reconnect_thread: threading.Thread | None = None
        self._reconnect_stop = threading.Event()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the serial port. Returns True on success, False otherwise.

        On failure the application continues in visualization-only mode.
        """
        try:
            self._serial = serial.Serial(
                self._port_path, self._baud, timeout=0.1
            )
            self._connected = True
            self._stop_reconnect_thread()
            logger.info("Serial port %s opened successfully.", self._port_path)
            return True
        except (serial.SerialException, OSError) as exc:
            logger.warning(
                "Could not open serial port %s: %s. "
                "Running in visualization-only mode.",
                self._port_path,
                exc,
            )
            self._serial = None
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Close the serial port and stop any reconnect attempts."""
        self._stop_reconnect_thread()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001
                pass
        self._serial = None
        self._connected = False
        logger.info("Serial port disconnected.")

    def is_connected(self) -> bool:
        """Return True if the serial port is currently open and usable."""
        return self._connected

    # ------------------------------------------------------------------
    # Servo packet construction & sending
    # ------------------------------------------------------------------

    @staticmethod
    def build_packet(
        servo_id: int, position: int, time_ms: int
    ) -> bytearray:
        """Build a 10-byte HX-35H servo packet.

        Packet layout::

            [0x55, 0x55, servo_id, length, cmd,
             pos_lsb, pos_msb, time_lsb, time_msb, checksum]

        Parameters
        ----------
        servo_id : int
            Target servo ID.
        position : int
            Servo position (will be clamped to [0, 1000]).
        time_ms : int
            Move duration in milliseconds (will be clamped to [0, 30000]).
        """
        # Clamp inputs
        position = max(0, min(1000, position))
        time_ms = max(0, min(30000, time_ms))

        length = 7
        cmd = 1  # Move Time Write
        pos_lsb = position & 0xFF
        pos_msb = (position >> 8) & 0xFF
        time_lsb = time_ms & 0xFF
        time_msb = (time_ms >> 8) & 0xFF
        checksum = (
            ~(servo_id + length + cmd + pos_lsb + pos_msb + time_lsb + time_msb)
        ) & 0xFF

        return bytearray(
            [0x55, 0x55, servo_id, length, cmd,
             pos_lsb, pos_msb, time_lsb, time_msb, checksum]
        )

    def move_servo(
        self, servo_id: int, position: int, time_ms: int = 100
    ) -> None:
        """Build and send a servo packet for a single servo."""
        if not self._connected or self._serial is None:
            return

        if position < 0 or position > 1000:
            logger.warning("Servo %d position %d clamped to [0, 1000].", servo_id, position)
        if time_ms < 0 or time_ms > 30000:
            logger.warning("Servo %d time_ms %d clamped to [0, 30000].", servo_id, time_ms)

        packet = self.build_packet(servo_id, position, time_ms)

        with self._lock:
            try:
                self._serial.write(packet)
            except serial.SerialException as exc:
                logger.error("Serial write failed: %s", exc)
                self._handle_disconnection()

    def move_all(
        self,
        base_pos: int,
        shoulder_pos: int,
        elbow_pos: int,
        time_ms: int = 100,
    ) -> None:
        """Send servo packets for all three joints.

        Servo ID mapping: base=2, shoulder=3, elbow=1.

        Enforces a 50 ms throttle interval — calls within 50 ms of the
        previous send are skipped so the latest positions are always sent
        on the next eligible call (no stale queuing).
        """
        if not self._connected:
            return

        # Throttle: skip if within MIN_INTERVAL_S of last send
        now = time.monotonic()
        if (now - self._last_send_time) < self.MIN_INTERVAL_S:
            return
        self._last_send_time = now

        self.move_servo(self.SERVO_IDS["base"], base_pos, time_ms)
        self.move_servo(self.SERVO_IDS["shoulder"], shoulder_pos, time_ms)
        self.move_servo(self.SERVO_IDS["elbow"], elbow_pos, time_ms)

    # ------------------------------------------------------------------
    # Servo read commands (position & temperature)
    # ------------------------------------------------------------------

    def _send_read_cmd(self, servo_id: int, cmd: int) -> bytearray | None:
        """Send a read command and return the response data bytes, or None.

        Must be called with self._lock held or will acquire it.
        """
        if not self._connected or self._serial is None:
            return None

        length = 3
        checksum = (~(servo_id + length + cmd)) & 0xFF
        packet = bytearray([0x55, 0x55, servo_id, length, cmd, checksum])

        with self._lock:
            try:
                self._serial.reset_input_buffer()
                self._serial.write(packet)
                time.sleep(0.01)  # give servo time to respond
                # Response: 0x55 0x55 servo_id length cmd data... checksum
                header = self._serial.read(5)
                if len(header) < 5 or header[0] != 0x55 or header[1] != 0x55:
                    return None
                resp_len = header[3]
                remaining = resp_len - 3 + 1  # data bytes + checksum
                data = self._serial.read(remaining)
                if len(data) < remaining:
                    return None
                return data
            except serial.SerialException as exc:
                logger.error("Serial read failed: %s", exc)
                self._handle_disconnection()
                return None

    def read_position(self, servo_id: int) -> int | None:
        """Read the current position (0-1000) from a servo. Returns None on failure."""
        data = self._send_read_cmd(servo_id, 28)  # cmd 28 = Pos Read
        if data is None or len(data) < 2:
            return None
        return data[0] | (data[1] << 8)

    def read_temperature(self, servo_id: int) -> int | None:
        """Read the current temperature (°C) from a servo. Returns None on failure."""
        data = self._send_read_cmd(servo_id, 26)  # cmd 26 = Temp Read
        if data is None or len(data) < 1:
            return None
        return data[0]

    def read_all_telemetry(self) -> dict[str, dict]:
        """Read position and temperature from all servos.

        Returns a dict like::

            {"base": {"id": 2, "position": 500, "temperature": 35},
             "shoulder": {"id": 3, "position": 450, "temperature": 37},
             "elbow": {"id": 1, "position": 600, "temperature": 33}}
        """
        result = {}
        for name, sid in self.SERVO_IDS.items():
            pos = self.read_position(sid)
            temp = self.read_temperature(sid)
            result[name] = {
                "id": sid,
                "position": pos,
                "temperature": temp,
            }
        return result

    # ------------------------------------------------------------------
    # Error handling & auto-reconnect  (Task 4.3)
    # ------------------------------------------------------------------

    def _handle_disconnection(self) -> None:
        """Mark connection as lost and start background reconnect."""
        self._connected = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001
                pass
            self._serial = None
        logger.warning("Serial connection lost. Starting auto-reconnect.")
        self._start_reconnect_thread()

    def _start_reconnect_thread(self) -> None:
        """Spawn a daemon thread that attempts reconnection every 2 s."""
        if (
            self._reconnect_thread is not None
            and self._reconnect_thread.is_alive()
        ):
            return  # already running
        self._reconnect_stop.clear()
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop, daemon=True
        )
        self._reconnect_thread.start()

    def _stop_reconnect_thread(self) -> None:
        """Signal the reconnect thread to stop and wait for it."""
        self._reconnect_stop.set()
        if (
            self._reconnect_thread is not None
            and self._reconnect_thread.is_alive()
        ):
            self._reconnect_thread.join(timeout=3)
        self._reconnect_thread = None

    def _reconnect_loop(self) -> None:
        """Background loop: try to reconnect every 2 seconds."""
        while not self._reconnect_stop.is_set():
            logger.info("Attempting serial reconnect to %s …", self._port_path)
            if self.connect():
                logger.info("Reconnected to %s.", self._port_path)
                return
            self._reconnect_stop.wait(timeout=2.0)
