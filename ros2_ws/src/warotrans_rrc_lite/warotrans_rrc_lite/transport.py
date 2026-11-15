"""Byte transports for the RRC Lite link: real serial port (single owner) and an in-process mock board."""

from __future__ import annotations

import glob
import os
from typing import Optional, Protocol

from .protocol import BAUD_RATE

DEFAULT_PORT_GLOB = "/dev/serial/by-id/usb-1a86_*"


class TransportError(OSError):
    pass


class Transport(Protocol):
    description: str

    def open(self) -> None: ...

    def close(self) -> None: ...

    def read(self, max_bytes: int, timeout_s: float) -> bytes: ...

    def write(self, data: bytes) -> None: ...


def resolve_port(port: str) -> str:
    """`port` may be an exact path or a glob. A glob must match exactly one device (never guess)."""
    if not port:
        raise TransportError("serial port is empty")
    if any(ch in port for ch in "*?["):
        matches = sorted(glob.glob(port))
        if len(matches) != 1:
            raise TransportError(f"port pattern {port!r} matched {len(matches)} devices: {matches}")
        return matches[0]
    return port


class SerialTransport:
    """pyserial transport. Exclusive: TIOCEXCL via pyserial plus an flock, so a second driver (or the
    Hiwonder SDK, or a stray `screen`) cannot share the port and inject commands."""

    def __init__(self, port: str = DEFAULT_PORT_GLOB, baud: int = BAUD_RATE) -> None:
        self._port_spec = port
        self._baud = int(baud)
        self._serial = None
        self.description = port

    def open(self) -> None:
        import serial  # imported lazily: tests and mock mode do not need pyserial

        path = resolve_port(self._port_spec)
        self.description = f"{path} -> {os.path.realpath(path)}"
        s = serial.Serial(None, self._baud, timeout=0)
        # Same order as Hiwonder's SDK: keep DTR/RTS low across open.
        s.rts = False
        s.dtr = False
        s.port = path
        s.exclusive = True
        try:
            s.open()
        except serial.SerialException as exc:
            raise TransportError(f"cannot open {path}: {exc}") from exc
        try:
            import fcntl

            fcntl.flock(s.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except ImportError:  # non-POSIX dev machine
            pass
        except OSError as exc:
            s.close()
            raise TransportError(f"{path} is locked by another process: {exc}") from exc
        s.reset_input_buffer()
        self._serial = s

    def close(self) -> None:
        s, self._serial = self._serial, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        s = self._require()
        try:
            s.timeout = timeout_s
            first = s.read(1)
            if not first:
                return b""
            waiting = s.in_waiting
            rest = s.read(min(waiting, max_bytes - 1)) if waiting else b""
            return first + rest
        except Exception as exc:  # serial.SerialException, OSError, termios.error on unplug
            raise TransportError(f"read failed: {exc}") from exc

    def write(self, data: bytes) -> None:
        s = self._require()
        try:
            s.write(data)
            s.flush()
        except Exception as exc:
            raise TransportError(f"write failed: {exc}") from exc

    def _require(self):
        if self._serial is None:
            raise TransportError("port not open")
        return self._serial


class MockTransport:
    """Talks to a MockBoard in-process, byte for byte, so the full parse/encode path is exercised."""

    def __init__(self, board) -> None:
        self._board = board
        self.description = "mock://rrc_lite"
        self._open = False

    def open(self) -> None:
        if not self._board.connected:
            raise TransportError("mock board unplugged")
        self._open = True

    def close(self) -> None:
        self._open = False

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        self._check()
        return self._board.read_host(max_bytes, timeout_s)

    def write(self, data: bytes) -> None:
        self._check()
        self._board.write_host(data)

    def _check(self) -> None:
        if not self._open:
            raise TransportError("mock port not open")
        if not self._board.connected:
            self._open = False
            raise TransportError("mock board unplugged")


def make_transport(kind: str, port: str, baud: int, board: Optional[object] = None) -> Transport:
    if kind == "serial":
        return SerialTransport(port, baud)
    if kind == "mock":
        if board is None:
            raise ValueError("mock transport needs a board")
        return MockTransport(board)
    raise ValueError(f"unknown transport {kind!r} (serial|mock)")
