#!/usr/bin/env python3
"""
Example client for sending forward/lateral/yaw commands to the Swarm Control Tower GUI.

The GUI listens on tcp://localhost:5558 (ZMQ PULL socket).
Send a JSON message with the drone ID and any combination of forward, lateral, yaw.

Values use the same scale as the GUI spinboxes:
    forward  : -2.0 … 2.0
    lateral  : -2.0 … 2.0
    yaw      : -180.0 … 180.0
"""

import zmq
import time


def make_sender(address: str = "tcp://localhost:5558") -> zmq.Socket:
    """Create and connect a ZMQ PUSH socket."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.connect(address)
    return sock


def send_command(sock: zmq.Socket, drone_id: int, *,
                 forward: float | None = None,
                 lateral: float | None = None,
                 yaw: float | None = None):
    """Send a command to update one or more motion parameters for a drone.

    Only the parameters you specify will be changed; the rest stay as-is.
    """
    msg = {"droneId": drone_id}
    if forward is not None:
        msg["forward"] = forward
    if lateral is not None:
        msg["lateral"] = lateral
    if yaw is not None:
        msg["yaw"] = yaw
    sock.send_json(msg)


# ── Example usage ──────────────────────────────────────────────
if __name__ == "__main__":
    sock = make_sender()

    # Give ZMQ a moment to finish connecting
    time.sleep(0.1)

    # Send some test commands to drone #8
    send_command(sock, drone_id=8, forward=0.5, lateral=-0.3, yaw=45.0)
    print("Sent: forward=0.5, lateral=-0.3, yaw=45.0 to drone #8")

    time.sleep(1)

    send_command(sock, drone_id=8, forward=0.0, lateral=0.0, yaw=0.0)
    print("Sent: forward=0.0, lateral=0.0, yaw=0.0 to drone #8")
