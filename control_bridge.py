#!/usr/bin/env python3
"""Serve the CartPole viewer and bridge browser commands to Gazebo Transport."""

import argparse
import json
import numbers
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from gz.msgs.entity_pb2 import Entity
from gz.msgs.entity_wrench_pb2 import EntityWrench
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.world_control_pb2 import WorldControl
from gz.transport import Node


COMMAND_TOPIC = "/model/vehicle_green/force_cmd"
ENTITY_NAME = "vehicle_green::chassis"
MAX_FORCE = 2000.0
COMMAND_TIMEOUT = 0.2
PUBLISH_HZ = 50.0


class ForcePublisher:
    """Publish the latest force command with a short dead-man timeout."""

    def __init__(self):
        self.node = Node()
        self.publisher = self.node.advertise(COMMAND_TOPIC, EntityWrench)
        self.lock = threading.Lock()
        self.force = 0.0
        self.last_command = time.monotonic()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()

    def set_force(self, value):
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ValueError("force must be a number")

        force = max(-MAX_FORCE, min(MAX_FORCE, float(value)))
        with self.lock:
            self.force = force
            self.last_command = time.monotonic()
        return force

    def current_force(self):
        with self.lock:
            if time.monotonic() - self.last_command > COMMAND_TIMEOUT:
                return 0.0
            return self.force

    def _publish(self, force):
        message = EntityWrench()
        message.entity.name = ENTITY_NAME
        message.entity.type = Entity.LINK
        message.wrench.force.x = force
        self.publisher.publish(message)

    def _publish_loop(self):
        period = 1.0 / PUBLISH_HZ
        while not self.stopped.is_set():
            started = time.monotonic()
            self._publish(self.current_force())
            self.stopped.wait(max(0.0, period - (time.monotonic() - started)))

    def close(self):
        self.stopped.set()
        self._publish(0.0)
        self.thread.join(timeout=1.0)

    def reset(self):
        self.set_force(0.0)
        request = WorldControl()
        request.reset.all = True
        executed, response = self.node.request(
            "/world/cart_pole/control", request, WorldControl, Boolean, 2000
        )
        if executed and response.data:
            time.sleep(0.05)
        return executed and response.data


class ViewerHandler(SimpleHTTPRequestHandler):
    server_version = "CartPoleControl/1.0"

    def _send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.end_headers()

    def do_GET(self):
        if self.path == "/api/health":
            self._send_json(200, {
                "ok": True,
                "topic": COMMAND_TOPIC,
                "force": self.server.force_publisher.current_force(),
            })
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/cartpole/reset":
            try:
                reset = self.server.force_publisher.reset()
            except RuntimeError as error:
                self._send_json(503, {"ok": False, "error": str(error)})
                return
            self._send_json(200 if reset else 503, {"ok": reset})
            return

        if self.path != "/api/cartpole/command":
            self._send_json(404, {"ok": False, "error": "not_found"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 4096:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            force = self.server.force_publisher.set_force(payload["force"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._send_json(400, {"ok": False, "error": str(error)})
            return

        self._send_json(200, {"ok": True, "force": force})

    def log_message(self, format_string, *args):
        if self.path.startswith("/api/"):
            super().log_message(format_string, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8009)
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent
    controller = ForcePublisher()
    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    server.force_publisher = controller

    print(f"Serving {project_root} on http://{args.host}:{args.port}")
    print(f"Publishing force commands on {COMMAND_TOPIC}")

    try:
        # SimpleHTTPRequestHandler serves from the process working directory.
        # The launcher is expected to be run from the project root.
        import os
        os.chdir(project_root)
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping control bridge")
    finally:
        controller.close()
        server.server_close()


if __name__ == "__main__":
    main()
