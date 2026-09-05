"""Serve the CartPole viewer and bridge browser commands to Gazebo Transport."""

import argparse
import json
import numbers
import os
import signal
import socket
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_pb2 import Entity
from gz.msgs.entity_wrench_pb2 import EntityWrench
from gz.msgs.world_control_pb2 import WorldControl
from gz.transport import Node

COMMAND_TOPIC = "/model/vehicle_green/force_cmd"
ENTITY_NAME = "vehicle_green::chassis"
MAX_FORCE = 2000.0
COMMAND_TIMEOUT = 0.2
PUBLISH_HZ = 50.0
WEBVIEW_WS_PORT = 9002
PACKAGE_ROOT = Path(__file__).resolve().parent
VISUALIZE_ROOT = PACKAGE_ROOT / "sim" / "visualize"


def ensure_tcp_port_available(port):
    """Fail before Gazebo starts when another process owns the WebView port."""

    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(("0.0.0.0", port))
    except OSError as error:
        raise RuntimeError(
            f"WebView port {port} is already in use. Stop the existing "
            "Gazebo / RL process; RL mode must be the only simulator."
        ) from error
    finally:
        probe.close()


def wait_for_tcp_listener(port, timeout=10.0):
    """Wait until the WebSocket system owned by this runtime is listening."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.settimeout(0.1)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.05)
    raise RuntimeError(
        f"Gazebo WebSocket did not start on port {port} within {timeout:.0f}s"
    )


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
            raise ValueError("force must be a number")  # noqa: TRY004 - API contract

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


async def _read_json(request: Request) -> dict:
    body = await request.body()
    if not body or len(body) > 4096:
        raise ValueError("invalid request size")
    payload = json.loads(body.decode("utf-8"))
    if not isinstance(payload, dict):
        raise TypeError("request body must be a JSON object")
    return payload


def _not_found() -> JSONResponse:
    return JSONResponse(status_code=404, content={"ok": False, "error": "not_found"})


def create_app(
    mode: str,
    *,
    runtime=None,
    force_publisher: ForcePublisher | None = None,
) -> FastAPI:
    """Build the HTTP API and mount the packaged WebView bundle."""

    if mode not in {"manual", "rl"}:
        raise ValueError(f"unsupported server mode: {mode}")
    if mode == "rl" and runtime is None:
        raise ValueError("runtime is required in rl mode")
    if mode == "manual" and force_publisher is None:
        raise ValueError("force_publisher is required in manual mode")
    if not VISUALIZE_ROOT.is_dir():
        raise RuntimeError(f"Visualization assets not found: {VISUALIZE_ROOT}")

    app = FastAPI(
        title="CartPole Control",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
    )

    @app.get("/api/health")
    def health():
        if mode == "rl":
            snapshot = runtime.snapshot()
            status = 503 if snapshot["phase"] == "ERROR" else 200
            return JSONResponse(
                status_code=status,
                content={
                    "ok": snapshot["phase"] != "ERROR",
                    "mode": "rl",
                    "checkpoint": str(runtime.checkpoint),
                    "phase": snapshot["phase"],
                    "ready": snapshot["ready"],
                    "websocket_port": WEBVIEW_WS_PORT,
                    "error": snapshot["error"],
                },
            )

        return {
            "ok": True,
            "mode": "manual",
            "topic": COMMAND_TOPIC,
            "force": force_publisher.current_force(),
        }

    @app.get("/api/rl/status")
    def rl_status():
        if mode != "rl":
            return _not_found()
        return runtime.snapshot()

    @app.post("/api/rl/start")
    def rl_start():
        if mode != "rl":
            return _not_found()
        try:
            runtime.start_episode()
        except RuntimeError as error:
            return JSONResponse(
                status_code=409,
                content={"ok": False, "error": str(error)},
            )
        return {"ok": True, "accepted": True}

    @app.post("/api/cartpole/reset")
    def reset():
        if mode == "rl":
            try:
                runtime.reset()
            except RuntimeError as error:
                return JSONResponse(
                    status_code=409,
                    content={"ok": False, "error": str(error)},
                )
            return {"ok": True, "accepted": True}

        try:
            reset_result = force_publisher.reset()
        except RuntimeError as error:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "error": str(error)},
            )
        return JSONResponse(
            status_code=200 if reset_result else 503,
            content={"ok": reset_result},
        )

    @app.post("/api/rl/disturbance")
    async def rl_disturbance(request: Request):
        if mode != "rl":
            return _not_found()
        try:
            payload = await _read_json(request)
            runtime.apply_disturbance(payload["direction"])
        except (KeyError, TypeError, ValueError) as error:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "error": str(error)},
            )
        except RuntimeError as error:
            return JSONResponse(
                status_code=409,
                content={"ok": False, "error": str(error)},
            )
        return {"ok": True, "accepted": True}

    @app.post("/api/cartpole/command")
    async def cartpole_command(request: Request):
        if mode == "rl":
            return JSONResponse(
                status_code=409,
                content={
                    "ok": False,
                    "error": "manual control is disabled in rl mode",
                },
            )
        try:
            payload = await _read_json(request)
            force = force_publisher.set_force(payload["force"])
        except (KeyError, TypeError, ValueError) as error:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "error": str(error)},
            )
        return {"ok": True, "force": force}

    # Mount last so API routes take precedence over the catch-all static route.
    app.mount(
        "/",
        StaticFiles(directory=VISUALIZE_ROOT, html=True),
        name="visualize",
    )
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8009)
    parser.add_argument("--mode", choices=("manual", "rl"), default="manual")
    parser.add_argument(
        "--checkpoint",
        default="src/checkpoints/ppo_cartpole.zip",
        help="PPO checkpoint used only when --mode rl.",
    )
    args = parser.parse_args()

    controller = None
    rl_runtime = None
    try:
        if args.mode == "rl":
            ensure_tcp_port_available(WEBVIEW_WS_PORT)
            os.environ["GZ_PARTITION"] = f"amr_cartpole_rl_{os.getpid()}"
            from cartpole.runtime import RLRuntime

            rl_runtime = RLRuntime(args.checkpoint)
        else:
            controller = ForcePublisher()

        app = create_app(
            args.mode,
            runtime=rl_runtime,
            force_publisher=controller,
        )

        print(f"Serving {VISUALIZE_ROOT} on http://{args.host}:{args.port}")
        if args.mode == "rl":
            print(f"Loading PPO checkpoint {rl_runtime.checkpoint}")
            # Gazebo may replace process signal handlers while TestFixture is
            # initialized on the worker. Restore normal terminal shutdown.
            snapshot = rl_runtime.wait_until_initialized(30.0)
            if snapshot["phase"] == "ERROR":
                raise RuntimeError(snapshot["error"])
            if not snapshot["ready"]:
                raise RuntimeError("RL runtime did not initialize within 30s")
            wait_for_tcp_listener(WEBVIEW_WS_PORT)
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.default_int_handler)
        else:
            print(f"Publishing force commands on {COMMAND_TOPIC}")

        uvicorn.run(
            app,
            host=args.host,
            port=args.port,
            access_log=False,
        )
    except KeyboardInterrupt:
        print("Stopping control bridge")
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    finally:
        if controller is not None:
            controller.close()
        if rl_runtime is not None:
            rl_runtime.close()


if __name__ == "__main__":
    main()
