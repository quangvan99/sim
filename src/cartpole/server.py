"""Serve the CartPole viewer and bridge browser commands to Gazebo Transport."""

import argparse
import json
import numbers
import os
import signal
import socket
import threading
import time

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_pb2 import Entity
from gz.msgs.entity_wrench_pb2 import EntityWrench
from gz.msgs.world_control_pb2 import WorldControl
from gz.transport import Node

from cartpole.config import CONFIG


def ensure_port_available(port: int) -> None:
    """Fail before Gazebo starts when its WebSocket port is already in use."""

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind(("", port))
        except OSError as error:
            raise RuntimeError(
                f"Port {port} is already in use. "
                "Stop the old CartPole/Gazebo process before starting a new one."
            ) from error


class ForcePublisher:
    """Publish the latest force command with a short dead-man timeout."""

    def __init__(self):
        self.node = Node()
        self.publisher = self.node.advertise(
            CONFIG.server.command_topic, EntityWrench
        )
        self.lock = threading.Lock()
        self.force = 0.0
        self.last_command = time.monotonic()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.thread.start()

    def set_force(self, value):
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ValueError("force must be a number")  # noqa: TRY004 - API contract

        force = max(
            -CONFIG.server.max_force_n,
            min(CONFIG.server.max_force_n, float(value)),
        )
        with self.lock:
            self.force = force
            self.last_command = time.monotonic()
        return force

    def current_force(self):
        with self.lock:
            if (
                time.monotonic() - self.last_command
                > CONFIG.server.command_timeout_seconds
            ):
                return 0.0
            return self.force

    def _publish(self, force):
        message = EntityWrench()
        message.entity.name = CONFIG.server.entity_name
        message.entity.type = Entity.LINK
        message.wrench.force.x = force
        self.publisher.publish(message)

    def _publish_loop(self):
        period = 1.0 / CONFIG.server.publish_hz
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
            f"/world/{CONFIG.cartpole.world_name}/control",
            request,
            WorldControl,
            Boolean,
            2000,
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


def _error_response(status_code: int, error: str) -> JSONResponse:
    return JSONResponse(
        status_code=status_code,
        content={"ok": False, "error": error},
    )


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
    visualize_root = CONFIG.paths.visualize_root
    if not visualize_root.is_dir():
        raise RuntimeError(f"Visualization assets not found: {visualize_root}")

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
                    "websocket_port": CONFIG.server.websocket_port,
                    "error": snapshot["error"],
                },
            )

        return {
            "ok": True,
            "mode": "manual",
            "topic": CONFIG.server.command_topic,
            "force": force_publisher.current_force(),
        }

    @app.get("/api/rl/status")
    def rl_status():
        if mode != "rl":
            return _error_response(404, "not_found")
        return runtime.snapshot()

    @app.post("/api/rl/start")
    def rl_start():
        if mode != "rl":
            return _error_response(404, "not_found")
        try:
            runtime.start_episode()
        except RuntimeError as error:
            return _error_response(409, str(error))
        return {"ok": True, "accepted": True}

    @app.post("/api/cartpole/reset")
    def reset():
        if mode == "rl":
            try:
                runtime.reset()
            except RuntimeError as error:
                return _error_response(409, str(error))
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
            return _error_response(404, "not_found")
        try:
            payload = await _read_json(request)
            runtime.apply_disturbance(payload["direction"])
        except (KeyError, TypeError, ValueError) as error:
            return _error_response(400, str(error))
        except RuntimeError as error:
            return _error_response(409, str(error))
        return {"ok": True, "accepted": True}

    @app.post("/api/cartpole/command")
    async def cartpole_command(request: Request):
        if mode == "rl":
            return _error_response(409, "manual control is disabled in rl mode")
        try:
            payload = await _read_json(request)
            force = force_publisher.set_force(payload["force"])
        except (KeyError, TypeError, ValueError) as error:
            return _error_response(400, str(error))
        return {"ok": True, "force": force}

    # Mount last so API routes take precedence over the catch-all static route.
    app.mount(
        "/",
        StaticFiles(directory=visualize_root, html=True),
        name="visualize",
    )
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=CONFIG.server.host)
    parser.add_argument("--port", type=int, default=CONFIG.server.http_port)
    parser.add_argument("--mode", choices=("manual", "rl"), default="manual")
    parser.add_argument(
        "--checkpoint",
        default=str(CONFIG.paths.checkpoint_path),
        help="PPO checkpoint used only when --mode rl.",
    )
    args = parser.parse_args()

    controller = None
    rl_runtime = None
    try:
        if args.mode == "rl":
            ensure_port_available(CONFIG.server.websocket_port)
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

        print(
            f"Serving {CONFIG.paths.visualize_root} "
            f"on http://{args.host}:{args.port}"
        )
        if args.mode == "rl":
            print(f"Loading PPO checkpoint {rl_runtime.checkpoint}")
            # Gazebo may replace process signal handlers while TestFixture is
            # initialized on the worker. Restore normal terminal shutdown.
            snapshot = rl_runtime.wait_until_initialized(30.0)
            if snapshot["phase"] == "ERROR":
                raise RuntimeError(snapshot["error"])
            if not snapshot["ready"]:
                raise RuntimeError("RL runtime did not initialize within 30s")
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.default_int_handler)
        else:
            print(
                "Publishing force commands on "
                f"{CONFIG.server.command_topic}"
            )

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
