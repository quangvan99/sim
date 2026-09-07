"""Serve unified manual and autonomous AMR control over Gazebo Transport."""

from __future__ import annotations

import argparse
import logging
import os
from typing import Literal

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from config import CONFIG
from navigation.navigator import (
    NavigationConflict,
    Navigator,
)
from navigation.global_planner import PlanningError
from navigation.types import Point2D
from navigation.helpers import SensorDataError
from transport import AMRTransport


LOGGER = logging.getLogger("uvicorn.error")


class VelocityCommand(BaseModel):
    """Validated manual differential-drive velocity command."""

    model_config = ConfigDict(extra="forbid")

    linear: float = Field(strict=True)
    angular: float = Field(strict=True)


class GoalPoint(BaseModel):
    model_config = ConfigDict(extra="forbid")

    x: float = Field(strict=True)
    y: float = Field(strict=True)


class NavigationPlanRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    goal: GoalPoint


class SpeedRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mode: Literal["slow", "fast"] | None = None
    speed_mps: float | None = Field(default=None, strict=True)


def create_app(
    transport: AMRTransport,
    navigator: Navigator,
) -> FastAPI:
    """Build the AMR HTTP API and mount self-contained viewer assets."""

    if not CONFIG.paths.visualize_root.is_dir():
        raise RuntimeError(
            f"Visualization assets not found: {CONFIG.paths.visualize_root}"
        )
    if not CONFIG.paths.models_root.is_dir():
        raise RuntimeError(f"Model assets not found: {CONFIG.paths.models_root}")

    app = FastAPI(
        title="AMR Control",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
    )

    @app.get("/api/health")
    def health():
        telemetry = transport.telemetry_snapshot()
        return {
            "ok": True,
            "ready": telemetry["ready"],
            "world": CONFIG.simulation.world_name,
            "model": CONFIG.simulation.model_name,
            "websocket_port": CONFIG.server.websocket_port,
            "partition": os.environ.get("GZ_PARTITION", ""),
            "freshness": telemetry["freshness"],
            "navigation": navigator.status(),
        }

    @app.get("/api/amr/telemetry")
    def telemetry():
        return transport.telemetry_snapshot()

    @app.post("/api/amr/command")
    def command(command: VelocityCommand):
        if navigator.owns_control:
            navigator.cancel()
            LOGGER.info("Manual command cancelled the active navigation mission")
        try:
            applied = transport.set_velocity(command.linear, command.angular)
        except ValueError as error:
            LOGGER.warning("Rejected AMR command: %s", error)
            return JSONResponse(
                status_code=400, content={"ok": False, "error": str(error)}
            )
        except RuntimeError as error:
            LOGGER.warning("AMR command unavailable: %s", error)
            return JSONResponse(
                status_code=409, content={"ok": False, "error": str(error)}
            )
        LOGGER.info(
            "AMR command published: linear=%.3f m/s angular=%.3f rad/s",
            applied["linear"],
            applied["angular"],
        )
        return {"ok": True, **applied}

    @app.post("/api/amr/speed")
    def speed(request: SpeedRequest):
        """Set the shared browser/DWA speed cap."""

        try:
            if (request.mode is None) == (request.speed_mps is None):
                raise ValueError("provide exactly one of mode or speed_mps")
            status = (
                navigator.set_speed_mps(request.speed_mps)
                if request.speed_mps is not None
                else navigator.set_speed_mode(request.mode)
            )
        except ValueError as error:
            return JSONResponse(
                status_code=422,
                content={"ok": False, "code": "INVALID_SPEED", "error": str(error)},
            )
        return {
            "ok": True,
            "speed_mode": status["speed_mode"],
            "speed_mps": status["speed_mps"],
        }

    @app.post("/api/amr/reset")
    def reset():
        try:
            result = navigator.reset()
        except NavigationConflict as error:
            return JSONResponse(
                status_code=409, content={"ok": False, "error": str(error)}
            )
        except RuntimeError as error:
            LOGGER.warning("AMR reset failed: %s", error)
            return JSONResponse(
                status_code=503, content={"ok": False, "error": str(error)}
            )
        LOGGER.info("AMR reset %s", "completed" if result else "failed")
        return JSONResponse(
            status_code=200 if result else 503,
            content={"ok": result},
        )

    def navigation_error(error: Exception) -> JSONResponse:
        if isinstance(error, PlanningError):
            status_code = 422
            code = error.code
        elif isinstance(error, SensorDataError):
            status_code = 503
            code = "SENSORS_NOT_READY"
        elif isinstance(error, NavigationConflict):
            status_code = 409
            code = "STATE_CONFLICT"
        elif isinstance(error, ValueError):
            status_code = 422
            code = "INVALID_GOAL"
        else:
            status_code = 500
            code = "NAVIGATION_ERROR"
        LOGGER.warning("Navigation request failed: %s", error)
        return JSONResponse(
            status_code=status_code,
            content={"ok": False, "code": code, "error": str(error)},
        )

    @app.post("/api/navigation/plan")
    def navigation_plan(request: NavigationPlanRequest):
        try:
            status = navigator.plan(
                Point2D(request.goal.x, request.goal.y)
            )
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/validate-goal")
    def navigation_validate_goal(request: NavigationPlanRequest):
        try:
            result = navigator.validate_goal(
                Point2D(request.goal.x, request.goal.y)
            )
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **result}

    @app.post("/api/navigation/start")
    def navigation_start():
        try:
            status = navigator.start()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/pause")
    def navigation_pause():
        try:
            status = navigator.pause()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/resume")
    def navigation_resume():
        try:
            status = navigator.resume()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/cancel")
    def navigation_cancel():
        try:
            status = navigator.cancel()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.get("/api/navigation/status")
    def navigation_status():
        try:
            status = navigator.status()
        except Exception as error:
            return navigation_error(error)
        return status

    app.mount(
        "/assets/models",
        StaticFiles(directory=CONFIG.paths.models_root),
        name="models",
    )
    app.mount(
        "/",
        StaticFiles(directory=CONFIG.paths.visualize_root, html=True),
        name="visualize",
    )
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=CONFIG.server.host)
    parser.add_argument("--port", type=int, default=CONFIG.server.http_port)
    parser.add_argument("--partition", default=CONFIG.server.partition)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    os.environ.setdefault("GZ_PARTITION", args.partition)
    transport = AMRTransport()
    navigator = Navigator(transport)
    try:
        print(
            f"Serving {CONFIG.paths.visualize_root} on "
            f"http://{args.host}:{args.port}; publishing "
            f"{CONFIG.simulation.command_topic} "
            f"using GZ_PARTITION={os.environ['GZ_PARTITION']}"
        )
        uvicorn.run(
            create_app(transport, navigator),
            host=args.host,
            port=args.port,
            access_log=False,
        )
    except KeyboardInterrupt:
        print("Stopping AMR control bridge")
    finally:
        navigator.close()
        transport.close()


if __name__ == "__main__":
    main()
