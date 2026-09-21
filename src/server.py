"""Serve unified manual and autonomous AMR control over Gazebo Transport."""

from __future__ import annotations

import argparse
import logging
import os

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from config import CONFIG
from navigation.adapter import NavigationAdapter
from navigation.factory import create_navigation_adapter
from navigation.custom.navigator import NavigationConflict
from navigation.nav2.adapter import Nav2GatewayError
from navigation.custom.global_planner import PlanningError
from navigation.types import Point2D
from navigation.custom.helpers import SensorDataError
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
    # Radians in the API; the WebUI exposes this as degrees.
    yaw: float | None = Field(default=None, strict=True)


class NavigationPlanRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    goal: GoalPoint


class SpeedRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    speed_mps: float = Field(strict=True)


def create_app(
    transport: AMRTransport,
    navigation: NavigationAdapter,
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
        try:
            navigation_status = navigation.status()
        except Nav2GatewayError as error:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "ready": False, "error": str(error)},
            )
        return {
            "ok": True,
            "ready": telemetry["ready"],
            "world": CONFIG.simulation.world_name,
            "model": CONFIG.simulation.model_name,
            "websocket_port": CONFIG.server.websocket_port,
            "partition": os.environ.get("GZ_PARTITION", ""),
            "freshness": telemetry["freshness"],
            "navigation": navigation_status,
        }

    @app.get("/api/amr/telemetry")
    def telemetry():
        return transport.telemetry_snapshot()

    @app.post("/api/amr/command")
    def command(command: VelocityCommand):
        if navigation.owns_control:
            navigation.cancel()
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
            status = navigation.set_speed_mps(request.speed_mps)
        except ValueError as error:
            return JSONResponse(
                status_code=422,
                content={"ok": False, "code": "INVALID_SPEED", "error": str(error)},
            )
        except Nav2GatewayError as error:
            return JSONResponse(
                status_code=503,
                content={"ok": False, "code": "NAV2_UNAVAILABLE", "error": str(error)},
            )
        return {"ok": True, "speed_mps": status["speed_mps"]}

    @app.post("/api/amr/reset")
    def reset():
        try:
            status = navigation.reset()
        except NavigationConflict as error:
            return JSONResponse(
                status_code=409, content={"ok": False, "error": str(error)}
            )
        except RuntimeError as error:
            LOGGER.warning("AMR reset failed: %s", error)
            return JSONResponse(
                status_code=503, content={"ok": False, "error": str(error)}
            )
        ok = status["error"] is None
        LOGGER.info("AMR reset %s", "completed" if ok else "failed")
        return JSONResponse(
            status_code=200 if ok else 503,
            content={"ok": ok, **status},
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
        elif isinstance(error, Nav2GatewayError):
            status_code = 503
            code = "NAV2_UNAVAILABLE"
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
            status = navigation.plan(
                Point2D(request.goal.x, request.goal.y, request.goal.yaw)
            )
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/navigate")
    def navigation_navigate(request: NavigationPlanRequest):
        try:
            status = navigation.navigate(
                Point2D(request.goal.x, request.goal.y, request.goal.yaw)
            )
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/validate-goal")
    def navigation_validate_goal(request: NavigationPlanRequest):
        try:
            valid = navigation.is_goal_free(
                Point2D(request.goal.x, request.goal.y, request.goal.yaw)
            )
        except Exception as error:
            return navigation_error(error)
        return {
            "ok": True,
            "valid": valid,
            "code": None if valid else "GOAL_BLOCKED",
            "message": (
                "Goal is valid" if valid else "Goal is blocked or outside the map"
            ),
        }

    @app.post("/api/navigation/start")
    def navigation_start():
        try:
            status = navigation.start()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.post("/api/navigation/cancel")
    def navigation_cancel():
        try:
            status = navigation.cancel()
        except Exception as error:
            return navigation_error(error)
        return {"ok": True, **status}

    @app.get("/api/navigation/status")
    def navigation_status():
        try:
            status = navigation.status()
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
    navigation = create_navigation_adapter(transport)
    try:
        print(
            f"Serving {CONFIG.paths.visualize_root} on "
            f"http://{args.host}:{args.port}; publishing "
            f"{CONFIG.simulation.command_topic} "
            f"using GZ_PARTITION={os.environ['GZ_PARTITION']}"
        )
        uvicorn.run(
            create_app(transport, navigation),
            host=args.host,
            port=args.port,
            access_log=False,
        )
    except KeyboardInterrupt:
        print("Stopping AMR control bridge")
    finally:
        navigation.close()
        transport.close()


if __name__ == "__main__":
    main()
