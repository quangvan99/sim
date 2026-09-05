"""Serve the AMR viewer and bridge manual commands to Gazebo Transport."""

from __future__ import annotations

import argparse
import logging
import os

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from amr.config import CONFIG
from amr.transport import AMRTransport


LOGGER = logging.getLogger("uvicorn.error")


class VelocityCommand(BaseModel):
    """Validated manual differential-drive velocity command."""

    model_config = ConfigDict(extra="forbid")

    linear: float = Field(strict=True)
    angular: float = Field(strict=True)


def create_app(transport: AMRTransport) -> FastAPI:
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
        }

    @app.get("/api/amr/telemetry")
    def telemetry():
        return transport.telemetry_snapshot()

    @app.post("/api/amr/command")
    def command(command: VelocityCommand):
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

    @app.post("/api/amr/reset")
    def reset():
        try:
            result = transport.reset()
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
    try:
        print(
            f"Serving {CONFIG.paths.visualize_root} on "
            f"http://{args.host}:{args.port}; publishing "
            f"{CONFIG.simulation.command_topic} "
            f"using GZ_PARTITION={os.environ['GZ_PARTITION']}"
        )
        uvicorn.run(
            create_app(transport),
            host=args.host,
            port=args.port,
            access_log=False,
        )
    except KeyboardInterrupt:
        print("Stopping AMR control bridge")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
