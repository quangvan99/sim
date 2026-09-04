# AMR Web Viewer

## Install Gazebo Jetty LTS

- [macOS](https://gazebosim.org/docs/latest/install_osx_src/)
- [Ubuntu 24.04](https://gazebosim.org/docs/latest/install_ubuntu_src/)

## Run Gazebo

Run from the project root:

```bash
gz sim -v 4 -r -s worlds/main.sdf
```

The Gazebo WebSocket server must listen on port `9002`.

## Stop Gazebo

```bash
lsof -t -iTCP:9002 -sTCP:LISTEN | xargs -r kill -9
```

## Run the web viewer

The control bridge serves the web viewer and handles keyboard commands. Run it
from the project root:

```bash
uv venv --system-site-packages .venv
uv sync
uv run control_bridge.py --host 0.0.0.0 --port 8009
```

The project pins Python 3.14 through `.python-version` and `pyproject.toml` so
the virtual environment can access the Gazebo Python bindings.

Open the viewer at:

```text
http://<server-ip>:8009/
```

In Version 1, use Arrow Left / Arrow Right to apply force to the CartPole and
Space to stop. `control_bridge.py` must run alongside Gazebo.

## World and model

A `world` typically defines:

- Physics rules: gravity, physics engine, and timestep
- Lighting and environment
- Gazebo systems and plugins
- Models that appear in the scene
- Sensors, actors, and other special objects

Minimal world structure:

```xml
<sdf version="1.11">
  <world name="cart_pole">
    <physics />

    <light name="sun" type="directional" />

    <include>
      <uri>models/cartpole</uri>
    </include>
  </world>
</sdf>
```

In this project, `worlds/main.sdf` is the simulation environment and
`models/cartpole/` is the only CartPole model used in the scene.

In simple terms: **the world is the simulation environment, and a model is an
object placed in it**.
