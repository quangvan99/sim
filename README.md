# AMR Navigation Simulator

**Prerequisite:** Gazebo Jetty LTS must be installed.

## Demo

![AMR Navigation demo](docs/image.png)

AMR autonomously navigates to the goal while avoiding obstacles.

## 1. Setup

```bash
uv venv --system-site-packages .venv
uv sync

```

### Docker

```bash
docker compose up --build
```

Open [http://localhost:8009](http://localhost:8009). To run tests through the
container's `uv` environment:

```bash
docker compose exec amr uv run python -m unittest discover -s src/tests -p 'test_*.py'
RUN_GZ_SIM_SMOKE=1 docker compose run --rm amr uv run python -m unittest src/tests/test_amr_navigation_sim_smoke.py
```

---

## 2. Run AMR Navigation

**Stack:** Gazebo Transport (No ROS 2) | Dijkstra cost-to-go field + DWA-lite (No Nav2)

**Start the processes in two separate terminals:**

*Terminal 1 - Gazebo:*

```bash
GZ_IP=127.0.0.1 GZ_PARTITION=amr_sim GZ_SIM_RESOURCE_PATH="$PWD/src/sim/models" gz sim -v 4 -r -s src/sim/worlds/main.sdf
```

*Terminal 2 - Server:*

```bash
GZ_IP=127.0.0.1  GZ_PARTITION=amr_sim uv run python -m server --host 0.0.0.0 --port 8009
```

---

## 3. Usage

Open [http://localhost:8009](http://localhost:8009) in your browser. *(Note: Allow ports `8009` and `9002` if accessing remotely).*

* **Manual Control:** Use `W/A/S/D` to drive and `Space` to stop. Any manual input instantly cancels active autonomous navigation.
* **Speed:** Starts at `0.50 m/s`; use the left/right triangle buttons to adjust the shared speed from `0.30` to `0.90 m/s`.
* **Autonomous Navigation:** Click `Select Goal` → click a destination on the map → click `Go`.
* **Other Controls:** Use `Stop` to cancel a mission, or `Reset` to restore the simulation to its initial state.

---

## 4. Testing

**Run Navigation Tests:**

```bash
uv run python -m unittest discover -s src/tests -p 'test_*.py'
RUN_GZ_SIM_SMOKE=1 uv run python -m unittest src/tests/test_amr_navigation_sim_smoke.py

```

**Stop All AMR Processes:**

```bash
pkill -f 'gz-sim-main.*src/sim/worlds/main.sdf'
kill "$(lsof -t -iTCP:8009 -sTCP:LISTEN)" 2>/dev/null || true

```
