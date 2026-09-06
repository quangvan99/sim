# AMR Navigation

**Prerequisite:** Gazebo Jetty LTS must be installed.

## 1. Setup

```bash
uv venv --system-site-packages .venv
uv sync

```

---

## 2. Run AMR Navigation

**Stack:** Gazebo Transport (No ROS 2) | A* + DWA-lite (No Nav2)

**Start the processes in two separate terminals:**

*Terminal 1 - Gazebo:*

```bash
GZ_IP=127.0.0.1 GZ_PARTITION=amr_sim GZ_SIM_RESOURCE_PATH="$PWD/src/amr/sim/models" gz sim -v 4 -r -s src/amr/sim/worlds/main.sdf
```

*Terminal 2 - Server:*

```bash
GZ_IP=127.0.0.1  GZ_PARTITION=amr_sim uv run python -m amr.server --host 0.0.0.0 --port 8009
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
pkill -f 'gz-sim-main.*src/amr/sim/worlds/main.sdf'
kill "$(lsof -t -iTCP:8009 -sTCP:LISTEN)" 2>/dev/null || true

```
