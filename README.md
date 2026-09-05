# Gazebo CartPole and AMR Demos

Install Gazebo Jetty LTS, then:

```bash
uv venv --system-site-packages .venv
uv sync
```

## AMR — RUN

The AMR demo is a standalone differential-drive simulation.
It uses Gazebo Transport directly and does not require ROS 2.

Terminal 1 — start Gazebo headless:

```bash
GZ_PARTITION=amr_sim \
GZ_SIM_RESOURCE_PATH="$PWD/src/amr/sim/models" \
gz sim -v 4 -r -s src/amr/sim/worlds/main.sdf
```

Terminal 2 — start the control and telemetry server:

```bash
GZ_PARTITION=amr_sim \
uv run python -m amr.server --host 0.0.0.0 --port 8010
```

Open <http://localhost:8010>. Manual controls:

- `W` / `S`: forward / reverse.
- `A` / `D`: turn left / right.
- `Space`: stop.
- `Reset`: restore the initial world state.

For access from another machine or a VPS, allow both TCP ports `8010` (HTTP)
and `9003` (Gazebo WebSocket). The browser connects to port `9003` on the same
hostname used to open the HTTP page.

The server exposes LiDAR, IMU, wheel odometry, and joint-state freshness at
`/api/amr/telemetry`. To inspect Gazebo topics directly:

```bash
GZ_PARTITION=amr_sim gz topic -l
GZ_PARTITION=amr_sim gz topic -e -t /amr/scan -n 1
GZ_PARTITION=amr_sim gz topic -e -t /amr/imu -n 1
GZ_PARTITION=amr_sim gz topic -e -t /amr/odom -n 1
```

Stop all AMR processes:

```bash
pkill -f 'gz-sim-main.*src/amr/sim/worlds/main.sdf'
kill "$(lsof -t -iTCP:8010 -sTCP:LISTEN)" 2>/dev/null || true
```

## CartPole — RUN

Open <http://localhost:8009>. Run one CartPole mode at a time.

Terminal 1:

```bash
GZ_SIM_RESOURCE_PATH="$PWD/src/cartpole/sim" gz sim -v 4 -r -s src/cartpole/sim/worlds/main.sdf
```

Terminal 2:

```bash
uv run python -m cartpole.server --host 0.0.0.0 --port 8009
```

OR

```bash
uv run python -m cartpole.server --mode rl --checkpoint src/checkpoints/ppo_cartpole.zip --host 0.0.0.0 --port 8009
```

Stop all CartPole processes:

```bash
pkill -f 'gz-sim-main.*src/cartpole/sim/worlds/main.sdf'
kill "$(lsof -t -iTCP:8009 -sTCP:LISTEN)" 2>/dev/null || true
```

## Train

```bash
uv run python -m cartpole.train --steps 25000 --output src/checkpoints/ppo_cartpole
```
