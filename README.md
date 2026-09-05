# CartPole Demo

Install Gazebo Jetty LTS, then:

```bash
uv venv --system-site-packages .venv
uv sync
```

Open <http://localhost:8009>. Run one mode at a time.

## RUN

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
