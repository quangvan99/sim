# AMR from scratch

## 1. Setup & Installation

* **Install Gazebo Jetty LTS:** [macOS](https://gazebosim.org/docs/latest/install_osx_src/) | [Ubuntu 24.04](https://gazebosim.org/docs/latest/install_ubuntu_src/)
* **Setup Python Environment:**
```bash
uv venv --system-site-packages .venv
uv sync

```
---

## 2. RUN

Access the web viewer at `http://localhost:8009/` after starting the servers.

*Requires running Gazebo and the web server separately. Control the CartPole using Left/Right arrows, and Space to stop.*

1. **Start Gazebo:**
```bash
GZ_SIM_RESOURCE_PATH="$PWD/src/cartpole/sim" gz sim -v 4 -r -s src/cartpole/sim/worlds/main.sdf

```

2. **Start/Stop Web Server:**

**For Manual control:**
```bash
uv run python -m cartpole.server --host 0.0.0.0 --port 8009

```

**For RL control:**
```bash
uv run python -m cartpole.server --mode rl --checkpoint src/checkpoints/ppo_cartpole.zip --host 0.0.0.0 --port 8009

```

**Stop:**
```bash
lsof -t -iTCP:9002 -sTCP:LISTEN | xargs -r kill -9

```


**Train Model:**
```bash
uv run python -m cartpole.train --steps 25000 --output src/checkpoints/ppo_cartpole

```