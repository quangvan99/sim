FROM ubuntu:26.04

COPY --from=ghcr.io/astral-sh/uv:0.12.17 /uv /uvx /bin/

ENV DEBIAN_FRONTEND=noninteractive \
    UV_PROJECT_ENVIRONMENT=/opt/venv \
    UV_LINK_MODE=copy \
    PATH="/opt/venv/bin:$PATH" \
    PYTHONUNBUFFERED=1 \
    GZ_SIM_COMMAND=/usr/libexec/gz/sim10/gz-sim-server

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
        python3.14 \
        python3.14-venv \
        libgl1-mesa-dri \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo \"$VERSION_CODENAME\") main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        gz-sim10-server \
        gz-tools2 \
        gz-transport15-cli \
        python3-gz-jetty-msgs \
        python3-gz-jetty-transport \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY pyproject.toml uv.lock ./
RUN uv venv --python /usr/bin/python3.14 --system-site-packages /opt/venv \
    && uv sync --locked --no-install-project

COPY . .
RUN uv sync --locked

CMD ["uv", "run", "python", "-m", "server", "--host", "0.0.0.0", "--port", "8009"]
