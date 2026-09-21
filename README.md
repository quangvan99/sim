# AMR Navigation Simulator

Gazebo-based AMR simulator with two navigation backends: custom and Nav2.

Requirements: Docker and Docker Compose.

## Run custom navigation

```bash
docker compose up --build
```

## Run Nav2 navigation

```bash
docker compose -f docker-compose.nav2.yml up --build
```

Nav2 Compose starts both the AMR simulator and the Nav2 gateway.

Open the WebUI at <http://localhost:8009>.

Stop the selected mode:

```bash
docker compose down
docker compose -f docker-compose.nav2.yml down
```

Run only one mode at a time because both modes use the same ports and container
names.

Run the unit tests:

```bash
docker compose run --rm amr uv run python -m unittest discover -s src/tests -p 'test_*.py'
```
