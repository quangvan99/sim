# Phase 4 — Gazebo fidelity và tuning

**Sẵn sàng:** Đủ để triển khai; cần đo tốc độ quay và gia tốc thật để chốt tuning.

## Mục tiêu

Tinh chỉnh Dijkstra/DWA trong Gazebo với đặc tính gần robot thật.

## Deliverables

- Profile thật ban đầu: radius `0.185 m`, linear `<= 0.33 m/s`, tolerance `0.12 m`.
- Fidelity mode: pose noise/jump, scan 12 Hz, dropout, stale và localization loss.
- Loader map PGM/YAML; unknown được xem là obstacle.
- Planning resolution tách khỏi map resolution.
- Bộ regression cho ground-truth và fidelity mode.

## File thêm

- `src/navigation/map_loaders.py`
- `src/tools/json_to_pgm.py`
- `src/tests/test_map_loaders.py`

## File sửa

- `src/adapters/gazebo.py`
- `src/navigation/global_planner.py`
- `src/config.py`
- `src/tests/test_amr_navigation_sim_smoke.py`

## Nghiệm thu

- Smoke test pass với JSON và PGM map.
- Test pass ở ground-truth và fidelity mode.
- Không có collision trong các kịch bản chuẩn.

