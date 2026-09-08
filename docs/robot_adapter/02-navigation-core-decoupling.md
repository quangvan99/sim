# Phase 2 — Tách navigation core

**Trạng thái:** Đủ thông tin để triển khai sau Phase 1.

## 1. Mục tiêu

Chuyển `src/navigation` từ schema Gazebo sang `RobotIO` đã định nghĩa ở Phase 1.
Core chỉ xử lý dữ liệu typed và không biết dữ liệu đến từ simulation hay robot.

Phase 2 và Phase 3 nên nằm trong cùng nhánh/PR: sau Phase 2, unit test chạy được
bằng fake nhưng Gazebo runtime chỉ được nối lại ở Phase 3.

## 2. Kiến trúc sau khi refactor

```text
Navigator
  |-- RobotIO.read() -> RobotState
  |-- RobotIO.drive(Velocity)
  |-- RobotIO.stop()
  |-- RobotIO.now()
  `-- RobotIO.profile() -> RobotProfile

GlobalPlanner <- map + NavigationConfig
LocalPlanner  <- map + NavigationConfig + RobotProfile
```

`NavigationConfig` chỉ giữ tham số thuật toán. Giới hạn vật lý phải lấy từ
`RobotProfile`.

## 3. Thay đổi trong `Navigator`

### Constructor

Đổi từ:

```python
Navigator(transport, config=..., clock=...)
```

thành:

```python
Navigator(io: RobotIO, *, simulation: SimulationControl | None = None, ...)
```

- Lưu `self.io`, `self.robot_profile` và `self.simulation`.
- Bỏ `NavigationTransport`, `Mapping`, `Protocol` và clock truyền riêng.
- Mọi timeout dùng `io.now()`.

### Đọc và phát dữ liệu

- `plan()` gọi `io.read()` để lấy start pose.
- `_step()` gọi `io.read()` đúng một lần cho mỗi cycle.
- `_stop_locked()` gọi `io.stop()`.
- `_commit()` gọi `io.drive(decision.command)`.
- Khi scan chưa đổi, heartbeat gọi lại `io.drive(mission.command)`.
- Không gọi `telemetry_snapshot()`, `set_velocity()` hoặc parser dictionary.

### Localization behavior

- `LOST`: phát stop, chuyển `State.ERROR`, error `localization lost`; không tự
  resume.
- `DEGRADED`: vẫn điều hướng nhưng dùng `degraded_speed_scale`; không được kết
  luận `SUCCEEDED`.
- `OK`: hành vi bình thường.
- Expose localization hiện tại trong `NavigationStatus` để UI/diagnostics đọc.

Thêm `degraded_speed_scale` vào `NavigationConfig`, mặc định ban đầu `0.5`.
Local planner phải áp dụng scale khi sinh dynamic window, không clamp command
sau khi trajectory đã được chấm điểm.

### `pose_epoch`

`_Mission` lưu epoch gần nhất:

- Lần đọc đầu tiên chỉ ghi epoch.
- Epoch thay đổi: reset progress anchor và stall timer về pose hiện tại.
- Không cho `SUCCEEDED` trong chính cycle có correction.
- Cycle tiếp theo lập đường/chấm trajectory từ pose mới.

### Reset

- Nếu có `SimulationControl`, giữ state `RESETTING` và gọi `simulation.reset()`.
- Nếu không có, raise `NavigationUnsupported` trước khi đổi state.
- Robot thật không bao giờ implement world reset.

## 4. Tách cấu hình vật lý

Chuyển nguồn sử dụng các giá trị sau sang `RobotProfile`:

- `robot_radius_m` -> `footprint_radius_m`.
- `min_linear_velocity_mps` -> `min_linear_mps`.
- `max_angular_velocity_rps` -> `max_angular_rps`.
- `linear_acceleration_mps2` -> `linear_accel_mps2`.
- `angular_acceleration_rps2` -> `angular_accel_rps2`.
- `goal_tolerance_m` -> `goal_tolerance_m`.

Các speed preset có thể ở lại `NavigationConfig`, nhưng phải bị giới hạn bởi
`RobotProfile.max_linear_mps`.

Các tham số sau vẫn thuộc thuật toán:

- safety margin, preferred clearance, lookahead;
- control/heartbeat rate, prediction horizon và sample count;
- score weights, obstacle TTL và no-progress timeout.

Metadata/extrinsic/timeout LiDAR được chuyển sang Gazebo adapter ở Phase 3.

## 5. Dọn sensor boundary khỏi core

Trong `helpers.py` chỉ giữ hình học thuần:

- `distance`, `normalize_angle`, `lerp`;
- transform point/path;
- path length, target point và clearance helpers.

Di chuyển khỏi core ở Phase 3:

- `SensorDataError` cũ;
- `_mapping`, `_finite` dùng cho raw telemetry;
- `parse_navigation_input()`;
- `parse_lidar_points()`.

Xóa `NavigationInput` khỏi `navigation/types.py`; `RobotState` thay thế vai trò
của nó tại biên Navigator.

## 6. Thay đổi `LocalPlanner`

- Constructor nhận thêm `RobotProfile`.
- `_window()` dùng giới hạn vận tốc/gia tốc từ profile.
- Collision radius và reported clearance dùng `footprint_radius_m`.
- `set_speed_mps()` không cho vượt `profile.max_linear_mps`.
- `command()` nhận localization speed scale hoặc speed limit đã tính trước.
- Scoring và collision algorithm không đổi ngoài nguồn tham số.

## 7. Deliverables

- Navigator chỉ phụ thuộc `RobotIO`.
- Local planner dùng `RobotProfile` cho mọi giới hạn vật lý.
- Core xử lý `OK`, `DEGRADED`, `LOST` và `pose_epoch`.
- Sensor parsing không còn trong `src/navigation`.
- Unit test state machine không dựng JSON Gazebo.
- Reset simulation được tách khỏi robot thật.

## 8. File thêm

- Không thêm file production mới ngoài Phase 1.
- Có thể thêm test riêng `src/tests/test_localization_contract.py` nếu
  `test_amr_navigator.py` trở nên quá dài.

## 9. File sửa

### Production

- `src/navigation/navigator.py`
- `src/navigation/local_planner.py`
- `src/navigation/global_planner.py`
- `src/navigation/helpers.py`
- `src/navigation/types.py`
- `src/config.py`

### Test

- `src/tests/fakes.py`
- `src/tests/test_amr_navigator.py`
- `src/tests/test_amr_dwa.py`
- `src/tests/test_amr_astar.py`
- `src/tests/test_amr_navigation_map.py`

`src/server.py`, `src/transport.py` và Gazebo smoke test được nối lại trong
Phase 3.

## 10. Test bắt buộc

- Plan/start/cancel dùng `FakeRobotIO`.
- Sensor error trong `read()` -> `ERROR` và stop.
- `LOST` giữa mission -> stop trong một cycle.
- `DEGRADED` -> giảm speed window và không `SUCCEEDED`.
- Pose nhảy vào goal trong cycle đổi epoch -> không `SUCCEEDED`.
- Đổi epoch reset anchor/no-progress timer.
- Cancel trong lúc `read()` hoặc planner đang block không phát command cũ.
- Reset thiếu `SimulationControl` bị từ chối; reset fake simulation vẫn pass.
- Local planner không vượt mọi giới hạn trong `RobotProfile`.

## 11. Trình tự thực hiện

1. Chuyển config vật lý sang `RobotProfile` tại các call site.
2. Chuyển LocalPlanner sang profile và cập nhật unit test.
3. Chuyển Navigator sang `RobotIO`.
4. Thêm behavior localization và epoch.
5. Tách reset simulation.
6. Xóa `NavigationInput` và parser khỏi core.
7. Migrate test sang `FakeRobotIO`.
8. Chạy toàn bộ unit test; sau đó thực hiện Phase 3 trước khi chạy smoke test.

## 12. Kiểm chứng

```bash
grep -RInE 'gz\.|telemetry_snapshot|set_velocity|model_pose|freshness|sensors\[' \
  src/navigation
grep -RInE '0\.13|0\.90|1\.50|lidar_count|lidar_sensor_' src/navigation
uv run python -m unittest discover -s src/tests -p 'test_*.py'
git diff --check
```

Hai lệnh `grep` phải không có kết quả ngoài dữ liệu test được giải thích rõ.

## 13. Ngoài phạm vi Phase 2

- Không implement Gazebo hoặc rosbridge adapter.
- Không đọc AMCL, TF, LiDAR hay odometry thật.
- Không thêm PGM/YAML loader hoặc fidelity mode.
- Không tune thông số robot thật.

## 14. Definition of Done

- `src/navigation` không biết backend hay raw telemetry.
- `RobotIO` là đường đọc/phát duy nhất của Navigator.
- Mọi giới hạn vật lý đến từ `RobotProfile`.
- Test localization/epoch/fail-safe pass bằng fake.
- Phase 3 có thể nối Gazebo lại mà không sửa contract hoặc thuật toán core.
