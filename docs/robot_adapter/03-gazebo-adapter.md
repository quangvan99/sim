# Phase 3 — Gazebo adapter

**Trạng thái:** Đủ thông tin để triển khai sau Phase 2.

## 1. Mục tiêu

Nối lại runtime Gazebo qua `RobotIO` mà không đưa schema Gazebo trở lại
`src/navigation`. Hành vi simulation và bộ smoke test hiện tại phải được giữ.

## 2. Kiến trúc

```text
Gazebo topics/services
        |
   AMRTransport              low-level protobuf, thread và publish
        |
   GazeboRobotIO             parse, validate, transform, RobotState
        |                    
     Navigator               core thuần

AMRTransport.reset()
        |
GazeboSimulation             SimulationControl
```

`AMRTransport` tiếp tục chịu trách nhiệm giao tiếp `gz.transport`. Adapter chịu
trách nhiệm đổi dữ liệu backend sang contract navigation.

## 3. `GazeboRobotIO`

Constructor dự kiến:

```python
GazeboRobotIO(
    transport: AMRTransport,
    profile: RobotProfile = GAZEBO_PROFILE,
    sensor: GazeboSensorConfig = GAZEBO_SENSOR,
    clock: Callable[[], float] = time.monotonic,
)
```

### `read()`

Đọc `transport.telemetry_snapshot()` đúng một lần và dựng `RobotState`:

| `RobotState` | Nguồn Gazebo |
|---|---|
| `pose` | `sensors.model_pose`, world được coi là map |
| `localization` | luôn `Localization.OK` |
| `pose_epoch` | luôn `0` ở baseline |
| `velocity` | `sensors.odometry.twist` |
| `obstacles` | `sensors.lidar`, transform bằng extrinsic |
| `scan_id` | sequence LiDAR |

Validation bắt buộc:

- LiDAR, odometry và model pose đã nhận và chưa stale.
- Pose, velocity và metadata LiDAR hữu hạn.
- Count khớp số range; angle step/range hợp lệ.
- `NaN` hoặc scan malformed raise `SensorDataError`, không coi là free space.
- `+inf`/no-return không tạo obstacle point.
- Range ngoài giới hạn được xử lý theo policy duy nhất có test.

### `drive()` và `stop()`

- `drive(Velocity)` validate finite rồi gọi `transport.set_velocity()`.
- Không chứa planner hoặc command arbitration.
- `stop()` gọi zero velocity và idempotent.
- Không thay đổi dead-man/heartbeat đang có trong low-level transport.

### `now()` và `profile()`

- `now()` dùng clock monotonic được inject để test deterministic.
- `profile()` trả cùng immutable object trong suốt vòng đời adapter.

## 4. `GazeboSimulation`

Wrapper nhỏ quanh `AMRTransport.reset()`:

```python
class GazeboSimulation(SimulationControl):
    def reset(self) -> bool: ...
```

- Không implement `RobotIO`.
- Không chứa navigation state.
- Reset failure trả `False` hoặc propagate lỗi theo contract hiện tại.

## 5. Cấu hình Gazebo

Thêm profile baseline tương đương behavior đã được kiểm chứng:

```text
footprint_radius_m = 0.13
min_linear_mps     = -0.06
max_linear_mps     = 0.90
max_angular_rps    = 1.50
linear_accel_mps2  = 0.50
angular_accel_rps2 = 2.00
goal_tolerance_m   = 0.04
```

Tách metadata adapter:

```text
lidar_count        = 360
sensor_x_m         = -0.032
sensor_y_m         = 0.0
sensor_yaw_rad     = 0.0
sensor_timeout_s   = 0.60
```

Các giá trị này chưa phải fidelity profile; Phase 4 mới mô phỏng robot thật.

## 6. Tích hợp `server.py`

Trong `main()`:

1. Tạo một `AMRTransport`.
2. Tạo `GazeboRobotIO(transport)`.
3. Tạo `GazeboSimulation(transport)`.
4. Tạo `Navigator(io, simulation=simulation)`.
5. Inject các object vào app; shutdown theo thứ tự Navigator rồi transport.

Các endpoint:

- Navigation/manual command đi qua `RobotIO` để giữ một command boundary.
- `/api/amr/telemetry` có thể tiếp tục trả raw telemetry cho UI trong phase này;
  navigation core không được dùng endpoint/schema đó.
- `/api/amr/reset` gọi `Navigator.reset()` như hiện tại.

## 7. Test adapter

`test_gazebo_adapter.py` dùng fake low-level transport, không cần chạy Gazebo:

- Snapshot hợp lệ -> đúng `RobotState`.
- Pose world/map và odometry velocity parse đúng.
- LiDAR extrinsic được áp dụng.
- `NaN`, metadata thiếu, count sai và stale -> `SensorDataError`.
- Cùng sequence -> cùng `scan_id`; sequence mới tăng đúng.
- `drive()` chuyển đúng linear/angular; `stop()` phát zero.
- `now()` và `profile()` trả đúng object/value.
- `GazeboSimulation.reset()` forward đúng một lần.

Hai test sensor-boundary hiện nằm trong `test_amr_dwa.py` được chuyển sang đây:

- `test_invalid_scan_is_not_treated_as_free_space`.
- `test_lidar_extrinsic_is_applied`.

## 8. Smoke test migration

`test_amr_navigation_sim_smoke.py` phải:

- Tạo low-level transport, adapter và simulation wrapper.
- Chờ readiness qua `GazeboRobotIO.read()`.
- Tạo `Navigator(io, simulation=simulation)`.
- Giữ nguyên goal, timeout, reset và collision assertions hiện tại.
- Không đọc raw snapshot để phục vụ navigation assertion.

Kết quả smoke phải tương đương trước refactor; Phase 3 không tune DWA.

## 9. Deliverables

- `GazeboRobotIO` implement đủ `RobotIO`.
- `GazeboSimulation` implement `SimulationControl`.
- Parser/freshness/LiDAR extrinsic nằm trong adapter.
- Server và smoke suite chạy qua adapter mới.
- Core không thay đổi để hỗ trợ Gazebo.

## 10. File thêm

- `src/adapters/__init__.py`
- `src/adapters/gazebo.py`
- `src/tests/test_gazebo_adapter.py`

## 11. File sửa

- `src/config.py`
- `src/transport.py`
- `src/server.py`
- `src/tests/test_amr_navigation_sim_smoke.py`
- `src/tests/test_amr_dwa.py`

`src/navigation/*` chỉ được sửa nếu phát hiện lỗi contract từ Phase 2; mọi sửa
như vậy phải có unit test và không được import Gazebo.

## 12. Trình tự thực hiện

1. Khai báo `GAZEBO_PROFILE` và `GazeboSensorConfig`.
2. Chuyển parser/freshness validation từ `helpers.py` sang `adapters/gazebo.py`.
3. Implement `GazeboRobotIO` và unit test bằng fake transport.
4. Implement `GazeboSimulation` và reset test.
5. Inject adapter vào server.
6. Migrate smoke test sang adapter.
7. Chạy unit test trước, sau đó chạy toàn bộ Gazebo smoke suite.

## 13. Kiểm chứng

```bash
uv run python -m unittest discover -s src/tests -p 'test_gazebo_adapter.py'
uv run python -m unittest discover -s src/tests -p 'test_*.py'
grep -RInE 'gz\.|telemetry_snapshot|model_pose|freshness|sensors\[' \
  src/navigation
git diff --check
```

Sau đó chạy smoke command hiện được dùng cho
`test_amr_navigation_sim_smoke.py`; toàn bộ scenario phải pass.

## 14. Ngoài phạm vi Phase 3

- Không thêm pose noise, dropout hoặc fidelity profile.
- Không load map PGM/YAML.
- Không kết nối rosbridge hoặc robot thật.
- Không thay đổi DWA weights, speed preset hoặc map geometry.

## 15. Definition of Done

- Server simulation khởi động và API giữ behavior hiện tại.
- Unit test adapter và toàn bộ smoke scenario pass.
- Gazebo-specific imports chỉ nằm trong transport/adapter/simulation code.
- `src/navigation` không biết raw Gazebo schema.
- Phase 4 có thể thêm fidelity mode chỉ bằng sửa Gazebo adapter/config/test.
