# Phase 1 — RobotIO contract

**Trạng thái:** Đủ thông tin để triển khai ngay.

## 1. Mục tiêu

Định nghĩa biên ổn định giữa `src/navigation` và mọi backend. Phase này chỉ
thêm contract cùng test double; chưa chuyển `Navigator` sang contract mới.

Kết quả mong muốn:

```text
src/navigation  ->  RobotIO  <-  Gazebo / rosbridge / direct hardware
```

## 2. Quy ước dữ liệu

- Đơn vị SI: mét, giây, radian, m/s và rad/s.
- `pose` luôn ở frame `map`.
- `velocity` và `obstacles` luôn ở frame thân robot `base_footprint`.
- Adapter phải validate dữ liệu trước khi tạo `RobotState`.
- Core không được thấy ROS message, Gazebo protobuf hoặc dictionary telemetry.

## 3. Contract cần thêm

### `Localization`

```python
class Localization(StrEnum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    LOST = "LOST"
```

- `OK`: pose đủ tin cậy để chạy bình thường.
- `DEGRADED`: vẫn dùng được nhưng core phải giảm mức tin cậy/tốc độ.
- `LOST`: pose không dùng được; navigation phải dừng.

### `RobotState`

```python
@dataclass(frozen=True)
class RobotState:
    pose: Pose2D
    localization: Localization
    pose_epoch: int
    velocity: Velocity
    obstacles: tuple[Point2D, ...]
    scan_id: int
```

Ý nghĩa:

- `pose_epoch` không âm, tăng khi localization hiệu chỉnh pose rời rạc.
- `scan_id` không âm, tăng khi adapter chấp nhận scan mới.
- `obstacles` đã lọc và transform khỏi frame LiDAR.
- Freshness không lộ ra core; dữ liệu stale làm `read()` thất bại.

### `RobotProfile`

```python
@dataclass(frozen=True)
class RobotProfile:
    footprint_radius_m: float
    max_linear_mps: float
    min_linear_mps: float
    max_angular_rps: float
    linear_accel_mps2: float
    angular_accel_rps2: float
    goal_tolerance_m: float
```

Profile mô tả khả năng vật lý của backend, không chứa tham số scoring/planning.
Giá trị Gazebo và robot thật sẽ được khai báo ở phase sau.

### `RobotIO`

```python
class RobotIO(Protocol):
    def read(self) -> RobotState: ...
    def drive(self, velocity: Velocity) -> None: ...
    def stop(self) -> None: ...
    def now(self) -> float: ...
    def profile(self) -> RobotProfile: ...
```

Hành vi:

- `read()` trả một snapshot đã validate; raise `SensorDataError` nếu thiếu,
  stale, sai frame hoặc sai format.
- `drive()` nhận vận tốc thân robot hữu hạn; adapter không chứa planner.
- `stop()` idempotent và phát zero command ngay.
- `now()` trả monotonic seconds dùng cho timeout.
- `profile()` ổn định trong suốt vòng đời adapter.

### `SimulationControl`

```python
class SimulationControl(Protocol):
    def reset(self) -> bool: ...
```

`reset()` không thuộc `RobotIO`, vì robot thật không reset được thế giới.

### `SensorDataError`

Exception chuẩn khi adapter không dựng được `RobotState` hợp lệ. Trong Phase 1,
class cũ trong `helpers.py` vẫn được giữ để tránh sửa consumer; Phase 2 sẽ chuyển
consumer sang class mới và xóa class cũ.

## 4. Deliverables

- Contract đầy đủ trong `navigation/ports.py`.
- `FakeRobotIO` có clock điều khiển được, state/profile có thể thay đổi, ghi lại
  command và mô phỏng lỗi `read()`.
- Unit test khóa tên field, tính bất biến và hành vi tối thiểu của fake.
- Không thay đổi hành vi runtime hiện tại.

## 5. File thêm

### `src/navigation/ports.py`

- Import `Point2D`, `Pose2D`, `Velocity` từ `navigation.types`.
- Thêm `Localization`, `RobotState`, `RobotProfile`.
- Thêm `RobotIO`, `SimulationControl`, `SensorDataError`.
- Không import `config`, `transport`, `gz`, ROS hoặc `roslibpy`.

### `src/tests/fakes.py`

- Thêm `FakeRobotIO` implement đủ 5 method.
- Cho phép inject `RobotState`, `RobotProfile` và `SensorDataError`.
- `drive()` ghi command; `stop()` ghi `STOP`; `now()` dùng fake clock.
- Không chứa dictionary mô phỏng schema Gazebo.

### `src/tests/test_navigation_ports.py`

- Kiểm tra enum có đúng ba trạng thái.
- Kiểm tra `RobotState` và `RobotProfile` là frozen dataclass.
- Kiểm tra field đúng tên và đúng thứ tự contract.
- Kiểm tra fake đọc state, ghi command, stop idempotent và tăng clock.
- Kiểm tra fake có thể raise `SensorDataError`.

## 6. File sửa

Không có. Các file sau chỉ được sửa từ Phase 2:

- `src/navigation/navigator.py`
- `src/navigation/helpers.py`
- `src/navigation/types.py`
- `src/navigation/local_planner.py`
- `src/config.py`
- `src/server.py`

## 7. Trình tự thực hiện

1. Thêm `ports.py` với docstring mô tả frame, đơn vị và invariant.
2. Thêm `FakeRobotIO` chỉ phụ thuộc contract.
3. Thêm unit test cho contract và fake.
4. Chạy test mới, sau đó chạy toàn bộ test hiện tại để xác nhận không regression.
5. Kiểm tra import để đảm bảo contract không phụ thuộc backend.

## 8. Kiểm chứng

```bash
uv run python -m unittest discover -s src/tests -p 'test_navigation_ports.py'
uv run python -m unittest discover -s src/tests -p 'test_*.py'
grep -En 'gz\.|roslibpy|rclpy|transport|config' src/navigation/ports.py
git diff --check
```

Lệnh `grep` phải không có kết quả.

## 9. Ngoài phạm vi Phase 1

- Không parse LiDAR, AMCL, odometry hoặc TF.
- Không triển khai Gazebo/rosbridge/direct adapter.
- Không thay đổi state machine hoặc hành vi `DEGRADED/LOST`.
- Không di chuyển `NavigationInput` hay hằng số khỏi `config.py`.
- Không thay đổi topic điều khiển.

## 10. Definition of Done

- Ba file mới tồn tại và test pass.
- `RobotIO` có đúng 5 method; `RobotState` có đúng 6 field.
- Contract chỉ dùng kiểu dữ liệu navigation thuần.
- Toàn bộ test cũ vẫn pass.
- Runtime Gazebo hiện tại không thay đổi.
