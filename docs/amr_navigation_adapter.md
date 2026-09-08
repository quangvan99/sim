# AMR Navigation — Adapter Port Plan

**Mục tiêu cuối: robot thật chạy bằng `src/navigation`, không ROS.**

Kế hoạch chia **hai giai đoạn**:

| | Mục tiêu | ROS còn không? | Pose từ đâu |
|---|---|---|---|
| **Giai đoạn A** | Robot chạy tuyến A→B bằng core mới, **sớm nhất có thể** | Còn — chỉ làm nguồn dữ liệu | AMCL đang chạy |
| **Giai đoạn B** | Bỏ ROS hoàn toàn | Không | `navigation/localization.py` tự viết |

**Vì sao A trước.** Giai đoạn A trả lời câu hỏi quan trọng nhất — *Dijkstra +
DWA-lite có thực sự lái được con robot này không* — **trước khi** đầu tư ~400
dòng vào scan matcher. Làm B trước rồi mới phát hiện core lái không tốt (footprint
tròn quá bảo thủ, DWA tune sai cho thân xe 0.31 m, hành lang quá hẹp) là đầu tư
sai chỗ.

**Không phí công.** Mọi thứ trong giai đoạn A đều dùng lại nguyên vẹn cho B, trừ
đúng một file cầu nối ROS (~100 dòng) sẽ vứt đi.

Nguyên tắc xuyên suốt: **mọi dòng code chạy trên robot thật phải được chứng minh
trên Gazebo trước, trừ đúng một file transport.**

Trạng thái V1 nằm ở `docs/amr_navigation.md`; contract và hành vi an toàn ở đó
phải giữ nguyên. Hiện trạng phần cứng lấy từ `docs/0709.md` (07/09/2026).

## 1. Vấn đề: port đã có nhưng đang rò

```python
# navigation/navigator.py
class NavigationTransport(Protocol):
    def telemetry_snapshot(self) -> Mapping[str, object]: ...
    def set_velocity(self, linear: float, angular: float) -> object: ...
    def reset(self) -> bool: ...
```

Protocol nhìn trung lập, nhưng truyền `dict` không kiểu và schema của dict đó
chính là schema Gazebo.

| Bằng chứng | Vị trí | Vì sao là coupling |
|---|---|---|
| Core tự bóc schema Gazebo | `navigation/helpers.py:155-199` | Đọc `freshness[...]["age_seconds"]`, `sensors["model_pose"]["position"]` |
| LiDAR của một sensor cụ thể là hằng số navigation | `config.py`, `helpers.py:128` | Từ chối scan nếu `count != 360` |
| Động học của một robot cụ thể là hằng số navigation | `config.py` | `robot_radius_m=0.13`, `max_angular_velocity_rps=1.50` |
| Unit test state machine phải chế JSON hình Gazebo | `tests/test_amr_navigator.py` | 25 dòng dựng `sensors.model_pose.position.z` chỉ để test `PLANNING → READY` |

Thêm một lỗi ngữ nghĩa: **`reset()` là năng lực simulation nằm trong port điều
hướng.** Robot thật không reset được thế giới.

## 2. Core cần gì, robot có gì

`parse_navigation_input()` đọc đúng ba nhóm dữ liệu. Đối chiếu với `0709.md`:

| Core cần | Robot có | Giai đoạn A | Giai đoạn B |
|---|---|---|---|
| LiDAR 360 tia + metadata | `/scan` 12.63 Hz, 360 tia, 0.12–8 m | ✅ khớp thẳng | driver USB thuần |
| Vận tốc thân `linear.x`, `angular.z` | `/odom_raw` 11.10 Hz | ✅ khớp thẳng | UART YB_Car |
| **Pose trong map frame** | TF `map → odom → base_footprint` | ✅ **AMCL cho sẵn** | ❌ **phải tự viết** |
| Map | `map.pgm` + `map.yaml` (v5, 35.113 byte) | ❌ cần loader | ❌ cần loader |

### 2.1. Lỗ hổng cốt lõi: core không tự sinh pose

`src/navigation` nhận `pose` như **dữ kiện đầu vào**, không bao giờ tự tính.
Gazebo cho ground truth; trên robot là AMCL. Bỏ ROS thì **không ai tính nữa**.

Dead reckoning bằng `/odom_raw` một mình trôi vô hạn: lệch hướng chỉ **2°**, đi
100 m đã sai `100 × sin(2°) ≈ 3.5 m`. Robot tour chạy 30 phút sẽ lạc hẳn.

Đây chính là lý do có giai đoạn A: **mượn AMCL để chạy trước**, rồi mới thay.

### 2.2. Phần cứng không có

`/joint_states` (0 publisher), bumper / cliff / range (0 publisher, 0 mẫu),
depth camera, GPS. `/safety/estop` chỉ là heartbeat phần mềm 2 Hz.

**IMU và odometry không nằm trong port** — chúng là đầu vào nội bộ của lớp
localization (B1). Core chỉ nhận `pose` đã hoàn chỉnh.

## 3. Cái đang từ bỏ — theo từng giai đoạn

| Thành phần ROS | Giai đoạn A | Giai đoạn B |
|---|---|---|
| `StopTurnStateLatticePlanner`, executor dừng–quay | **bỏ** | bỏ |
| Nav2 planner / controller / RPP / costmap | **bỏ** | bỏ |
| AMCL, `robot_localization` EKF | **giữ** | bỏ |
| `twist_mux`, `velocity_smoother` | **giữ** | bỏ |
| `rovera_motion_safety` (50 Hz) | **giữ — lưới an toàn** | bỏ ⚠️ xem §7.5 |
| SLAM Toolbox | offline, giữ để tạo map | offline |

Năng lực mất đi so với stack cũ, ghi ra để không ai ngạc nhiên:

| Stack ROS có | Core repo |
|---|---|
| Footprint **có hướng**, kiểm tra vùng quét khi quay | hình tròn (§7.1) |
| 3 candidate + `ROUTE_SELECTION` UI, countdown 10 s | 1 đường duy nhất |
| `DynamicObstacleOverlay`: gom cụm, ước lượng vận tốc, TTC 3 s | lớp TTL 5 s |
| `SAFE_STANDOFF`, snap goal tới 0.45 m | từ chối goal bị chặn |
| AMCL trưởng thành | scan matching tự viết (B1) |

Đổi lại: một ngôn ngữ, một stack, không Docker, không DDS, debug được, test được
bằng smoke suite hiện có.

**Hai lỗi chặn trong `0709.md` biến mất ở giai đoạn B**: EKF covariance
1.37×10¹⁶ m² và costmap trust 72.8% đều thuộc `robot_localization` và Nav2. Ở
giai đoạn A chúng **vẫn còn** — xem §7.6.

## 4. Ba khác biệt buộc phải đổi contract

### 4.1. Pose thật có chất lượng; pose sim thì không

Gazebo cho ground truth liên tục. AMCL (giai đoạn A) và scan matching (giai đoạn
B) đều cho ước lượng **nhảy khi hiệu chỉnh** và **có thể lạc**. Code hiện tại vỡ
ở hai chỗ:

- `navigator._begin_cycle`: `distance(pose, goal) <= goal_tolerance` → một cú
  nhảy **báo tới đích giả** trong khi robot còn cách xa.
- `navigator._commit`: `progress_m(anchor, position)` → nhảy lùi thành "no
  progress", nhảy tới thành progress ảo.

**Áp dụng cho cả hai giai đoạn** — AMCL nhảy y như scan matching nhảy.

### 4.2. Footprint và động học khác một trời một vực

| | Gazebo (repo) | Robot thật |
|---|---|---|
| Thân xe | hộp 0.14 × 0.14, `robot_radius_m = 0.13` | **0.31 × 0.20 m** |
| Tốc độ tiến | 0.30 / 0.50 / 0.90 m/s | ≤ **0.33 m/s** |
| Tốc độ quay | 1.50 rad/s | **phải đo** — xem ghi chú |
| Dung sai đích | 0.04 m | 0.12 m |

Ghi chú: `0.40 rad/s` trong `0709.md` là giới hạn **pha quay tại chỗ của executor
stop-turn** đang bị thay thế. DWA chạy arc liên tục nên không áp dụng số đó.

### 4.3. Metadata LiDAR đọc từ nguồn, không phải config

Cả hai tình cờ đều 360 tia, nhưng tầm đo khác (8 m thật vs 3.5 m sim). Extrinsic
là thuộc tính lắp đặt (giai đoạn A: đọc từ TF; giai đoạn B: hằng số transport).

→ Không có `SensorConfig` dùng chung.

## 5. Contract

```python
# navigation/ports.py

class Localization(StrEnum):
    OK = "OK"              # pose dùng được
    DEGRADED = "DEGRADED"  # dùng được nhưng bất định — nới dung sai, giảm tốc
    LOST = "LOST"          # không dùng được — navigation phải dừng


@dataclass(frozen=True)
class RobotState:
    """Một lần đọc robot đã validate, đủ cho một control cycle."""

    pose: Pose2D                      # map frame
    localization: Localization        # chất lượng pose, adapter quyết định
    pose_epoch: int                   # tăng khi pose nhảy rời rạc
    velocity: Velocity                # body frame
    obstacles: tuple[Point2D, ...]    # base frame, đã lọc và transform extrinsic
    scan_id: int                      # tăng đơn điệu, để dedupe scan


@dataclass(frozen=True)
class RobotProfile:
    """Khả năng vật lý của một robot cụ thể — không phải tham số thuật toán."""

    footprint_radius_m: float         # bán kính ngoại tiếp, §7.1
    max_linear_mps: float
    min_linear_mps: float
    max_angular_rps: float
    linear_accel_mps2: float
    angular_accel_rps2: float
    goal_tolerance_m: float


class SensorDataError(RuntimeError):
    """Adapter không dựng được RobotState hợp lệ (thiếu, stale, sai format)."""


class RobotIO(Protocol):
    def read(self) -> RobotState: ...          # raise SensorDataError
    def drive(self, velocity: Velocity) -> None: ...
    def stop(self) -> None: ...
    def now(self) -> float: ...
    def profile(self) -> RobotProfile: ...


class SimulationControl(Protocol):
    """Chỉ simulation mới có. Robot thật không implement."""

    def reset(self) -> bool: ...
```

Sáu field thay vì bốn không vi phạm nguyên tắc "port phải hẹp". Nguyên tắc đó là
**không phơi thêm sensor**. `localization` và `pose_epoch` phơi **đảm bảo khác
nhau giữa các backend** — đúng thứ core buộc phải rẽ nhánh.

| Việc | Gazebo | Robot (A) | Robot (B) |
|---|---|---|---|
| Nguồn pose | `dynamic_pose` | TF từ AMCL | `localization.py` |
| Chất lượng pose | luôn `OK` | covariance `/amcl_pose` | điểm scan match |
| `pose_epoch` | không đổi | `map→odom` đổi quá ngưỡng | hiệu chỉnh vượt ngưỡng |
| LiDAR | `/amr/scan` | `/scan` | driver USB |
| Lệnh | `/amr/cmd_vel` | `/cmd_vel_nav` → mux | UART |

## 6. Chiến lược kiểm chứng

**Nguyên tắc.** Mọi dòng code chạy trên robot phải đã qua Gazebo smoke suite,
trừ đúng file transport của giai đoạn tương ứng.

**Vấn đề.** Gazebo cho ground truth hoàn hảo nên smoke test **không bao giờ
chạm** nhánh `DEGRADED`, `LOST`, hay `pose_epoch` đổi. Bê sang robot là lần đầu
những nhánh đó thực thi — đúng lúc không được phép sai.

**Cách giải: fidelity mode (A7).** Adapter Gazebo mô phỏng **đảm bảo** của robot
thật, không chỉ dữ liệu:

| Bơm vào Gazebo adapter | Mô phỏng cái gì |
|---|---|
| `RobotProfile` thật (r = 0.185 m, ≤ 0.33 m/s, tol 0.12 m) | Động học và kích thước |
| Nhiễu pose Gauss + nhảy rời rạc định kỳ | Hiệu chỉnh AMCL / scan matching |
| Hạ `localization` xuống `DEGRADED`/`LOST` theo lịch | Mất/suy giảm định vị |
| Scan ~12 Hz, tầm 8 m, bỏ ngẫu nhiên một phần tia | LiDAR thật và dropout |

**Cơ hội riêng cho giai đoạn B.** Trên Gazebo có `dynamic_pose` làm ground truth
— thứ robot thật **không bao giờ có**. Chạy scan matcher song song và đo sai số
trực tiếp. Đây là cách duy nhất tune localization trước khi chạm phần cứng.

## 7. Quyết định đã chốt

### 7.1. Footprint: hình tròn ngoại tiếp, chưa làm chữ nhật

Robot thật 0.31 × 0.20 m → bán kính ngoại tiếp **0.185 m**, tức coi robot rộng
0.20 m thành đường kính 0.37 m, mất ~9 cm mỗi bên.

**Chấp nhận cho lần deploy đầu**, vì Nav2 cũ đang dùng `inflation_radius = 0.18 m`
nên không tệ hơn cái đang chạy. Footprint có hướng là **khoảng cách thuật toán**,
không phải config, và là milestone riêng.

### 7.2. Giai đoạn A: `drive()` đổ vào `twist_mux`, không phải `/cmd_vel`

`rovera_motion_safety` là publisher duy nhất của `/cmd_vel`. Cầu nối ROS publish
vào **đầu vào Auto của `twist_mux`** (`/cmd_vel_nav`, priority 10), và **phải tắt
`rovera_navigation_adapter`** để không tranh cùng topic.

Ưu tiên joystick 100 / manual 50 / E-stop lock 255 giữ nguyên. Core mới **nằm
trên** lớp safety đang có, không thay nó. Đây là điểm mạnh của giai đoạn A: giữ
được lưới an toàn 50 Hz trong lúc thử một core điều hướng chưa từng chạy thật.

### 7.3. Độ phân giải planning tách khỏi độ phân giải map

`GlobalPlanner` dựng field Dijkstra toàn lưới, đo được **126 ms** cho lưới
200×200 (20×20 m @ 0.10 m). Chi phí xấp xỉ tuyến tính theo số ô = diện tích /
resolution².

Map thật ở **0.05 m**:

| Khu vực | Số ô | Dijkstra ước tính |
|---|---|---|
| ~9.3 × 9.3 m (v5 hiện có) | ~35.000 | ~110 ms ✅ |
| 20 × 20 m | 160.000 | ~500 ms ⚠️ |
| 40 × 40 m | 640.000 | ~2 s ❌ vỡ chu kỳ 200 ms |

→ Loader nhận `planning_resolution_m` riêng. Planning không cần 5 cm; 10 cm giảm
**4× số ô**. LiDAR và DWA vẫn tránh vật cản ở độ phân giải thật.

### 7.4. Ô "chưa biết" phải coi là vật cản

`OccupancyGrid` có hai trạng thái; PGM có ba (trống / vật cản / **chưa biết** =
xám 205). Với map SLAM, vùng chưa biết rất lớn.

| Ánh xạ | Hậu quả |
|---|---|
| unknown → **trống** | planner vạch đường xuyên vùng chưa quét → đâm tường chưa từng thấy |
| unknown → **vật cản** | an toàn, nhưng robot bị nhốt trong vùng đã quét |

**Chọn "vật cản".** Xem lại khi có `preview.png` thật.

### 7.5. Giai đoạn B mất lưới an toàn — phải bù

Giai đoạn A giữ `rovera_motion_safety` chạy 50 Hz làm lớp cắt vận tốc cuối. Giai
đoạn B bỏ nó, khi đó **DWA của core là lớp tránh va chạm duy nhất**, chạy 5 Hz
theo scan.

Đây là **suy giảm an toàn thật**. Trước khi bắt đầu B2 phải quyết: port một lớp
safety tối giản (kiểm tra khoảng phanh theo vận tốc + scan, tần số cao, độc lập
với planner), hay chấp nhận rủi ro ở tốc độ ≤ 0.33 m/s. Ghi thành mục B3.

### 7.6. Giai đoạn A vẫn dính hai lỗi của stack cũ

Vì A còn dùng AMCL + EKF:

- **EKF covariance y ≈ 1.37×10¹⁶ m²** → `localization` sẽ phải báo `DEGRADED`
  hoặc `LOST` gần như liên tục nếu lấy tiêu chí từ covariance.
- Cầu nối ROS nên lấy chất lượng từ **`/amcl_pose` covariance** thay vì
  `/odometry/filtered`, để né lỗi EKF. Nếu vẫn tệ thì phải sửa EKF trước A10.

### 7.7. `State.RESETTING` giữ lại

World reset mất vài giây; trong lúc đó navigation phải từ chối `plan()`. Guard
này là logic navigation, không phải logic sim.

## 8. Cấu trúc thư mục

```text
src/
  navigation/              core — không import gz, ROS, hay driver phần cứng
    ports.py               (A1) RobotState, RobotProfile, RobotIO, ...
    map_loaders.py         (A8) OccupancyGrid, load_static_map, load_pgm_map
    localization.py        (B1) scan matching trên likelihood field
    types.py               (sửa) bỏ NavigationInput
    helpers.py             (sửa) chỉ còn hình học thuần
    global_planner.py      (sửa) OccupancyGrid ra map_loaders
    local_planner.py       (sửa) lấy giới hạn từ RobotProfile
    navigator.py           (sửa) phụ thuộc RobotIO
  adapters/                nơi duy nhất chạm backend
    gazebo.py              (A2/A7) GazeboRobotIO, GazeboSimulation, fidelity
    ros_bridge.py          (A9) ~100 dòng — VỨT ĐI sau giai đoạn B
    robot.py               (B2) transport thuần Python
  transport.py             giữ nguyên vai trò, gz binding cấp thấp
  config.py                (sửa) tách RobotProfile
  tools/json_to_pgm.py     (A8) tool dev, sinh PGM đối chứng
  tests/
    fakes.py               (A5) FakeRobotIO
    test_gazebo_adapter.py (A2)
    test_map_loaders.py    (A8)
    test_localization.py   (A6, mở rộng ở B1)
```

## 9. GIAI ĐOẠN A — robot chạy bằng core mới, ROS làm nguồn dữ liệu

Mốc kết thúc: **robot chạy tuyến A→B dưới điều khiển của `src/navigation`.**

### A1 — Port contract

**Thêm:** `src/navigation/ports.py` — `RobotState`, `Localization`,
`RobotProfile`, `RobotIO`, `SimulationControl`, `SensorDataError`.

**Sửa:** không. Bổ sung thuần.

**Nghiệm thu:** không import `transport`, `adapters`, `gz.*`, driver nào.
`ruff --select F` sạch; 56 unit test pass.

### A2 — Đưa parsing và profile ra khỏi core

**Thêm:** `src/adapters/__init__.py`, `src/adapters/gazebo.py` (`GazeboRobotIO`,
nhận `parse_lidar_points` + `parse_navigation_input` từ `helpers.py`),
`src/tests/test_gazebo_adapter.py` (nhận `test_lidar_extrinsic_is_applied` và
`test_invalid_scan_is_not_treated_as_free_space` từ `test_amr_dwa.py`, cộng test
freshness/stale).

**Sửa:** `helpers.py` (xoá dòng 85-199, còn ~84 dòng hình học);
`types.py` (xoá `NavigationInput`); `config.py` (tách `RobotProfile`:
`robot_radius_m`, `*_linear_velocity_mps`, `max_angular_velocity_rps`,
`*_acceleration_*`, `goal_tolerance_m`; xoá `lidar_count`, `lidar_sensor_*`,
`sensor_timeout_seconds`; thêm `GAZEBO_PROFILE` và `REAL_ROBOT_PROFILE`);
`test_amr_dwa.py`.

**Nghiệm thu:** `grep -rn "gz\.\|sensors\[\|freshness" src/navigation/` → rỗng.
`grep -rn "0\.13\|0\.90\|1\.50\|lidar_count" src/navigation/` → rỗng.
`map_version`, `safety_margin_m`, `preferred_clearance_m`, `score_*` **ở lại**
`NavigationConfig` — đó là tham số thuật toán thật.

### A3 — Core phụ thuộc port và profile

**Sửa:** `navigator.py` (xoá `NavigationTransport`; `__init__(io: RobotIO)`; bỏ
`clock` dùng `io.now()`; `_step` gọi `io.read()`; `_begin_cycle`/`_decide` nhận
`RobotState`; `_stop_locked` gọi `io.stop()`; `_commit` gọi `io.drive()`; dung
sai từ `io.profile()`); `local_planner.py` (`_window()` và collision radius từ
`io.profile()`); `server.py`; `test_amr_navigation_sim_smoke.py`.

**Nghiệm thu:** `grep -rn "telemetry_snapshot\|transport" src/navigation/` → rỗng.
**12 Gazebo smoke test pass** — bằng chứng chính.

### A4 — Tách reset chỉ-có-ở-simulation

**Sửa:** `navigator.py` (`Navigator(io, *, simulation: SimulationControl | None)`;
`reset()` raise `NavigationUnsupported` khi thiếu); `adapters/gazebo.py`
(`GazeboSimulation`); `server.py` (`/api/amr/reset` → `501 NOT_SUPPORTED`);
`app.js` (xử lý 501, ẩn nút `Reset`); `test_amr_navigator.py`.

**Bắt buộc cho giai đoạn A** — robot không có reset, thiếu bước này thì cầu nối
ROS sẽ vỡ ngay khi ai đó bấm Reset.

**Đã cân nhắc phương án khác.** Bỏ hẳn `reset()` và để `server.py` tổ hợp
`cancel() → simulation.reset() → clear()` bị loại: giữa hai lệnh, state là
`CANCELLED`, mà `CANCELLED` **cho phép** `plan()` — một request đúng lúc đó sẽ
lập kế hoạch từ pose cũ trong khi thế giới đang reset.

### A5 — Test double trên port

**Thêm:** `src/tests/fakes.py` — `FakeRobotIO`.

**Sửa:** `test_amr_navigator.py` — xoá `FakeTransport` (25 dòng JSON Gazebo) và
hàm `scan()`.

**Nghiệm thu — thước đo chính:**
`grep -rni "gazebo\|gz\.\|sensors\|freshness\|model_pose" src/tests/test_amr_navigator.py`
→ **rỗng**.

### A6 — Core chịu được pose nhảy và mất định vị

**Đây là deliverable làm cho việc chạy trên robot trở nên an toàn.** AMCL nhảy
y hệt scan matching nhảy — cần cho **cả hai** giai đoạn.

**Thêm:** `src/tests/test_localization.py` — `FakeRobotIO` bơm `DEGRADED`,
`LOST`, `pose_epoch` đổi.

**Sửa:** `types.py` (thêm `State.LOCALIZATION_LOST`, hoặc tái dùng `ERROR` với
message riêng — **chốt khi implement**); `navigator.py`:

- `LOST` → publish zero, chuyển `ERROR`/`LOCALIZATION_LOST`, không tự resume.
- `DEGRADED` → chạy tiếp nhưng **không được kết luận `SUCCEEDED`**; nới dung sai
  hoặc giảm tốc — **chốt khi implement**.
- `pose_epoch` đổi → reset `anchor` và `last_progress`; **chặn `SUCCEEDED` trong
  cycle đó**.

Cộng `app.js` — hiện trạng thái định vị trên top bar.

**Nghiệm thu:** test chứng minh nhảy pose vào đúng goal **không** sinh
`SUCCEEDED`; `LOST` giữa mission → command zero trong một cycle.

### A7 — Gazebo fidelity mode

**Sửa:** `adapters/gazebo.py` (tuỳ chọn fidelity theo bảng §6);
`test_amr_navigation_sim_smoke.py` (biến thể chạy 12 kịch bản dưới fidelity).

**Nghiệm thu:** 12 smoke test pass ở **cả hai** chế độ. Điều kiện tiên quyết để
bắt đầu A9.

### A8 — Map: loader PGM/YAML

Phát triển và verify **hoàn toàn không cần file từ Pi**.

**Thêm**

- `src/navigation/map_loaders.py` — `OccupancyGrid` chuyển từ `global_planner.py`
  sang, cộng `load_static_map()` (JSON, đã có) và:

  ```python
  def load_pgm_map(
      pgm: Path, yaml_path: Path, config, *, planning_resolution_m: float | None
  ) -> OccupancyGrid:
      # đọc P5/P2; flipud vì PGM đếm từ trên xuống, origin ROS ở góc dưới-trái
      # đen (<128) = vật cản; xám ~205 = chưa biết → CŨNG là vật cản (§7.4)
      # raise nếu origin[2] != 0 — không âm thầm bỏ qua map bị xoay
      # downsample về planning_resolution_m nếu có (§7.3)
  ```

- `src/tools/json_to_pgm.py` — convert `amr_world.json` → PGM + YAML, để có bản
  đối chứng khi chưa có map thật.
- `src/tests/test_map_loaders.py` — **test round-trip**: `load_static_map(json)`
  và `load_pgm_map(pgm, yaml)` phải cho `raw`, `occupied`, `min_x`, `min_y`,
  `resolution_m` **giống hệt**.

**Sửa:** `global_planner.py` (import `OccupancyGrid` từ `map_loaders`);
`config.py` (thêm `planning_resolution_m`); `test_amr_navigation_sim_smoke.py`
(biến thể chạy 12 kịch bản qua **đường PGM**).

**Nghiệm thu:** round-trip test pass — có ground truth đối chứng, thứ file thật
không cho được. 12 smoke test pass qua đường PGM. Loader `raise` khi
`origin[2] != 0`.

**Khi có map thật:** chỉ thay file và xem lại tỉ lệ ô unknown. **Không sửa code.**

### A9 — Cầu nối ROS (~100 dòng, sẽ vứt đi)

**Thêm:** `src/adapters/ros_bridge.py`:

| Port | Nguồn ROS |
|---|---|
| `pose` | TF `map → base_footprint` |
| `localization` | covariance của `/amcl_pose` (**không** dùng `/odometry/filtered`, xem §7.6) |
| `pose_epoch` | tăng khi `map → odom` đổi quá ngưỡng |
| `velocity` | `/odom_raw.twist` |
| `obstacles` | `/scan`, transform qua TF `base_footprint → laser_frame` |
| `scan_id` | `header.stamp` |
| `drive()` | publish `/cmd_vel_nav` (§7.2) |
| `now()` | ROS time |
| `profile()` | `REAL_ROBOT_PROFILE` |

**Nghiệm thu:** không chứa logic điều hướng — chỉ đọc, dịch, validate. Giữ dưới
~150 dòng. Đây là **file duy nhất chưa qua Gazebo** ở giai đoạn A.

### A10 — Bring-up và tuning trên robot

**Thêm:** `docs/amr_deploy_checklist.md`.

**Việc**

1. Tắt `rovera_navigation_adapter` để không tranh `/cmd_vel_nav`.
2. Nạp map v5 qua `load_pgm_map()`; kiểm tra tỉ lệ ô unknown có nhốt robot không.
3. Chạy có giám sát, tốc độ thấp nhất, tay sẵn trên E-stop.
4. **Tune lại trọng số DWA** — `score_progress` 2.4 / `score_clearance` 2.8 …
   được calibrate bằng Gazebo cho robot **nhỏ hơn và nhanh hơn**. Với thân 0.31 m
   và ≤ 0.33 m/s gần như chắc chắn phải chỉnh.
5. Đo tốc độ quay tối đa thật → cập nhật `REAL_ROBOT_PROFILE`.

**Nghiệm thu:** robot đi hết một tuyến A→B thật, dừng trong dung sai, không va
chạm. **Đây là mốc kết thúc giai đoạn A.**

## 10. GIAI ĐOẠN B — bỏ ROS hoàn toàn

Chỉ bắt đầu sau khi A10 đạt. Lúc đó đã biết chắc core lái được robot.

### B1 — Localization: scan matching trên likelihood field

Thay AMCL. Mảnh lớn nhất về thuật toán.

**Điểm tựa: hạ tầng đã có sẵn.** AMCL dùng *likelihood field* — bảng "mỗi ô cách
vật cản gần nhất bao xa". Trong repo đó **chính là** `OccupancyGrid.clearance`,
sinh bởi `_clearance_metres()`.

**Thêm:** `src/navigation/localization.py` (~300–400 dòng):

| Bước | Nội dung |
|---|---|
| 1. Dự đoán | tích phân wheel odom + gyro từ pose trước |
| 2. Sinh ứng viên | lưới nhỏ hoặc particle quanh dự đoán |
| 3. Chấm điểm | transform 360 điểm scan sang map frame, tra `clearance`; điểm scan phải rơi **đúng** lên vật cản nên clearance nhỏ = tốt |
| 4. Kết luận | chọn ứng viên tốt nhất; xuất `pose`, `Localization`, tăng `pose_epoch` khi hiệu chỉnh vượt ngưỡng |

Chấm điểm phải **robust**: bỏ x% tia tệ nhất, vì người đi lại làm nhiều tia không
khớp map tĩnh.

**Nghiệm thu — tận dụng lợi thế Gazebo**

Chạy `localization.py` **song song** với `dynamic_pose` trên 12 kịch bản smoke,
ghi sai số vị trí và góc. Ngưỡng đề xuất: **p95 < 0.05 m, < 3°**. Đây là phép đo
robot thật không bao giờ cho được.

**Ba chỗ khó, xử lý tường minh**

1. **Global localization** (khởi động không biết ở đâu) khó hơn tracking nhiều.
   **Né:** robot luôn khởi động từ dock cố định → seed pose cứng. Không implement
   ở milestone này.
2. **Hành lang dài trống trải** — scan matching mất ràng buộc dọc trục.
3. **Người đi lại** — dùng robust scoring ở bước 3.

### B2 — Transport thuần Python

**Thêm:** `src/adapters/robot.py`:

| Port | Nguồn |
|---|---|
| `obstacles` | driver LiDAR Python thuần (RPLIDAR/YDLIDAR SDK) |
| `pose`, `localization`, `pose_epoch` | `navigation/localization.py` |
| `velocity` | odometry YB_Car qua UART |
| `drive()` | gói lệnh vận tốc qua UART |
| `now()` | `time.monotonic()` |
| `profile()` | `REAL_ROBOT_PROFILE` |

**Câu hỏi kỹ thuật chặn.** IMU, odometry và lệnh motor đều đi chung qua
`YB_Car_Node` trên **micro-ROS, UART `/dev/ttyUSB0` @921600**. micro-ROS nói
XRCE-DDS; tự nói giao thức đó không có ROS rất cực.

| Khả năng | Hệ quả |
|---|---|
| Board Yahboom có firmware **serial packet thường** | Sạch — chỉ `pyserial` |
| Chỉ có micro-ROS | Phải giữ `micro_ros_agent` + process cầu nối. Còn một mảnh ROS |
| Tự viết firmware MCU | Sạch nhất, tốn công nhất |

LiDAR thường không vướng: cắm USB, có driver Python thuần.

**Câu hỏi này không chặn giai đoạn A** — chỉ cần trả lời trước khi bắt đầu B2.

**Nghiệm thu:** không chứa logic điều hướng. Giữ dưới ~250 dòng.

### B3 — Bù lại lớp safety đã mất

Xem §7.5. Bỏ `rovera_motion_safety` nghĩa là DWA 5 Hz trở thành lớp tránh va
chạm duy nhất. Quyết định trước khi chuyển đổi: port một lớp safety tối giản
(khoảng phanh theo vận tốc + scan, tần số cao, độc lập planner), hay chấp nhận
rủi ro ở ≤ 0.33 m/s.

### B4 — Chuyển đổi và verify

Đổi `ros_bridge.py` → `robot.py`, tắt toàn bộ container ROS, chạy lại tuyến A→B
có giám sát. Xoá `ros_bridge.py`.

## 11. Thứ tự và rủi ro

| Bước | Giai đoạn | Rủi ro | Kiểm chứng bằng |
|---|---|---|---|
| A1 | A | Không | — |
| A2 | A | Thấp — di chuyển cơ học | 56 unit test |
| A3 | A | **Cao** — chạm state machine và vòng điều khiển | 12 smoke test |
| A4 | A | Trung bình — chạm HTTP API và browser | thủ công nút Reset |
| A5 | A | Thấp; thấy khó là dấu hiệu core còn coupling | grep |
| A6 | A | Trung bình — thêm nhánh state machine | `test_localization.py` |
| A7 | A | Thấp — chỉ adapter và test | 12 smoke × 2 chế độ |
| A8 | A | Thấp — có round-trip đối chứng | round-trip + smoke qua PGM |
| A9 | A | Trung bình — file duy nhất chưa qua Gazebo | đọc kỹ, giữ mỏng |
| A10 | A | **Chỉ đo được trên phần cứng** | robot có giám sát |
| B1 | B | **Cao nhất về thuật toán** | đo sai số vs `dynamic_pose` |
| B2 | B | Cao — phụ thuộc firmware | robot có giám sát |
| B3 | B | An toàn — xem §7.5 | quyết định trước khi chuyển |
| B4 | B | Trung bình | chạy lại tuyến A→B |

A1–A8 làm được **ngay bây giờ**: không cần Pi, không cần map thật, không cần
robot. A9–A10 cần robot. B1 làm được hoàn toàn trên Gazebo.

## 12. Ngoài phạm vi

- **Footprint chữ nhật có hướng** — §7.1. Milestone riêng.
- **Global localization / kidnapped robot** — né bằng seed từ dock.
- **Vùng "không biết" trong LiDAR runtime** — dropout hiện bị coi là *trống*;
  robot thật phải là *unknown*. Cần thêm trường vào `RobotState`.
- **Bumper / cliff / range** — chưa có phần cứng. Khi nối thì vào qua lớp safety
  riêng, **không qua port điều hướng**.
- **Tự khám phá tạo map** — tạo map vẫn là việc offline.

## 13. Cần xác nhận

| # | Câu hỏi | Chặn |
|---|---|---|
| 1 | Tuyến chạy có hành lang hẹp tới mức xấp xỉ tròn 0.185 m làm mất đường? | A3, §7.1 |
| 2 | Tốc độ quay tối đa thật của xe? | A2 (`REAL_ROBOT_PROFILE`) |
| 3 | `server.py` + `app.js` của repo có sống trên robot, hay UI khác điều khiển? | A4, A9 |
| 4 | Tắt được `rovera_navigation_adapter` mà giữ AMCL + twist_mux + safety không? | A9, A10 |
| 5 | Robot luôn khởi động từ dock cố định? | B1 (né global localization) |
| 6 | Firmware YB_Car có bản serial packet thường không? | B2 |
| 7 | Có port lớp safety tối giản sang giai đoạn B không? | B3 |

Chỉ #1–#4 chặn giai đoạn A. #5–#7 để lại tới khi bắt đầu B.

## 14. Definition of Done

**Giai đoạn A**

- `src/navigation/` không import `gz`, ROS, driver, `transport`, `adapters`.
- `src/navigation/` không chứa `sensors`, `freshness`, `model_pose`, và không
  chứa hằng số động học hay sensor nào.
- `test_amr_navigator.py` không nhắc tới Gazebo dưới bất kỳ dạng nào.
- `RobotIO` có đúng 5 method; `RobotState` có đúng 6 field.
- 12 Gazebo smoke test pass ở cả hai chế độ (ground truth / fidelity) và cả hai
  đường map (JSON / PGM).
- **Robot đi hết một tuyến A→B thật dưới điều khiển của `src/navigation`.**

**Giai đoạn B**

- Sai số localization trên Gazebo đạt ngưỡng B1.
- Không còn tiến trình ROS nào chạy trên robot.
- `ros_bridge.py` đã xoá.
- Cắm backend mới = viết một file trong `src/adapters/`, không sửa file nào
  trong `src/navigation/`.
