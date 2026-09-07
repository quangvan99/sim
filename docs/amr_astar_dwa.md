# AMR A* + DWA-lite Navigation Plan

## 1. Trạng thái và quyết định kiến trúc

Implementation V1 trong tài liệu này đã hoàn thành trên branch
`feat/amr_astar`. Các giá trị cấu hình đã được calibrate bằng Gazebo smoke test;
contract và hành vi an toàn phải được giữ ổn định khi tiếp tục chỉnh sửa.

| Deliverable | Trạng thái | Kết quả |
|---|---|---|
| D1 — Map contract | Hoàn thành | Ground 20 x 20 m, chỉ 9 trụ; 38.128/38.128 free cell liên thông |
| D2 — A* | Hoàn thành | 8 hướng deterministic, clearance cost, smoothing và resample |
| D3 — DWA-lite | Hoàn thành | LiDAR collision checking, dynamic window và safe stop |
| D4 — Runtime/API | Hoàn thành | State machine, command ownership, re-plan và fail-safe |
| D5 — Browser | Hoàn thành | Top bar + viewer, `Select Goal`, `Go`, `Stop`, `Reset` |
| D6 — Verify/docs | Hoàn thành | Ba goal regression/near-pillar/open-ground đã pass Gazebo độc lập |

Quyết định cho milestone tiếp theo:

- Tự triển khai bằng Python, không dùng ROS 2, Nav2, MPPI hoặc Hybrid-A*.
- Không dùng machine learning hoặc training.
- A* lập đường toàn cục từ vị trí A tới đích B trên static occupancy map.
- DWA-lite bám global path và dùng LiDAR để tránh vật cản tại runtime.
- Dùng trực tiếp Gazebo model pose, odometry vận tốc, LiDAR và `/amr/cmd_vel` hiện có.
- Kết quả phải deterministic, có fail-safe và kiểm thử được bằng unit test.
- UI/runtime hợp nhất manual và autonomous; manual input luôn là operator
  override và hủy mission trước khi phát command.

Luồng navigation runtime:

```text
transport snapshot --> helpers.py --> NavigationInput
                                      | pose + obstacles
StaticMap --> CostMap --> GlobalPlanner --> GlobalPlan
                         |                |
                         +-------+--------+
                                 v
                           LocalPlanner
                                 |
                           ControlDecision
                                 |
                             Navigator
                                 |
                           /amr/cmd_vel
```

`Navigator` chỉ điều phối lifecycle và fail-safe. Raw telemetry không đi qua
sensor boundary; local planner không biết schema transport và global planner không biết
LiDAR.

V1 không teleport robot tới một điểm A tùy ý. A luôn là Gazebo model pose tại thời
điểm bấm `Go` (API tương ứng gọi `Plan`); sau `Reset`, A là spawn pose
`(-2.0, -0.5)`. Cách này tránh
thêm một cơ chế set-pose cạnh tranh với Gazebo physics và làm runtime đơn giản,
an toàn hơn. Chọn A tùy ý có thể là extension riêng sau khi V1 ổn định.

## 2. Phạm vi

### Trong phạm vi V1

- Static 2D map khớp với collision footprint của `amr_world`.
- Inflate obstacle theo footprint robot và safety margin.
- A* 8 hướng, cấm cắt chéo qua góc obstacle.
- Làm gọn path bằng line-of-sight và sinh waypoint.
- DWA-lite cho differential drive, cho phép tiến và quay tại chỗ.
- Tránh vật cản tĩnh hoặc vật cản bất ngờ nhìn thấy bởi LiDAR.
- Re-plan khi đường bị chặn lâu hoặc robot không có tiến triển.
- API điều khiển plan/start/pause/resume/cancel/reset.
- Viewer chọn B, vẽ goal, global path và executed trail trên scene hiện có.
- Fail-safe khi sensor stale, không tìm thấy đường hoặc không có trajectory an
  toàn.
- Unit, contract và Gazebo smoke tests.

### Ngoài phạm vi V1

- ROS 2, Nav2 và ROS bridge.
- Machine learning, training và inference.
- SLAM và tự xây occupancy map từ một môi trường hoàn toàn chưa biết.
- Localization production hoặc SLAM; simulation dùng model pose làm ground truth.
- Dynamic-object tracking/prediction theo vận tốc từng obstacle.
- Multi-robot, traffic management và mission queue.
- Tối ưu final yaw tại B.
- Arbitrary start pose/teleport từ browser.
- Reverse driving trong normal navigation. Robot có thể dừng và quay tại chỗ;
  nếu không thoát được thì chuyển `BLOCKED` thay vì lùi mù.

## 3. Contract đầu vào và đầu ra hiện có

| Dữ liệu | Contract hiện tại | Cách navigation sử dụng |
|---|---|---|
| Odometry | `/amr/odom`, `gz.msgs.Odometry` | Vận tốc thực tế |
| LiDAR | `/amr/scan`, 360 rays, `0..2pi`, `0.12..3.5 m`, 5 Hz | Point cloud cục bộ và kiểm tra clearance |
| Command | `/amr/cmd_vel`, `gz.msgs.Twist` | `linear.x` và `angular.z` |
| Dynamic pose | `/world/amr_world/dynamic_pose/info` | Pose world dùng cho navigation và render |
| Safety limits | `0.90 m/s`, `2.84 rad/s` | Navigation chỉnh `0.30`–`0.90 m/s` theo Speed |
| Dead-man | `0.3 s` | Command heartbeat phải nhanh hơn timeout |

Navigation không lấy pose từ WebSocket browser. Gazebo model pose là nguồn pose
world; odometry chỉ cung cấp vận tốc. Browser chỉ
gửi goal và hiển thị snapshot do server trả về.

DiffDrive odometry bắt đầu tại `(0, 0, 0)` trong frame `odom`, còn robot spawn
ở world pose `(-2.0, -0.5, 0)`. Navigation dùng dynamic model pose nên A*,
browser overlay và temporary obstacle layer đều hoạt động trực tiếp trong world
coordinates; odometry frame chỉ còn phục vụ vận tốc và chẩn đoán.

LiDAR nằm trong frame `base_scan`, lệch khoảng `x=-0.032 m` so với robot base.
Code phải transform ray endpoint sang `base_footprint`; không được giả định
sensor nằm đúng tại tâm robot.

## 4. Static navigation map

Thêm `src/sim/maps/amr_world.json` làm map contract của planner. Map này
không phải SLAM output. Nó là biểu diễn 2D có version của collision geometry
đang nằm trong:

- `src/sim/worlds/main.sdf`
- `src/sim/models/world/model.sdf`

Schema đã triển khai:

```json
{
  "version": "amr-world-nav-v1",
  "world": "amr_world",
  "resolution_m": 0.10,
  "bounds": {"min_x": -10.0, "max_x": 10.0, "min_y": -10.0, "max_y": 10.0},
  "boundary": [[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]],
  "robot_radius_m": 0.13,
  "safety_margin_m": 0.05,
  "obstacles": [
    {"id": "one_one", "type": "circle", "x": -1.1, "y": -1.1, "radius": 0.15},
    {"id": "...", "type": "polygon", "points": [[0.0, 0.0], [1.0, 0.0]]}
  ]
}
```

Boundary là đúng mặt ground `20 x 20 m`. Chỉ chín trụ tròn là obstacle
navigation. Collision và visual trang trí `head`, tay, chân và tường lục giác
được đưa xuống dưới ground trong SDF để không tạo vật cản vô hình trái với UI.

Quy tắc map:

- Cell nằm ngoài navigation bounds được xem là occupied.
- Rasterize circle/polygon vào grid `0.10 m`.
- Inflate obstacle `0.18 m = robot_radius 0.13 + margin 0.05` sau khi calibrate
  với grid và Gazebo collision thực tế.
- Inflation áp dụng cho chín trụ và mép ground.
- Start/goal nằm trong occupied cell bị từ chối.
- Chỉ start model pose được snap sang cell free gần nhất trong recovery radius
  suy ra từ global margin (V1 tối đa ba cell), và đoạn recovery phải raw-free.
  Goal trong occupied/inflated cell luôn bị từ chối; không âm thầm dời B.
- Static map là source dùng chung cho A*, validation API và overlay browser.
- Test phải phát hiện map `world`/version sai và các landmark SDF bị drift.

Map loader chỉ dùng Python standard library và NumPy đang có trong project;
không thêm OpenCV, SciPy hoặc một navigation dependency lớn.

## 5. Global planner A*

### 5.1 Thuật toán

- Grid 8-connected.
- Chi phí cardinal `1`, diagonal `sqrt(2)`.
- Heuristic octile distance, không overestimate.
- Không cho diagonal move nếu một trong hai cell kề góc bị occupied.
- Priority queue dùng `heapq` và monotonic insertion counter để tie-break ổn
  định; cùng map/start/goal luôn trả cùng một path.
- Thêm deterministic clearance cost cho cell cách raw obstacle dưới `0.38 m`.
  Inflation vẫn là hard constraint; clearance cost khiến path đi giữa hành lang
  thay vì bám đúng mép inflation và dành chỗ cho sai số tracking thực tế.
- Trả lỗi rõ ràng cho `START_BLOCKED`, `GOAL_BLOCKED`, `OUT_OF_BOUNDS` và
  `NO_PATH`.

### 5.2 Path post-processing

Raw grid path thường có nhiều điểm gãy. Sau A*:

1. Dùng line-of-sight trên inflated grid để bỏ waypoint thừa, nhưng shortcut
   không được giảm bottleneck clearance của raw A* segment.
2. Không smoothing bằng spline nếu spline có thể đi xuyên obstacle.
3. Resample các segment dài thành waypoint cách nhau tối đa `0.25 m` để
   DWA-lite có mục tiêu cục bộ ổn định.
4. Giữ raw path trong `GlobalPlan` để test planner; status chỉ expose path đã resample.

Path hợp lệ phải thỏa:

- Mọi cell/segment đều nằm trong free space đã inflate.
- Điểm đầu khớp A trong sai số một cell.
- Điểm cuối nằm trong goal tolerance.
- Không có diagonal corner cutting.

## 6. Local planner DWA-lite

### 6.1 Vì sao là DWA-lite

DWA-lite không dựng costmap/local planner framework kiểu Nav2. Mỗi control
cycle chỉ:

1. Lấy pose, velocity và scan mới nhất.
2. Chọn local target trên global path.
3. Sinh tập hữu hạn candidate `(v, w)` reachable từ velocity hiện tại.
4. Forward-simulate differential-drive trajectory.
5. Loại trajectory va vào LiDAR point hoặc static inflated map.
6. Chấm điểm các trajectory còn lại và publish candidate tốt nhất.

### 6.2 Thông số đã triển khai

| Tham số | Giá trị V1 |
|---|---:|
| DWA evaluation rate | `5 Hz`, đồng bộ scan mới |
| Command heartbeat | `10 Hz` |
| Min linear velocity | `-0.06 m/s` |
| Max linear velocity | Mặc định `0.50`, điều chỉnh `0.30`–`0.90 m/s` bằng nút Speed |
| Max angular velocity | `1.50 rad/s` |
| Linear acceleration | `0.50 m/s^2` |
| Angular acceleration | `2.00 rad/s^2` |
| Prediction horizon | `2.0 s` |
| Simulation step | `0.10 s` |
| Linear samples | `7` và luôn thêm `v=0` |
| Angular samples | `21` và luôn thêm `w=0` |
| Lookahead distance | `0.45 m` |
| Goal tolerance | `0.04 m` |
| Emergency clearance | `0.16 m` |
| No-progress timeout | `3.0 s` |
| Temporary obstacle TTL | `5.0 s` |

Tất cả giới hạn navigation nhỏ hơn giới hạn vật lý hiện tại. Các giá trị này đã
được kiểm tra với physics thật bằng Gazebo smoke test direct và static detour.

### 6.3 Dynamic window

Với chu kỳ `dt`, cửa sổ vận tốc reachable được giới hạn bởi:

```text
v_min = max(min_v,   current_v - linear_accel  * dt)
v_max = min(max_v,   current_v + linear_accel  * dt)
w_min = max(-max_w,  current_w - angular_accel * dt)
w_max = min( max_w,  current_w + angular_accel * dt)
```

`v=0` và `w=0` luôn được thêm như safety candidates, kể cả khi nằm ngoài cửa
sổ gia tốc một chu kỳ, để robot có thể phanh hoặc quay tại chỗ thay vì báo
blocked trước khi base kịp dừng. Vận tốc lùi được phép nhưng có penalty nên
chỉ được chọn khi an toàn và có lợi hơn tiến hoặc xoay.

Mỗi candidate được tích phân bằng mô hình differential-drive:

```text
x'     = x + v * cos(yaw) * dt
y'     = y + v * sin(yaw) * dt
yaw'   = wrap(yaw + w * dt)
```

### 6.4 Collision và LiDAR

- Chuyển từng LiDAR range hợp lệ thành point trong robot frame.
- `+inf`/no-return thành max range; `NaN`, thiếu ray hoặc scan stale là lỗi,
  không được coi là free space.
- Transform đúng extrinsic `base_scan -> base_footprint`.
- Kiểm tra toàn bộ simulated trajectory với local LiDAR points và static map.
- Candidate bị loại nếu clearance nhỏ hơn robot radius `0.13 m` + local
- safety margin `0.03 m`. Global map dùng margin `0.05 m`.
- Nếu pose có sai số đặt footprint vừa lọt vào local margin nhưng
  chưa collision, DWA chỉ cho stop/quay hoặc trajectory không làm clearance
  giảm thêm; nhờ đó robot có thể xoay để thoát thay vì false `BLOCKED`.
- Nếu model pose lọt vào inflation của static map nhưng chưa nằm trong raw
  obstacle, DWA chỉ cho trajectory giữ hoặc tăng raw-map clearance cho tới khi
  thoát vùng đệm. Raw obstacle vẫn luôn bị cấm.
- Không khóa cứng theo front arc: mọi trajectory tiến, lùi và xoay đều được
  kiểm tra với scan 360 độ; trajectory chỉ hợp lệ khi không làm clearance nhỏ
  hơn trạng thái hiện tại.
- Không dùng LiDAR làm collision truth sau va chạm; smoke test kiểm chứng không
  contact bằng Gazebo/pose và khoảng cách bảo thủ.

V1 coi obstacle đứng yên trong prediction horizon 2 giây. Đây là cách bảo thủ
và deterministic; chưa làm velocity tracking cho dynamic obstacle.

### 6.5 Scoring

Mọi thành phần được normalize về `[0, 1]` trước khi cộng trọng số:

```text
score =
    w_progress  * tiến triển dọc global path
  + w_heading   * hướng về local target
  + w_clearance * khoảng cách tới obstacle
  + w_speed     * độ lớn vận tốc
  - w_path      * lệch khỏi global path
  - w_smooth    * thay đổi so với command trước
  - reverse_penalty nếu chạy lùi
```

Ưu tiên ban đầu: safety/clearance, sau đó progress/path, cuối cùng speed. Test
khóa normalized terms và tie-break order để thay đổi weight không tạo hành vi
khó giải thích.

Khi hai candidate có score bằng nhau, tie-break cố định theo thứ tự:

1. Clearance lớn hơn.
2. Progress lớn hơn.
3. `v` lớn hơn để ưu tiên tiến khi các phương án thực sự tương đương.
4. `abs(w)` nhỏ hơn.

Nếu có candidate tiến với progress dương đã qua toàn bộ collision checks, DWA
ưu tiên candidate tiến tốt nhất thay vì idle `(0, 0)`. Quy tắc liveness này
không nới clearance và tránh local minimum dẫn tới false no-progress re-plan.

## 7. Vật cản runtime và re-plan

DWA-lite xử lý obstacle ngắn hạn trực tiếp từ scan. Tuy nhiên nếu global path
đi xuyên qua một obstacle mới đứng yên, chỉ steering cục bộ có thể gây dao
động. V1 vì vậy có một temporary obstacle layer:

- Project cụm LiDAR gây chặn từ robot frame sang world frame bằng model pose.
- Rasterize vào overlay grid với cùng inflation như static map.
- Mỗi cell tạm có TTL mặc định `5 s`; scan thấy lại sẽ refresh TTL.
- Không ghi đè file static map.
- Trigger A* re-plan khi không có safe forward trajectory hoặc progress dưới
  ngưỡng trong `8 s` (đủ thời gian xoay tại chỗ ở vùng hẹp).
- No-progress recovery chỉ dùng static map vì DWA vẫn có trajectory an toàn;
  chỉ lỗi collision từ DWA mới thêm obstacle runtime vào temporary layer.
- Re-plan bắt đầu từ model pose hiện tại tới B trên `static + temporary` grid.
- Tối đa hai lần recovery/re-plan liên tiếp trong `10 s`.
- Nếu vẫn không có path, chuyển `BLOCKED` và giữ command zero.

Obstacle di chuyển ra chỗ khác sẽ hết TTL, cho phép lần plan sau dùng lại vùng
free. Cơ chế này chưa phải SLAM: overlay chỉ tồn tại trong RAM của mission.

## 8. Navigation runtime

Thêm `Navigator` sở hữu một worker thread. Chỉ navigator được quyền gọi
`AMRTransport.set_velocity()` trong lúc autonomous mission active.

Control ownership:

- Manual command được phép ở mọi state.
- Nếu mission đang sở hữu control, manual endpoint gọi `cancel()` trước rồi
  mới publish command của operator; không có hai nguồn cùng điều khiển.
- `Pause`, `Cancel`, error, stale sensor và shutdown phải publish zero trước
  khi đổi state.

Worker dùng `time.monotonic()` cho timeout, không dùng wall clock. Global planner và
local planner không `sleep()` bên trong thuật toán; chỉ outer loop chờ tới control
deadline.

### 8.1 State machine

```text
IDLE --Plan(B)--> PLANNING --path ok--> READY --Start--> NAVIGATING
                         \--failed-----------------------------> ERROR

NAVIGATING --goal reached-------------------------------> SUCCEEDED
NAVIGATING --Pause--> PAUSED --Resume-------------------> NAVIGATING
NAVIGATING --stale sensor / exception--------------------> ERROR
NAVIGATING --blocked / no progress-----------------------> BLOCKED

PLANNING / READY / NAVIGATING / PAUSED
                         --Cancel-------------------------> CANCELLED

Reset từ mọi state: publish zero -> RESETTING -> reset world -> IDLE
```

`SUCCEEDED` chỉ xảy ra khi distance tới B `<= 0.04 m`; Navigator publish zero và
không tự bắt đầu mission khác.

### 8.2 Sensor synchronization và fail-safe

`AMRTransport` cần sequence counter/timestamp cho LiDAR, odometry và model pose để
Navigator biết dữ liệu nào mới. Navigation chỉ tính DWA với scan mới; heartbeat
có thể publish lại command an toàn giữa hai scan.

- Navigation sensor timeout dự kiến `0.6 s` (ba chu kỳ LiDAR).
- LiDAR, odometry hoặc model pose stale: command zero, state `ERROR`.
- Scan sai `count=360`, chứa `NaN`, hoặc metadata góc/range sai: `ERROR`.
- Server/Gazebo shutdown: zero velocity best-effort.
- Exception trong worker không được làm thread chết âm thầm; lưu error message,
  publish zero và chuyển `ERROR`.

## 9. HTTP API

Manual API hiện có được giữ nguyên. Thêm:

| Method | Endpoint | Ý nghĩa |
|---|---|---|
| `POST` | `/api/navigation/plan` | Lấy A từ model pose hiện tại, validate B và chạy A* |
| `POST` | `/api/navigation/start` | Bắt đầu bám path đã plan |
| `POST` | `/api/navigation/pause` | Dừng robot, giữ mission/path |
| `POST` | `/api/navigation/resume` | Tiếp tục từ pose hiện tại, re-plan nếu cần |
| `POST` | `/api/navigation/cancel` | Dừng và hủy mission hiện tại |
| `GET` | `/api/navigation/status` | State, path, local trajectory, metrics và error |
| `POST` | `/api/amr/speed` | Chọn `speed_mps` trong khoảng `0.30`–`0.90` cho browser/DWA |

Plan request:

```json
{
  "goal": {"x": 2.2, "y": 1.8}
}
```

Plan response rút gọn:

```json
{
  "ok": true,
  "state": "READY",
  "start": {"x": -2.0, "y": -0.5},
  "goal": {"x": 2.2, "y": 1.8},
  "path_length_m": 5.4,
  "waypoint_count": 24
}
```

Status snapshot:

```json
{
  "state": "NAVIGATING",
  "start": {"x": -2.0, "y": -0.5},
  "goal": {"x": 2.2, "y": 1.8},
  "distance_to_goal_m": 1.47,
  "global_path": [{"x": -2.0, "y": -0.5}],
  "command": {"linear": 0.14, "angular": -0.32},
  "minimum_clearance_m": 0.43,
  "error": null
}
```

Validation dùng strict numeric JSON, từ chối bool/string/extra fields. Goal
ngoài map hoặc trong inflated obstacle trả `422`; state conflict trả `409`;
sensor chưa ready trả `503`.

## 10. Browser wireframe

Browser giữ layout tối giản với một top bar cao khoảng
`48 px`, phần còn lại là Gazebo viewer toàn màn hình. Không thêm dashboard,
sidebar, bảng metric hoặc nhiều mode điều khiển trên màn hình.

A là robot pose khi bấm `Go`. B được chọn bằng raycast xuống ground. Viewer chỉ
vẽ những thông tin cần để hiểu mission: marker B, global path và đường robot đã
đi. Candidate DWA, raw LiDAR và temporary costmap vẫn có trong status phục vụ
test/debug nhưng không render mặc định.

```text
+--------------------------------------------------------------------------------+
| Connected | Sensors 4/4 | NAVIGATING | B: 2.20, 1.80 | 1.47 m                  |
|                                      [Select Goal] [Go] [Stop] [Reset]          |
+--------------------------------------------------------------------------------+
|                                                                                |
|                              GAZEBO 3D VIEWER                                  |
|                                                                                |
|        A [AMR>] ---- executed trail                                            |
|                    \                                                           |
|                     o======o======o       +----------+                         |
|                         global path       | obstacle |                         |
|                                      o====+          +====o===== ◎ B            |
|                                                                                |
|                         Click ground to select goal                            |
|                                                                                |
+--------------------------------------------------------------------------------+
```

Luồng thao tác chính chỉ có ba bước:

```text
Select Goal -> click B trên ground -> Go
```

Button behavior:

- `Select Goal`: bật chế độ click ground để đặt hoặc đổi B.
- `Go`: chụp model pose hiện tại làm A, gọi `/plan`; nếu có path thì gọi `/start`
  ngay. User không phải bấm riêng `Plan` rồi `Start`.
- `Stop`: cancel mission và publish zero ngay; marker B được giữ để có thể bấm
  `Go` lại.
- `Reset`: cancel trước, reset Gazebo, xóa path/trail và trở về `IDLE`.
- `Go` disabled khi chưa chọn B, sensor chưa ready hoặc mission đang chạy.
- Manual keyboard/buttons luôn hiển thị cạnh navigation controls. Nhấn WASD
  hoặc nút manual trong lúc AUTO chạy sẽ cancel mission và chuyển quyền ngay.

Top bar chỉ hiển thị:

- Gazebo connection.
- Sensor freshness `n/4`.
- Navigation state.
- Tọa độ B và distance còn lại.
- Bốn button `Select Goal`, `Go`, `Stop`, `Reset`.

Các trạng thái dài được rút thành thông báo ngắn trong top bar: `Select goal`,
`Planning`, `Moving`, `Avoiding obstacle`, `Goal reached`, `Blocked` hoặc
`Sensor error`.

Màu overlay:

- B: vàng.
- Global path: xanh dương.
- Executed trail: cyan.
- Invalid goal/no-path: marker đỏ và message rõ ràng.

## 11. Kịch bản hoạt động A -> B

### Kịch bản 1 — Đường thẳng không bị chặn

```text
A [AMR>] =============================== ◎ B
            A* path thẳng
            DWA giữ heading và tăng tốc trong limit
```

1. User chọn B và bấm `Go`.
2. Browser gọi plan rồi start; A* trả path thẳng với ít waypoint.
3. DWA ưu tiên progress/speed nhưng vẫn kiểm tra mọi scan.
4. Khi tâm robot cách B không quá `0.04 m`, robot về zero và state `SUCCEEDED`.

### Kịch bản 2 — Static obstacle đã có trên map

```text
                         o======o
                       /          \
A [AMR>] =======o=====+    #####   +=====o===== ◎ B
                            #####
                         static obstacle
```

1. A* thấy inflated obstacle trên static grid.
2. Path global đi vòng với clearance đủ cho footprint robot.
3. DWA chọn local target dọc path, không cắt góc chỉ để rút ngắn quãng đường.
4. LiDAR xác nhận obstacle thật và loại các trajectory quá gần.
5. Robot vòng qua obstacle rồi hội nhập segment tiếp theo.

### Kịch bản 3 — Vật cản bất ngờ không có trong static map

```text
Global path ban đầu:
A =================== X =================== B
                      obstacle mới

DWA tránh cục bộ:
A ==========o       .----.
             \\     /  X   \\____ rejoin global path ===== B
              [AMR>]

Nếu X chặn lâu:
LiDAR cluster -> temporary grid + TTL -> A* re-plan -> path mới
```

1. Scan mới phát hiện obstacle nằm trên candidate đi thẳng.
2. Candidate nguy hiểm bị loại trước khi publish command.
3. Nếu còn cung tránh an toàn, DWA đi vòng và hướng về waypoint xa hơn.
4. Mỗi scan mới cập nhật temporary costmap và cost-to-go field ngay trong
   `NAVIGATING`.
5. Nếu không có path hoặc robot không tiến triển trong `8 s`, Navigator publish
   zero; hết timeout thì chuyển `BLOCKED`.

### Kịch bản 4 — Lối đi hoàn toàn bị khóa

```text
A [AMR>] ======== ######## wall mới ######## ======== B
                 không còn corridor hợp lệ
```

- DWA không tìm được trajectory an toàn nên command luôn zero.
- A* re-plan với temporary layer trả `NO_PATH`.
- State thành `BLOCKED`; browser hiển thị lý do và không tự thử vô hạn.
- User có thể dọn obstacle rồi bấm `Go`, hoặc `Stop`/`Reset`.

### Kịch bản 5 — Sensor mất hoặc stale

```text
NAVIGATING -> LiDAR/odom stale > 0.6 s -> ZERO CMD -> ERROR
```

Robot không tiếp tục bằng scan cũ. User chỉ có thể plan lại sau khi sensor fresh
hoặc reset simulation.

## 12. Deliverables và file thay đổi

### D1 — Navigation contract và static map

Mục tiêu: có map 2D versioned khớp world hiện tại và geometry primitives dùng
chung cho validation/planning.

File thêm:

- `src/navigation/types.py`
- `src/navigation/helpers.py`
- `src/navigation/global_planner.py`
- `src/navigation/local_planner.py`
- `src/navigation/navigator.py`
- `src/sim/maps/amr_world.json`
- `src/tests/test_amr_navigation_map.py`

`types.py` chỉ định nghĩa contract, `helpers.py` chứa geometry thuần và parse
sensor, còn `global_planner.py` sở hữu static map, temporary costmap và global
planning. `local_planner.py` xử lý DWA; `navigator.py` điều phối lifecycle.

File sửa:

- `src/config.py`: thêm `NavigationConfig`, map path, footprint, timing và
  threshold tập trung.
- `pyproject.toml`: bảo đảm navigation map JSON được đóng gói trong wheel.

Nghiệm thu:

- Load/raster/inflate map deterministic.
- Known-free và known-occupied fixtures khớp world SDF.
- Start spawn `(-2.0, -0.5)` là free sau inflation.
- Goal validation chỉ từ chối khi footprint tại B chồng trụ hoặc ra ngoài
  mép ground; browser giữ chế độ chọn và yêu cầu chọn lại B.

### D2 — Global A* planner

File thêm:

- `src/navigation/global_planner.py`
- `src/tests/test_amr_astar.py`

Nghiệm thu:

- Direct, detour, no-path, blocked-start/goal đều có test.
- Không corner cutting.
- Path smoothing không xuyên occupied cell.
- Cùng input luôn ra cùng path.

### D3 — DWA-lite local planner

File thêm:

- `src/navigation/local_planner.py`
- `src/tests/test_amr_dwa.py`

Nghiệm thu:

- Candidate luôn nằm trong velocity/acceleration limits.
- Không obstacle: chọn tiến về local target.
- Obstacle trước mặt: không chọn trajectory va chạm.
- Có khe an toàn: chọn hướng vòng đúng.
- Không có candidate: trả explicit safe-stop result.
- LocalPlanner chỉ nhận obstacle points đã validate; invalid/stale scan bị chặn
  tại sensor boundary trước khi local planner chạy.

### D4 — Runtime orchestration và API

File thêm:

- `src/navigation/navigator.py`
- `src/tests/test_amr_navigator.py`

File sửa:

- `src/transport.py`: thêm sensor sequence/timestamps cần cho synchronized
  control snapshot; giữ nguyên topic và manual behavior.
- `src/server.py`: khởi tạo/shutdown `Navigator`, thêm API và command
  ownership guard.

Nghiệm thu:

- State machine đúng contract.
- Pause/cancel/error/shutdown publish zero.
- Manual command cancel AUTO rồi chiếm command atomically.
- Re-plan có giới hạn, không busy-loop.
- Worker exception được phản ánh qua status và không để robot chạy tiếp.

### D5 — Browser navigation UX

File sửa:

- `src/sim/visualize/index.html`
- `src/sim/visualize/app.js`
- `src/sim/visualize/styles.css`

Nghiệm thu:

- Click ground chọn B và invalid click không phá goal hợp lệ trước đó.
- `Go` thực hiện plan rồi start; lỗi plan không làm robot chuyển động.
- Vẽ B, global path và executed trail.
- Top bar và bốn button chính hoạt động đúng state.
- Manual và navigation controls cùng hiển thị trong một UI.
- WebSocket reconnect không tự start lại mission đã dừng.

### D6 — Gazebo integration, tài liệu và regression

File thêm:

- `src/sim/worlds/navigation_test.sdf`: world nhỏ, primitive collision và
  scenario cố định để smoke test nhanh, không bind thêm WebSocket port.
- `src/tests/test_amr_navigation_sim_smoke.py`

File sửa:

- `README.md`: một lệnh chạy AMR hợp nhất manual/navigation và API demo.
- `docs/amr.md`: cập nhật trạng thái milestone sau khi implementation pass.

Chỉ sửa `src/sim/worlds/main.sdf` hoặc model khi smoke test chứng minh world
hiện tại thiếu contract cần thiết. Không đổi topic, geometry hoặc physics chỉ
để làm unit test dễ hơn.

Nghiệm thu:

- Scenario direct: tới B và dừng trong tolerance.
- Scenario static detour: A* đi vòng, không collision.
- Scenario runtime obstacle: DWA stop/avoid hoặc re-plan an toàn.
- Scenario blocked: state `BLOCKED`, command zero.
- Sensor stale/shutdown: fail-safe zero.
- Toàn bộ test hiện tại vẫn pass.

## 13. Thứ tự triển khai

1. D1: đo/calibrate world footprint, khóa map schema và config.
2. D2: implement/test A* độc lập với Gazebo.
3. D3: implement/test DWA-lite bằng synthetic scan và fake odometry.
4. D4: ghép Navigator với `AMRTransport`, API và command ownership.
5. Chạy headless Gazebo để tune footprint, clearance và timing.
6. D5: thêm browser overlay và controls sau khi API ổn định.
7. D6: smoke test các scenario, regression và cập nhật tài liệu trạng thái.

Không bắt đầu browser trước khi A*, DWA và state machine đã được test bằng API.

## 14. Test strategy

### Unit test — không cần Gazebo

- Coordinate/world-grid transform và boundary.
- SDF/DAE collision geometry phải khớp static JSON map.
- Tất cả free cell phải liên thông từ spawn theo đúng no-corner-cut của A*.
- Rasterization/inflation circle/polygon.
- A* heuristic, tie-break, no-corner-cut và smoothing.
- LiDAR preprocessing/extrinsic transform.
- Dynamic window, trajectory rollout, collision rejection và score.
- Temporary obstacle TTL.
- State machine, no-progress, re-plan budget và fail-safe.
- Strict API validation và command ownership.

Synthetic tests không assert chính xác một floating-point command khi nhiều
candidate tương đương; chúng assert safety invariant và deterministic selected
candidate với fixture đã khóa.

### Integration test — Gazebo opt-in

- Topics/sensors sẵn sàng trước khi plan.
- Robot đi thẳng, quay và dừng.
- Đi từ spawn A tới các B cố định với static detour.
- Không chạm obstacle và minimum clearance không dưới ngưỡng.
- Goal success dựa trên model pose, không dựa trên browser.
- Reset giữa scenario để test độc lập.

Edge-case sweep đã chạy từ spawn `(-2.0, -0.5)`:

| Vùng B | Goal | Kết quả |
|---|---:|---|
| Direct | `(-2.00, 0.50)` | PASS |
| Static detour | `(-0.55, 0.55)` | PASS |
| Goal từng bị BLOCKED | `(-0.44, 0.43)` | PASS |
| Goal gần trụ giữa | `(0.19, -0.35)` | PASS |
| Phía trái | `(-2.45, 0.00)` | PASS |
| Phía trên | `(0.00, 2.00)` | PASS |
| Phía dưới | `(0.00, -2.00)` | PASS |
| Corridor giữa | `(0.00, 0.55)` | PASS |
| Vùng hợp lệ xa nhất bên phải | `(2.15, 0.55)` | PASS |
| Ngoài boundary lục giác cũ | `(1.70, 3.99)` | PASS |

Exhaustive graph test chứng minh `38.128/38.128` cell free sau inflation đều
reachable từ spawn. Goal `(2.45, 0.0)` và `(1.70, 3.99)` nay hợp lệ vì các
collision trang trí không còn nằm trên ground; chỉ chín trụ chặn A*/DWA.

### Browser/manual check

- Chọn B trên ground ở desktop và touch.
- Overlay bám đúng world coordinates khi camera orbit/zoom.
- Stop/reset dừng ngay.
- Mất WebSocket chỉ ảnh hưởng render; mất HTTP/sensor phải dừng navigation.
- WASD/nút manual điều khiển được ở mọi navigation state và cancel AUTO trước.

## 15. Definition of Done

Milestone được xem là hoàn thành khi:

- A* tạo path collision-free trên inflated static map và trả lỗi rõ ràng khi
  không có đường.
- DWA-lite bám path, tránh obstacle nhìn thấy bởi LiDAR và không vi phạm
  velocity/acceleration limit.
- Persistent runtime obstacle trigger re-plan hữu hạn; không loop vô tận.
- Robot tới B trong tolerance rồi publish zero.
- Sensor stale, pause, cancel, blocked, exception và shutdown đều dẫn tới zero
  command.
- Browser tối giản thể hiện đúng B, global path, executed trail và state.
- Không có dependency ngoài classical navigation và Gazebo Transport.
- Unit/contract tests và Gazebo smoke tests đều pass.
- README đủ để clone repo, chạy Gazebo + server và thực hiện mission A -> B.

## 16. Các default đã triển khai

Implementation V1 đang dùng các default sau:

1. A là model pose hiện tại; V1 không cho click/teleport A.
2. Goal chỉ yêu cầu vị trí, không yêu cầu final yaw.
3. DWA cho phép lùi tới `-0.06 m/s`, có penalty để tránh lùi khi không cần.
4. Ground `20 x 20 m`, map resolution `0.10 m`, inflation `0.18 m`.
5. DWA evaluate theo LiDAR `5 Hz`, command heartbeat `10 Hz`.
6. Vật cản runtime tồn tại lâu được đưa vào temporary map TTL `5 s` để re-plan.
7. Browser polling navigation status, không thêm một application WebSocket mới.

Implementation không mang code RL trên local branch `amr` vào branch A* +
DWA-lite. Nếu đổi một default ảnh hưởng geometry, frame hoặc timing thì phải
chạy lại full unit suite và Gazebo navigation smoke tests.
