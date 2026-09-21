# Nav2 Adapter Plan

## 1. Mục tiêu

Thêm Nav2 như một navigation backend thứ hai cho AMR simulator mà không làm
thay đổi API và Web UI hiện tại.

Kết quả mong muốn:

- Có thể chọn `custom` hoặc `nav2` bằng cấu hình.
- Cả hai backend dùng chung HTTP API và Web UI.
- Nav2 chạy trong container ROS 2 riêng, không ép `rclpy` vào môi trường Python
  3.14 hiện tại.
- Nav2 phải vượt qua toàn bộ 12 Gazebo smoke case hiện có.
- Chưa xóa navigation custom cho đến khi Nav2 ổn định và có kết quả so sánh.

Baseline hiện tại:

- Navigation custom: Dijkstra cost-to-go + DWA-lite.
- Unit suite: 59 tests pass, 12 smoke tests được skip mặc định.
- Full Gazebo smoke suite: 12/12 pass.
- Docker image hiện tại: `amr-app:latest`.

## 2. Ngoài phạm vi giai đoạn đầu

- Không thay Web UI.
- Không thay REST API công khai.
- Không xóa `src/navigation`.
- Không yêu cầu SLAM và EKF ngay trong lần tích hợp đầu tiên.
- Không copy nguyên tham số của `robot_nav`; kích thước và động học của hai
  robot khác nhau.

## 3. Kiến trúc mục tiêu

```text
Browser
   │ HTTP
   ▼
FastAPI server
   │
   ├── CustomNavigationAdapter ──> CustomNavigator hiện tại
   │
   └── Nav2NavigationAdapter ──HTTP/JSON──> Nav2 gateway (ROS 2)
                                               │
                                               ├── NavigateToPose actions
                                               ├── Costmaps / planner / controller
                                               └── ROS topics and TF
                                                        │
                                             ros_gz_bridge / sim bridge
                                                        │
                                                        ▼
                                                   Gazebo AMR
```

Biến môi trường chọn backend:

```env
NAVIGATION_BACKEND=custom
NAVIGATION_BACKEND=nav2
```

`custom` vẫn là mặc định trong suốt quá trình tích hợp.

## 4. Kiểm tra tương thích trước khi triển khai

`robot_nav` hiện dùng ROS 2 Humble, trong khi simulator đang dùng Gazebo Jetty
trên Ubuntu 26.04. Không nên đặt `rclpy`, Nav2 và Gazebo Python bindings vào cùng
một Python environment.

Trước khi viết adapter hoàn chỉnh, cần làm một compatibility spike:

1. Chọn ROS 2/Nav2 distribution có `ros_gz_bridge` tương thích với Gazebo đang
   dùng.
2. Ưu tiên Humble nếu có bridge đã được kiểm chứng để gần `robot_nav` nhất.
3. Nếu Humble không tương thích trực tiếp, dùng một ROS 2/Nav2 distribution phù
   hợp cho simulator nhưng giữ topic, frame và Nav2 contract tương thích Humble.
4. Chỉ tiếp tục khi bridge được bốn đường dữ liệu tối thiểu:

   - Gazebo `/amr/scan` sang ROS `/scan`.
   - Gazebo `/amr/odom` sang ROS `/odom`.
   - ROS `/cmd_vel` sang Gazebo `/amr/cmd_vel`.
   - Gazebo `/clock` sang ROS `/clock`.

Điều kiện pass của spike:

- Tần số `/scan` và `/odom` ổn định trong ít nhất 60 giây.
- Gửi `/cmd_vel` làm robot di chuyển và gửi zero làm robot dừng.
- ROS time tăng đều, không nhảy lùi.
- Không mất bridge khi Gazebo reset world.

## 5. Navigation adapter contract

Tạo protocol dùng chung, bám theo các thao tác mà `server.py` đang gọi:

```python
class NavigationAdapter(Protocol):
    @property
    def owns_control(self) -> bool: ...

    def plan(self, goal: Point2D) -> NavigationStatus: ...
    def navigate(self, goal: Point2D) -> NavigationStatus: ...
    def is_goal_free(self, goal: Point2D) -> bool: ...
    def start(self) -> NavigationStatus: ...
    def cancel(self) -> NavigationStatus: ...
    def reset(self) -> NavigationStatus: ...
    def set_speed_mps(self, speed_mps: float) -> NavigationStatus: ...
    def status(self) -> NavigationStatus: ...
    def close(self) -> None: ...
```

Hai implementation:

- `CustomNavigationAdapter`: wrapper mỏng quanh `CustomNavigator` hiện tại.
- `Nav2NavigationAdapter`: gọi Nav2 gateway và chuyển response về
  `NavigationStatus` hiện tại.

`server.py` chỉ phụ thuộc `NavigationAdapter`. Việc chọn implementation nằm ở
application bootstrap, không nằm trong HTTP endpoint.

## 6. Ánh xạ API hiện tại sang Nav2

### Plan

API hiện tại tách `plan` và `start`, trong khi `NavigateToPose` thường lập kế
hoạch và di chuyển trong cùng một action.

Luồng chính dùng `navigate(goal)`: gateway gửi `NavigateToPose` một lần; Nav2
tự lập global path trong action. Adapter vẫn mô phỏng `plan`/`start` cũ ở phía
app để giữ contract chung, nhưng gateway không cần các endpoint trung gian.

Goal có dạng `{x, y, yaw}`; `yaw` dùng radian trong HTTP/ROS contract. WebUI
hiển thị và cho nhập yaw theo độ. Nếu bỏ qua `yaw`, gateway giữ hành vi cũ và
tự dùng hướng từ pose hiện tại tới goal.

Nav2 tự xác nhận goal bằng `SimpleGoalChecker` (`xy=0.04 m`, `yaw=0.03 rad`).
Khi action trả `SUCCEEDED`, gateway trả `SUCCEEDED` ngay; gateway không tự phát
vận tốc và không có controller settle thứ hai.

### Validate goal

WebUI không gọi pre-validation riêng. Goal không hợp lệ được Nav2 báo qua action
result (`BLOCKED`/`ERROR`), tránh một request planning bị lặp.

### Cancel

`cancel()` hủy action đang chạy. Nav2 controller/behavior sở hữu output `/cmd_vel`;
gateway không publish command cạnh tranh.

### Reset

`reset()` thực hiện theo thứ tự:

1. Adapter cancel goal hiện tại.
2. Reset Gazebo world.
3. Clear global và local costmap.
4. Chờ `/clock`, `/scan`, `/odom` và TF mới.
5. Trả trạng thái `IDLE` khi toàn bộ dữ liệu đã fresh.

### Speed

`set_speed_mps()` dùng Nav2 speed limit. Adapter vẫn trả `speed_mps` theo schema
hiện tại.

### Status

Ánh xạ lifecycle/action của Nav2 về state hiện tại:

| Nav2 | API state |
|---|---|
| Chưa có goal | `IDLE` |
| Goal đã lưu, chưa NavigateToPose | `READY` |
| Action accepted/running | `NAVIGATING` |
| Action succeeded | `SUCCEEDED` |
| Action canceled | `CANCELLED` |
| Recovery đang chạy | `NAVIGATING` |
| Không còn recovery hợp lệ | `BLOCKED` |
| Bridge, TF hoặc Nav2 lỗi | `ERROR` |

`distance_to_goal_m`, yaw error, speed và error tiếp tục được gateway trả về.
Các field telemetry không có trong Nav2 gateway sẽ dùng giá trị mặc định ở
adapter; navigation không phụ thuộc vào chúng.

## 7. Nav2 gateway

Nav2 gateway là một ROS 2 node nhỏ chạy trong container Nav2. Node này dùng
`rclpy` và cung cấp HTTP/JSON nội bộ cho FastAPI container.

Các endpoint nội bộ đề xuất:

```text
GET  /health
GET  /status
POST /navigate
POST /cancel
POST /reset
POST /speed
POST /initialpose
```

Gateway sở hữu:

- `NavigateToPose` action client.
- Global/local costmap clear clients.
- Subscription `/odom` để cập nhật pose và publish `map -> odom`.
- Không tự publish `/cmd_vel`, không tự plan và không có settling controller.

FastAPI không truy cập ROS trực tiếp. Điều này giữ dependencies của app hiện tại
nhỏ và tránh xung đột Python ABI.

## 8. ROS topics và TF

Topic contract cho simulator:

| Dữ liệu | Gazebo | ROS 2 |
|---|---|---|
| LiDAR | `/amr/scan` | `/scan` |
| Odometry | `/amr/odom` | `/odom` |
| Velocity command | `/amr/cmd_vel` | `/cmd_vel` |
| Simulation clock | `/clock` | `/clock` |

TF tối thiểu:

```text
map -> odom -> base_footprint -> base_link -> laser_frame
```

Smoke profile hiện dùng ground-truth localization để tránh trộn lỗi AMCL với
planner/controller:

- Gateway publish dynamic `map -> odom` từ pose khởi tạo và `/odom`.
- `odom -> base_footprint` từ Gazebo odometry.
- Static transforms cho `base_link` và `base_scan` theo model SDF.
- AMCL không chạy trong `nav2_sim.launch.py`.
- Endpoint nội bộ `/initialpose` re-anchor localization sau khi test teleport
  robot bằng Gazebo `set_pose`.

Cách này kiểm thử planner/controller trước, không trộn thêm lỗi SLAM hoặc EKF.
Sau khi Nav2 pass, có thể thêm `slam_toolbox` và `robot_localization` giống
`robot_nav`.

## 9. Static map cho Nav2

Nav2 cần ROS occupancy map. Map hiện tại nằm tại
`src/sim/maps/amr_world.json`.

Thêm các asset:

```text
ros/nav2/maps/amr_world.pgm
ros/nav2/maps/amr_world.yaml
```

Thêm script hoặc build step tạo ROS map từ JSON hiện tại để tránh duy trì hai
nguồn dữ liệu bằng tay.

Contract test phải xác nhận:

- Bounds và resolution giống JSON.
- Obstacle footprint giống SDF trong sai số một cell.
- Map version đồng nhất.
- Start và 12 smoke goals nằm đúng vùng dự kiến.

## 10. Nav2 simulator configuration

Tạo `ros/nav2/config/nav2_sim.yaml`. Không dùng trực tiếp `nav2.yaml` của
`robot_nav`.

Thông số ban đầu lấy từ navigation custom để so sánh công bằng:

```text
use_sim_time: true
robot_radius: 0.13
goal_tolerance: 0.04
max_linear_velocity: 0.90
max_angular_velocity: 1.50
global_resolution: 0.05
local_resolution: 0.05
```

Plugin dùng cho vòng smoke hiện tại:

- Global planner: NavFn với `use_astar: false` để gần Dijkstra hiện tại.
- Local controller: Regulated Pure Pursuit để ổn định trên các path smoke.
- Global costmap: static + obstacle + inflation.
- Local costmap: rolling obstacle + inflation.
- Behaviors: spin, backup, wait, clear costmap.
- Progress checker: timeout tương đương 8 giây hiện tại.
- Không chạy docking, route, waypoint follower hoặc collision monitor trong
  smoke profile; RPP vẫn bật collision checking và behavior server vẫn cung cấp
  spin/backup/wait recovery.

Chỉ tuning sau khi có log cho một smoke case thất bại. Không thay nhiều nhóm
tham số cùng lúc.

## 11. Docker Compose

Giữ `docker-compose.yml` hiện tại cho backend custom. Thêm
`docker-compose.nav2.yml` hoặc Compose profile `nav2`.

Các service dự kiến:

```text
amr-sim       Gazebo server + visualization API
amr-nav2      ROS 2 + ros_gz_bridge + Nav2 + gateway
```

Trong lần đầu, dùng host networking trên Linux để Gazebo Transport và DDS trao
đổi ổn định giữa hai container. Cả hai service dùng cùng:

```env
GZ_PARTITION=amr_sim
ROS_DOMAIN_ID=20
```

Health check của `amr-nav2` chỉ pass khi:

- Bridge topics đã xuất hiện.
- TF tree đầy đủ.
- Nav2 lifecycle nodes đều active.
- Gateway trả `ready: true`.

## 12. Refactor smoke test

Smoke test hiện tại khởi tạo trực tiếp `CustomNavigator`. Refactor test để nhận backend
từ environment:

```env
NAVIGATION_BACKEND=custom
NAVIGATION_BACKEND=nav2
```

Giữ nguyên 15 scenario hiện có:

1. Clear goal.
2. Static-obstacle detour.
3. Diagonal goal.
4. Goal gần center pillar.
5. Left boundary.
6. Upper boundary.
7. Lower boundary.
8. Narrow center corridor.
9. Far right boundary.
10. Open ground beyond pillar field.
11. Model pose khác odometry.
12. Blocked-start escape.

Mỗi scenario phải kiểm tra:

- Action kết thúc `SUCCEEDED` trước timeout.
- Khoảng cách cuối nhỏ hơn hoặc bằng goal tolerance.
- Velocity command trở về zero.
- Không có collision event.
- Sensor và TF không stale.
- Không còn goal/action sau reset.

Mỗi case được cô lập bằng các bước:

1. Cancel goal cũ và clear costmaps.
2. Đặt model Gazebo explicit về spawn pose; `model_only` reset không được coi là
   đủ vì Gazebo giữ pose đã set bằng `set_pose`.
3. Re-anchor `map -> odom` và chờ sensor/TF ổn định.
4. Chỉ sau đó mới plan/start goal.

Case 11 và 12 là gate riêng:

- Case 11 xác nhận localization/TF không dùng sai odometry frame.
- Case 12 xác nhận recovery có thể thoát inflation hoặc blocked start.

## 13. Báo cáo so sánh

Mỗi smoke run ghi một JSON report:

```json
{
  "backend": "nav2",
  "scenario": "static_obstacle_detour",
  "result": "SUCCEEDED",
  "duration_seconds": 0.0,
  "path_length_m": 0.0,
  "minimum_clearance_m": 0.0,
  "final_error_m": 0.0,
  "recovery_count": 0
}
```

So sánh với baseline custom theo:

- Success rate.
- Navigation duration.
- Path length.
- Minimum clearance.
- Recovery count.
- Stop latency khi cancel/emergency.

Nav2 không cần thắng mọi metric, nhưng không được giảm safety hoặc success rate.

## 14. Các phase triển khai

### Phase 0 - Compatibility spike

- Chọn ROS 2/Nav2/Gazebo bridge combination.
- Bridge `/scan`, `/odom`, `/cmd_vel`, `/clock`.
- Xác nhận reset không làm bridge chết.

Deliverable: một launch/compose command làm robot chạy bằng ROS `/cmd_vel`.

### Phase 1 - Adapter abstraction

- Thêm `NavigationAdapter` protocol.
- Bọc `CustomNavigator` bằng `CustomNavigationAdapter`.
- Chuyển `server.py` sang protocol.
- Chạy unit suite và custom smoke suite.

Deliverable: backend custom vẫn pass 12/12, API không đổi.

### Phase 2 - Nav2 bringup

- Thêm ROS map, TF publishers và `nav2_sim.yaml`.
- Start planner, controller, behavior, BT navigator và lifecycle manager.
- Xác nhận lifecycle active.

Deliverable: gửi `NavigateToPose` bằng ROS CLI và robot tới một clear goal.

### Phase 3 - Nav2 gateway và adapter

- Thêm gateway HTTP/JSON.
- Implement `Nav2NavigationAdapter`.
- Giữ plan/start ở adapter để tương thích; gateway chỉ cần navigate/cancel/reset/speed/status.
- Chạy Web UI không thay frontend.

Deliverable: chọn goal và điều khiển Nav2 từ UI hiện tại.

### Phase 4 - Smoke test parity

- Parameterize smoke suite theo backend.
- Chạy 15 case custom.
- Chạy 15 case Nav2.
- Tuning từng failure dựa trên log và report.

Deliverable: báo cáo A/B cho đủ 15 scenario.

### Phase 5 - Stability gate

- Chạy Nav2 smoke suite ba lần liên tiếp.
- Không flaky failure, collision hoặc stale sensor/TF.
- Verify normal unit suite và Docker health checks.

Deliverable: 45/45 scenario runs pass.

### Phase 6 - Chuyển mặc định

- Đổi default sang `NAVIGATION_BACKEND=nav2` nếu stability gate pass.
- Giữ `custom` làm fallback trong ít nhất một release.
- Chỉ xóa custom backend bằng một thay đổi riêng sau khi được duyệt.

## 15. Rủi ro và cách xử lý

### ROS 2 Humble không tương thích Gazebo Jetty

Xử lý bằng container/bridge version phù hợp cho simulator; giữ contract tương
thích `robot_nav` thay vì ép chung Python environment.

### TF sai hoặc thiếu

Thêm test kiểm tra toàn bộ chain `map -> laser_frame` tại timestamp của scan.
Không tuning planner trước khi TF ổn định.

### Nav2 không thoát blocked start

Tuning recovery và inflation; nếu cần thêm behavior/plugin riêng. Không nới
collision constraint chỉ để test pass.

### Reset để lại costmap cũ

Reset adapter phải cancel action, clear cả hai costmap và chờ sensor sequence mới
trước khi trả `IDLE`.

### Hai backend trả status khác nhau

Đặt mapping state và schema test ở adapter contract. Web UI không nhận raw Nav2
status.

### Docker image quá lớn

Giữ ROS 2/Nav2 trong image riêng và chỉ bật qua profile. Image `amr-app` hiện tại
không chứa Nav2.

## 16. Definition of Done

Nav2 adapter được xem là hoàn tất khi:

- API và Web UI không cần thay đổi contract.
- Backend được chọn bằng environment.
- Custom backend vẫn pass 12/12 smoke cases.
- Nav2 backend pass 12/12 smoke cases trong ba lần liên tiếp.
- Unit tests, Compose config và health checks pass.
- Cancel, reset và speed limit hoạt động.
- Không collision, stale TF hoặc stale sensor.
- Có báo cáo A/B giữa custom và Nav2.
- README có lệnh chạy cả hai backend.
- Navigation custom chưa bị xóa và có thể dùng để rollback.
