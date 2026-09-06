# AMR Simulation Integration Plan

## Trạng thái navigation hiện tại

Milestone mô phỏng nền bên dưới đã được mở rộng bằng A* + DWA-lite thuần
Python trên branch `feat/amr_astar`:

- Static occupancy map có version, obstacle inflation và clearance cost.
- Map phủ ground `20 x 20 m`; chỉ 9 trụ là obstacle navigation và 38.128/38.128
  ô hợp lệ liên thông từ spawn.
- A* 8 hướng deterministic, không corner-cut, có line-of-sight smoothing.
- DWA-lite dùng Gazebo model pose, vận tốc odometry và LiDAR, phát
  `/amr/cmd_vel` an toàn.
- Navigation và UI lấy vị trí tuyệt đối từ dynamic model pose; wheel odometry
  chỉ cung cấp vận tốc.
- Temporary obstacle layer và A* re-plan khi runtime path bị chặn.
- Navigation state machine, command ownership, fail-safe và HTTP API.
- Browser tối giản: `Select Goal`, `Go`, `Stop`, `Reset` và hai nút tam giác
  trái/phải để chỉnh `Speed`.
- Goal gần trụ `(0.19, -0.35)`, regression `(-0.44, 0.43)` và goal ngoài vùng
  lục giác cũ `(1.70, 3.99)` đều đã tới nơi trong Gazebo.
- Browser validate B khi click, marker B bán kính `4.5 cm`; robot chỉ báo tới
  nơi khi tâm cách B không quá `4 cm`.
- Một UI duy nhất luôn có cả WASD/nút manual và chọn B; manual input hủy
  autonomous mission đang chạy rồi chiếm `/amr/cmd_vel` ngay.

Không có ROS 2, RL, training hoặc checkpoint trong implementation này. Thiết
kế, wireframe, deliverables và test contract nằm tại
`docs/amr_astar_dwa.md`.

## 1. Mục tiêu

Tích hợp một mô phỏng AMR differential-drive hoàn chỉnh vào
`src/amr/sim`.

Milestone đầu tiên phải chạy được bằng Gazebo Jetty và Gazebo Transport,
không phụ thuộc ROS 2, đồng thời cung cấp đủ:

- Robot model và collision/inertial hợp lệ.
- Differential-drive control.
- 2D LiDAR.
- IMU.
- Wheel odometry.
- Joint states.
- World có ground, ánh sáng và vật cản.
- Chạy headless trên server.
- Có thể quan sát qua browser.
- Python có thể điều khiển robot và đọc sensor bằng Gazebo Transport.

Full navigation như mapping, localization, A*, DWA và autonomous mission
không thuộc milestone này. Chúng sẽ dùng simulation contract được tạo ở đây.

## 2. Hiện trạng và nguyên tắc tích hợp

Repo hiện tại là Python project dùng `uv` và `src` layout, không phải ROS
workspace. CartPole đang dùng hai kiểu tích hợp Gazebo:

- Chạy `gz sim` độc lập cho interactive AMR.
- Dùng `gz.sim.TestFixture` cho deterministic RL mode.
- Dùng `gz.transport.Node` cho command, service và telemetry.
- Dùng Gazebo WebSocket system để browser nhận scene và dynamic pose.

AMR sẽ tuân theo kiến trúc này và không đưa ROS 2 vào project.

### Quy tắc độc lập với CartPole

`src/amr` phải là một package self-contained. Nếu cần tái sử dụng file, asset
hoặc cách triển khai đang có trong `src/cartpole`, file đó phải được **copy**
sang vị trí tương ứng trong `src/amr` rồi mới chỉnh sửa cho AMR.

Không được tạo dependency runtime từ AMR sang CartPole, bao gồm:

- Không dùng Python import như `from cartpole...` hoặc `import cartpole...`.
- Không đọc file bằng đường dẫn trỏ tới `src/cartpole`.
- Không dùng symlink từ `src/amr` sang `src/cartpole`.
- Không để HTML, JavaScript hoặc CSS của AMR tham chiếu asset trong
  `cartpole/sim/visualize`.
- Không dùng chung config object, server state hoặc simulation resource path
  của CartPole.

Các file được copy phải trở thành tài sản do package `amr` sở hữu. Sau đó hai
package có thể thay đổi độc lập mà không làm hỏng package còn lại. Việc tách
thành shared package chỉ được thực hiện trong một refactor riêng, không nằm
trong milestone này.

## 3. Quy ước simulation contract

Các tên dưới đây phải cố định giữa SDF, Python, test và browser:

| Thành phần | Giá trị đề xuất |
|---|---|
| World name | `amr_world` |
| Robot instance name | `amr` |
| Command topic | `/amr/cmd_vel` |
| Odometry topic | `/amr/odom` |
| LiDAR topic | `/amr/scan` |
| IMU topic | `/amr/imu` |
| Joint state topic | `/amr/joint_states` |
| Dynamic pose topic | `/world/amr_world/dynamic_pose/info` |
| World control service | `/world/amr_world/control` |
| HTTP port | `8009` |
| Gazebo WebSocket port | `9002` |

AMR dùng chung HTTP `8009` và WebSocket `9002` với CartPole. Contract này giả
định mỗi thời điểm chỉ chạy một simulator, giúp giữ cách chạy thống nhất.

Command contract:

- Message: `gz.msgs.Twist`.
- `linear.x`: vận tốc tiến/lùi, đơn vị m/s.
- `angular.z`: vận tốc quay, đơn vị rad/s.
- Các trục còn lại bằng 0.
- Có dead-man timeout; nếu không nhận command mới thì publish vận tốc 0.

Sensor contract:

- LiDAR: `gz.msgs.LaserScan`, 360 samples, có timestamp và frame `base_scan`.
- IMU: `gz.msgs.IMU`, có angular velocity và linear acceleration.
- Odometry: `gz.msgs.Odometry`, có pose, linear velocity và angular velocity.
- Model pose: `gz.msgs.Pose_V` trên dynamic pose topic, dùng làm world pose
  tuyệt đối của robot trong simulation.
- Joint states: dùng để kiểm tra chuyển động của hai wheel joints.

## 4. Cấu trúc đích

```text
src/amr/
├── __init__.py
├── config.py
├── transport.py
├── server.py
└── sim/
    ├── models/
    │   ├── robot/
    │   │   ├── model.config
    │   │   └── model.sdf
    │   ├── common/
    │   │   ├── model.config
    │   │   └── meshes/
    │   │       ├── bases/base.stl
    │   │       ├── sensors/lidar.stl
    │   │       └── wheels/
    │   │           ├── left_wheel.stl
    │   │           └── right_wheel.stl
    │   └── world/
    │       ├── model.config
    │       ├── model.sdf
    │       └── meshes/
    │           ├── hexagon.dae
    │           └── wall.dae
    ├── worlds/
    │   └── main.sdf
    └── visualize/
        ├── index.html
        ├── app.js
        ├── styles.css
        └── vendor/
            ├── eventemitter2.min.js
            ├── gz.js
            └── gz3d.js
```

Không import các phần ROS-only của upstream:

- `package.xml`
- `CMakeLists.txt`
- `launch/`
- `params/*bridge.yaml`
- `urdf/`
- `src/*.cpp`
- `include/`

## 5. Deliverables

### D1 — Self-contained AMR assets

Mục tiêu: đưa tập asset tối thiểu cho AMR vào repo chính.

File mới:

- `src/amr/sim/models/robot/model.sdf`
- `src/amr/sim/models/robot/model.config`
- `src/amr/sim/models/common/model.config`
- `src/amr/sim/models/common/meshes/bases/base.stl`
- `src/amr/sim/models/common/meshes/sensors/lidar.stl`
- `src/amr/sim/models/common/meshes/wheels/left_wheel.stl`
- `src/amr/sim/models/common/meshes/wheels/right_wheel.stl`
- `src/amr/sim/models/world/model.sdf`
- `src/amr/sim/models/world/model.config`
- `src/amr/sim/models/world/meshes/hexagon.dae`
- `src/amr/sim/models/world/meshes/wall.dae`

Thay đổi cần thực hiện trong `robot/model.sdf`:

- Giữ nguyên geometry, collision, inertia và wheel joints từ upstream.
- Dùng model URI `robot`.
- Đổi các topic tương đối thành topic tuyệt đối theo simulation contract.
- Giữ GPU LiDAR, IMU, DiffDrive và JointStatePublisher plugins.
- Không thêm ROS plugin hoặc ROS message type.

Tiêu chí nghiệm thu:

- Mọi `model://...` URI được resolve chỉ bằng `src/amr/sim`.
- Không cần tải model hoặc mesh từ Internet.

### D2 — Standalone Gazebo world

Mục tiêu: tạo world có thể chạy trực tiếp bằng `gz sim`, không cần ROS launch
để spawn robot.

File mới:

- `src/amr/sim/worlds/main.sdf`

Nội dung thiết kế của world:

- SDF world name là `amr_world`.
- Physics step ban đầu là `0.001` giây.
- Khai báo `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` và `Imu`.
- Khai báo `WebsocketServer` trên port `9002`.
- Ground plane và directional light được khai báo local.
- Include `model://world` làm môi trường tĩnh.
- Include `model://robot`, đặt instance name là `amr`.
- Đặt robot ở pose không giao với tường hoặc obstacle.
- Không dùng Ground Plane và Sun từ Gazebo Fuel.

Tiêu chí nghiệm thu:

- World chạy headless không có lỗi SDF, plugin hoặc missing resource.
- Robot spawn đúng pose và không rơi xuyên ground.
- Robot đứng yên khi chưa có command.
- Gazebo process dừng sạch khi nhận SIGINT/SIGTERM.

### D3 — Gazebo Transport control và sensor access

Mục tiêu: Python điều khiển AMR và đọc được sensor mà không cần ROS bridge.

File mới:

- `src/amr/__init__.py`
- `src/amr/config.py`
- `src/amr/transport.py`
- `src/amr/server.py`

Trách nhiệm từng file:

- `config.py`: đường dẫn asset, world/model name, topic, port, velocity limit,
  publish rate, sensor timeout và dead-man timeout.
- `transport.py`: sở hữu một `gz.transport.Node`, publisher `Twist`, subscriber
  LiDAR/IMU/Odometry/JointState, thread-safe sensor snapshot và world reset.
- `server.py`: FastAPI health endpoint, manual velocity và speed-profile
  endpoints, reset endpoint, telemetry endpoint và static mount cho AMR viewer.

Nguyên tắc triển khai:

- Không import `cartpole.config.CONFIG`.
- Không thay đổi topic hoặc behavior của CartPole.
- Dùng `GZ_PARTITION` riêng để tránh đọc nhầm simulation khác.
- Validate kiểu dữ liệu và giới hạn `linear.x`, `angular.z` trước khi publish.
- Publish zero velocity khi command quá hạn hoặc server shutdown.
- Callback Gazebo chỉ cập nhật snapshot; không chạy xử lý nặng trong callback.
- Health endpoint phải báo sensor nào chưa xuất hiện hoặc đã stale.
- Reset dùng một tiến trình `gz service` ngắn hạn để gọi full world reset. Đây
  là workaround cho Gazebo Transport Python trên Jetty/macOS: blocking service
  request có thể timeout sau khi cùng process đã subscribe sensor tốc độ cao.
  Sensor và velocity command vẫn giao tiếp trực tiếp bằng `gz.transport.Node`.

Tiêu chí nghiệm thu:

- Publish `linear.x > 0` làm robot tiến.
- Publish `angular.z != 0` làm robot quay.
- Odometry thay đổi tương ứng với chuyển động.
- LiDAR cung cấp đủ ranges và phát hiện được obstacle.
- IMU có message mới khi robot tăng tốc hoặc quay.
- Reset đưa robot về initial pose và command về zero.

### D4 — Browser visualization và manual control

Mục tiêu: xem world/robot trên browser và lái robot bằng keyboard hoặc button.

File mới:

- `src/amr/sim/visualize/index.html`
- `src/amr/sim/visualize/app.js`
- `src/amr/sim/visualize/styles.css`
- `src/amr/sim/visualize/vendor/eventemitter2.min.js`
- `src/amr/sim/visualize/vendor/gz.js`
- `src/amr/sim/visualize/vendor/gz3d.js`

Viewer AMR không copy nguyên logic CartPole. Nó cần:

- Dùng world `amr_world` và WebSocket port `9002`.
- Gửi `linear.x` và `angular.z`, không gửi force.
- Keyboard mapping: `W/S` tiến lùi, `A/D` quay, `Space` dừng.
- Hai nút tam giác `Speed` đồng bộ tốc độ điều khiển tay và trần tốc độ DWA;
  mặc định `0.50 m/s`, mỗi lần chỉnh `0.10 m/s`, từ `0.30` đến `0.90 m/s`.
- Hiển thị connection state, command hiện tại và sensor freshness.
- Theo dõi `/world/amr_world/dynamic_pose/info`.
- Hỗ trợ mesh visuals hoặc có visual fallback đã được kiểm thử.
- Dừng robot khi pointer/keyboard focus bị mất.

Nếu tái sử dụng các vendor library hoặc phần giao diện từ CartPole, chúng phải
được copy vào `src/amr/sim/visualize` theo danh sách file ở trên. AMR server và
browser không được serve hoặc import trực tiếp từ `src/cartpole`.

Tiêu chí nghiệm thu:

- Browser load được scene và hiển thị robot cùng obstacle.
- Pose robot cập nhật liên tục khi robot di chuyển.
- Nhả phím hoặc mất focus khiến command trở về zero.
- Viewer reconnect được sau khi Gazebo WebSocket restart.

### D5 — Packaging, tests và tài liệu chạy

Mục tiêu: AMR simulation có thể cài đặt, kiểm thử và chạy theo cách nhất quán
với CartPole.

Phần kiểm thử AMR hiện được tinh gọn vào các test của navigation module và
integration test Gazebo; các contract/regression test cũ không còn nằm trong
`src/tests`.

File sửa:

- `pyproject.toml`
- `README.md`

Thay đổi trong `pyproject.toml`:

- Khai báo package data cho `amr` gồm SDF, model config, mesh, texture và
  viewer assets.
- Chỉ thêm dependency mới nếu implementation thực tế yêu cầu.
- Không thay đổi Python/Gazebo dependency contract của CartPole.

Nội dung test:

- Contract test kiểm tra file tồn tại, XML parse được và topic name đồng nhất.
- Contract test xác nhận source và asset của `amr` không chứa import, symlink
  hoặc đường dẫn runtime trỏ tới `src/cartpole`.
- Transport unit test dùng fake publisher/subscriber để kiểm tra validation,
  snapshots và dead-man timeout.
- Simulation smoke test chạy Gazebo có timeout, xác nhận topics xuất hiện,
  gửi một command ngắn và xác nhận odometry thay đổi.
- Toàn bộ test CartPole hiện tại vẫn phải pass.

Thay đổi trong `README.md`:

- Thêm AMR prerequisites.
- Thêm lệnh chạy Gazebo headless.
- Thêm lệnh chạy AMR server.
- Ghi URL browser và ports.
- Thêm lệnh kiểm tra topic và cách dừng process an toàn.
- Tách rõ CartPole và AMR để người dùng không trộn hai mode.

Tiêu chí nghiệm thu:

- `uv sync` vẫn thành công.
- Package cài editable và wheel đều chứa AMR simulation assets.
- Unit tests và smoke test pass.
- CartPole không có regression.
- Một máy chỉ có repo `amr` và Gazebo Jetty vẫn chạy được AMR demo.

## 6. Thứ tự thực hiện

1. Hoàn thành D1: thêm asset self-contained.
2. Hoàn thành D2: chạy world độc lập và kiểm tra physics/plugins.
3. Chốt topic thực tế bằng `gz topic -l` và cập nhật contract nếu cần.
4. Hoàn thành D3: command, sensor subscriptions, reset và health.
5. Hoàn thành D4: browser visualization và manual driving.
6. Hoàn thành D5: packaging, automated tests và README.

Không bắt đầu browser hoặc navigation trước khi D2 và sensor topics đã ổn định.

## 7. Rủi ro và cách xử lý

### SDF/Gazebo version

AMR model dùng SDF 1.8 trong khi CartPole hiện dùng SDF 1.6. Giữ SDF
1.8 trong bước import đầu tiên và smoke-test với Gazebo Jetty đang cài. Chỉ
chuyển version nếu parser hoặc plugin thực tế báo lỗi.

### Topic namespace

Upstream dùng topic tương đối như `cmd_vel`, `odom`, `scan` và `imu`. Khi model
được include với tên khác, resolved topic có thể thay đổi. Việc chuyển sang
topic tuyệt đối trong model vendored giúp Python và test có contract ổn định.

### Mesh trên browser

CartPole viewer hiện chủ yếu xử lý primitive geometry. AMR dùng STL và
DAE, vì vậy browser deliverable phải kiểm thử mesh loading riêng. Gazebo server
simulation không được coi là lỗi chỉ vì browser mesh renderer chưa hoàn tất.

### Remote assets

World upstream dùng Gazebo Fuel cho Ground Plane và Sun. World mới sẽ khai báo
hai thành phần này local để loại bỏ network dependency khi chạy VPS hoặc CI.

### Process/topic collision

CartPole và AMR phải dùng port, world name và `GZ_PARTITION` riêng. Test và
server shutdown phải luôn publish zero velocity và giải phóng port.

## 8. Definition of Done

Milestone AMR simulation được xem là hoàn thành khi:

- `src/amr/sim` chứa đầy đủ model, mesh, world và viewer cần thiết.
- Gazebo Jetty chạy world headless hoàn toàn không cần ROS 2 hay upstream clone.
- Robot nhận velocity command và di chuyển đúng differential-drive.
- Python nhận được LiDAR, IMU, odometry, model pose và joint states.
- Reset hoạt động và dead-man timeout dừng robot an toàn.
- Browser hiển thị được world/robot và manual control hoạt động.
- Test mới pass và test CartPole không regression.
- README có đủ lệnh để một người khác clone repo rồi chạy lại demo.

Sau milestone này mới bắt đầu các phase navigation: occupancy grid, localization,
global planning, local planning và autonomous control.
