# AMR Navigation Simulator — Executive Overview

## 1. Executive Decision

> **Chọn Gazebo Jetty + Python thuần + Gazebo Transport + Web Visualization.**

Không dùng ROS 2/Nav2 ở giai đoạn đầu.

**Lý do:** mục tiêu hiện tại là hiểu và tự xây dựng bản chất của AMR navigation:

```text
LiDAR + Odometry
       ↓
 Localization
       ↓
    Map
       ↓
 Global Planning
       ↓
 Local Planning
       ↓
 Robot Control
```

Gazebo cho phép kiểm soát trực tiếp simulator, sensor, physics và robot command; đồng thời có thể chạy **headless trên VPS** và xem simulation từ **browser**.

---

# 2. Problem Being Solved

Ta cần một môi trường có thể:

- Chạy trên VPS qua SSH
- Không yêu cầu GUI trên VPS
- Có robot AMR
- Có 2D LiDAR
- Có wheel odometry
- Có IMU
- Có physics
- Cho phép điều khiển robot
- Sinh sensor data theo thời gian thực
- Python có thể đọc sensor và gửi command
- Browser có thể xem robot/simulation

### Vì sao không chỉ dùng Dataset?

Dataset là dữ liệu **đã được ghi lại**:

```text
LiDAR ───────┐
Odometry ────┼──► Recorded Data
IMU ─────────┘
```

Bạn không thể tùy ý nói:

> "Cho robot chạy thêm 1 mét rồi rẽ trái."

Simulator thì có feedback loop:

```text
Python
  ↓
velocity command
  ↓
Robot
  ↓
Physics
  ↓
Robot movement
  ↓
┌──────────────┬──────────────┐
LiDAR       Odometry          IMU
  └──────────────┬──────────────┘
                 ↓
              Python
```

Đây chính là thứ cần để học navigation.

---

# 3. Selected Simulator

## Gazebo Jetty

### Why?

| Requirement | Gazebo Jetty |
|---|---|
| AMR simulation | ✅ |
| Physics | ✅ |
| 2D LiDAR | ✅ |
| Odometry | ✅ |
| IMU | ✅ |
| Differential drive | ✅ |
| Python | ✅ |
| Không ROS 2 | ✅ |
| Headless VPS | ✅ |
| Browser visualization | ✅ |
| Custom world | ✅ |
| Custom robot | ✅ |
| Long-term support | ✅ |

Jetty là **LTS**, với thời gian hỗ trợ dự kiến đến **2031**.

---

# 4. Final Architecture

```text
                         LAPTOP
                    ┌─────────────┐
                    │   Browser   │
                    │             │
                    │ Gazebo Web  │
                    │ Visualization
                    └──────┬──────┘
                           │
                       WebSocket
                           │
═══════════════════════════╪════════════════════
                           │
                           ▼
                     VPS / SERVER
              ┌─────────────────────────┐
              │      Gazebo Jetty       │
              │       headless          │
              │                         │
              │      ┌──────────┐       │
              │      │   AMR    │       │
              │      └────┬─────┘       │
              │           │             │
              │     ┌─────┼──────┐      │
              │     ▼     ▼      ▼      │
              │   LiDAR  Odom   IMU     │
              │                         │
              └───────────┬─────────────┘
                          │
                   Gazebo Transport
                          │
                          ▼
                 ┌──────────────────┐
                 │      Python      │
                 │                  │
                 │ Sensor handling  │
                 │ Localization     │
                 │ Mapping          │
                 │ A*               │
                 │ DWA              │
                 │ Controller       │
                 └────────┬─────────┘
                          │
                    velocity command
                          │
                          ▼
                       AMR Robot
```

---

# 5. Technology Stack

```text
OS
└── Ubuntu 24.04

Simulator
└── Gazebo Jetty

Robot
└── Differential-drive AMR

Sensors
├── 2D LiDAR
├── Wheel encoder / Odometry
└── IMU

Communication
└── Gazebo Transport

Programming
└── Python

Navigation
├── Localization
├── Occupancy Grid
├── A* / Dijkstra
├── DWA
└── Differential-drive Controller

Visualization
└── Gazebo Web Visualization

Deployment
└── VPS + SSH
```

---

# 6. Data Flow

## Sensor → Navigation

### LiDAR

LiDAR cung cấp:

```text
angle_min
angle_max
angle_increment
ranges[]
```

Ví dụ:

```text
ranges =
[
  2.31,
  2.30,
  2.28,
  ...
  0.81,
  0.82,
  ...
]
```

→ Biết **obstacle ở đâu**.

---

### Odometry

Odometry cung cấp thông tin robot đã di chuyển:

```text
x
y
theta

linear_velocity
angular_velocity
```

→ Biết **robot nghĩ mình đang ở đâu / đã di chuyển thế nào**.

Odometry có drift theo thời gian.

---

### IMU

Cung cấp:

```text
angular velocity
linear acceleration
orientation
```

→ Hỗ trợ estimate chuyển động/orientation.

---

# 7. Navigation Pipeline

```text
               Sensors
                  │
        ┌─────────┼─────────┐
        ▼         ▼         ▼
      LiDAR      Odom      IMU
        │         │         │
        └─────────┼─────────┘
                  ▼
             Localization
                  │
                  ▼
             Robot Pose
              (x,y,θ)
                  │
       ┌──────────┴──────────┐
       ▼                     ▼
     Map                  LiDAR
       │                     │
       └──────────┬──────────┘
                  ▼
             Costmap /
          Obstacle Map
                  │
                  ▼
           Global Planner
              A* / Dijkstra
                  │
                  ▼
                Path
                  │
                  ▼
            Local Planner
                DWA
                  │
                  ▼
              Controller
                  │
                  ▼
        v = linear velocity
        ω = angular velocity
                  │
                  ▼
               Wheels
                  │
                  ▼
                Robot
                  │
                  └──────────► Sensors
```

Đây là **feedback loop**, không phải pipeline một chiều.

---

# 8. Python sẽ làm gì?

Mục tiêu là Python của mình đóng vai trò như một **mini Navigation Stack**.

```python
while True:

    lidar = get_lidar()

    odom = get_odometry()

    imu = get_imu()

    pose = model_pose()

    path = astar(
        map,
        pose,
        goal
    )

    trajectory = dwa(
        pose,
        path,
        lidar
    )

    v, omega = controller(
        trajectory
    )

    send_velocity(v, omega)
```

Không cần:

```text
❌ ROS 2
❌ Nav2
```

ở giai đoạn học nền tảng.

---

# 9. Simulator vs Dataset

| | Simulator | Dataset |
|---|---|---|
| Điều khiển robot | ✅ | ❌ |
| Sinh sensor theo movement | ✅ | ❌ |
| Thử obstacle mới | ✅ | ❌ |
| Thử planner | ✅ | ⚠️ |
| Replay real-world data | ❌ | ✅ |
| Benchmark | ⚠️ | ✅ |
| Học navigation | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| Debug | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| Realism | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

### Chiến lược

```text
Simulator
   ↓
Hiểu navigation
   ↓
Tự implement
   ↓
Test algorithm
   ↓
Dataset
   ↓
Benchmark / validate
   ↓
Real AMR
```

---

# 10. Vì sao không chọn SimLabs?

SimLabs phù hợp nếu mục tiêu chính là:

> **"Tôi muốn chạy simulator trên cloud và mở browser càng nhanh càng tốt."**

Nó là một platform có thể host/stream nhiều simulator.

Nhưng mục tiêu của project này là:

> **"Tôi muốn hiểu và kiểm soát navigation stack."**

Do đó thêm một abstraction layer như SimLabs không mang lại lợi ích lớn bằng việc trực tiếp làm việc với Gazebo.

### Decision

```text
SimLabs
   │
   └── Dễ triển khai / cloud-first
             ↓
         Convenience

Gazebo Jetty
   │
   ├── Physics
   ├── Sensors
   ├── Robot
   ├── Transport
   ├── World
   └── Python
             ↓
       Control & Learning
```

**→ Gazebo Jetty thắng cho use case này.**

---

# 11. Development Roadmap

## Phase 1 — Robot & Sensors

```text
Gazebo
  ↓
AMR
  ↓
LiDAR
  ↓
Odometry
  ↓
IMU
```

Mục tiêu:

> Hiểu raw sensor data.

---

## Phase 2 — Robot Control

```text
Python
  ↓
v, ω
  ↓
Robot
```

Mục tiêu:

> Gửi velocity và quan sát robot di chuyển.

---

## Phase 3 — Odometry

```text
Wheel movement
      ↓
Odometry
      ↓
(x,y,θ)
```

Mục tiêu:

> Hiểu robot estimate motion như thế nào.

---

## Phase 4 — LiDAR

```text
LiDAR
 ↓
ranges[]
 ↓
(x,y) points
 ↓
Obstacle representation
```

---

## Phase 5 — Localization

```text
LiDAR
 +
Odometry
 +
Map
 ↓
Pose estimation
```

---

## Phase 6 — Global Planning

```text
Map
 +
Start
 +
Goal
 ↓
A*
 ↓
Global Path
```

---

## Phase 7 — Local Planning

```text
Global Path
 +
Current Pose
 +
LiDAR
 ↓
DWA
 ↓
Safe trajectory
```

---

## Phase 8 — Closed-loop Navigation

```text
Goal
 ↓
Localization
 ↓
A*
 ↓
DWA
 ↓
Controller
 ↓
Robot
 ↓
LiDAR + Odometry
 ↓
Localization
 ↓
...
```

Đến đây bạn đã có một **miniature AMR navigation stack**.

---

# 12. Final Recommendation

### Chốt stack:

```text
┌──────────────────────────────┐
│          Browser             │
│    Gazebo Web Visualization  │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│         VPS                  │
│                              │
│     Gazebo Jetty             │
│       headless               │
│          │                   │
│          ▼                   │
│        AMR                    │
│     /   |    \               │
│ LiDAR  Odom  IMU             │
│          │                   │
│          ▼                   │
│       Python                 │
│                              │
│ Localization                │
│ Mapping                     │
│ A*                          │
│ DWA                         │
│ Controller                  │
└──────────────────────────────┘
```

### **Final decision**

> **Gazebo Jetty + Ubuntu 24.04 + headless + Gazebo Transport + Python + Web Visualization + differential-drive AMR + 2D LiDAR + wheel odometry + IMU.**

Đây là setup cân bằng tốt nhất giữa **học bản chất robotics, khả năng tự code navigation, chạy được trên VPS qua SSH và vẫn quan sát được simulation bằng browser**.
