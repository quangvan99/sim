# Phase 7 — Direct adapter không ROS

**Sẵn sàng:** Chưa đủ; cần chốt firmware/protocol, đường LiDAR và localization độc lập.

## Mục tiêu

Thay Rosbridge adapter bằng transport trực tiếp mà không sửa navigation core.

## Deliverables

- Firmware ESP32-S3 đọc motor, encoder, IMU, pin và LiDAR nếu LiDAR nối qua board.
- Protocol UART/UDP có version, sequence, timestamp, CRC và watchdog.
- Standalone localization: odometry/gyro prediction cộng LiDAR scan matching.
- Safety độc lập tần số cao, nằm giữa navigation command và actuator.
- `DirectRobotIO` dùng cùng `RobotState`/`RobotProfile` của Phase 1.

## File thêm

- `src/adapters/direct_robot.py`
- `src/navigation/localization.py`
- `src/safety/motion_guard.py`
- `src/tests/test_direct_robot_adapter.py`
- `src/tests/test_localization.py`
- `firmware/yahboom_direct/`

## File sửa

- `src/server.py`
- `src/config.py`
- `pyproject.toml`
- `bootstrap.sh` hoặc service khởi động production tương ứng.

## Nghiệm thu

- Không còn tiến trình ROS, rosbridge hoặc micro-ROS Agent.
- Localization đạt ngưỡng sai số đã chốt trong Gazebo trước khi chạy thật.
- Đổi từ `RosbridgeRobotIO` sang `DirectRobotIO` không sửa `src/navigation`.
- Robot chạy lại tuyến Phase 6 với đầy đủ watchdog và safety.
