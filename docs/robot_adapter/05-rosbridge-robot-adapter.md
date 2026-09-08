# Phase 5 — Rosbridge robot adapter

**Sẵn sàng:** Đủ để viết; chưa đủ nghiệm thu autonomous.

## Mục tiêu

Cho navigation core điều khiển robot thật qua stack ROS hiện tại mà không import ROS.

## Deliverables

- Pose liên tục từ TF `map -> odom -> base_footprint`.
- Quality từ `/amcl_pose`; ánh xạ sang `OK/DEGRADED/LOST`.
- Velocity từ `/odom_raw`; obstacles từ `/scan` trong `base_footprint`.
- Freshness, snapshot validation, `scan_id` và `pose_epoch`.
- `drive()` publish `/cmd_vel_nav`; `stop()` luôn phát zero.
- Kiểm tra cách chạy `roslibpy` với Python 3.14; dùng sidecar nếu không tương thích.

## File thêm

- `src/adapters/rosbridge_robot.py`
- `src/tests/test_rosbridge_robot_adapter.py`

## File sửa

- `src/server.py`
- `src/config.py`
- `pyproject.toml`
- `/home/pi/quangnv/robot_transport/micro_ros_car_interface.py`
- `/home/pi/quangnv/robot_transport/test_interface.py`
- `/home/pi/quangnv/robot_transport/bootstrap.sh`

## Nghiệm thu

- `RobotIO.read()` trả snapshot hợp lệ khi robot đứng yên và di chuyển.
- Scan/odom/TF stale hoặc localization mất đều làm robot dừng.
- Integration test bắt buộc có map pose và quality.
- Không phát autonomous command trực tiếp vào `/cmd_vel`.

