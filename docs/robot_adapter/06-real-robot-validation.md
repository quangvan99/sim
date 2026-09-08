# Phase 6 — Kiểm chứng robot thật

**Sẵn sàng:** Chưa thể hoàn thành nếu không chạy test vật lý.

## Mục tiêu

Chứng minh `src/navigation` chạy A đến B an toàn qua Rosbridge adapter.

## Deliverables

- Pipeline `/cmd_vel_nav -> mux -> smoother -> safety -> /cmd_vel`.
- Checklist HIL: đọc sensor khi khóa motor, teleop thấp tốc, rồi autonomous.
- Đo max angular speed, acceleration, braking distance và odometry scale.
- Tune `REAL_ROBOT_PROFILE` và DWA bằng kết quả thực tế.
- Kiểm tra E-stop, manual override, mất WebSocket và sensor stale.

## File thêm

- `docs/amr_deploy_checklist.md`
- `src/tests/test_real_robot_contract.py`

## File sửa

- `src/config.py`
- `src/adapters/rosbridge_robot.py`
- `/home/pi/quangnv/robot_transport/bootstrap.sh`
- `/home/pi/quangnv/robot_transport/test_interface.py`

## Nghiệm thu

- Robot chạy hết tuyến A đến B trong dung sai, không va chạm.
- Mọi lỗi sensor/kết nối đều phát zero command.
- Manual và E-stop luôn giành quyền điều khiển.

