# CartPole manual control và RL

## Mục tiêu

Chia CartPole thành hai version, nhưng giữ nguyên model và dynamics để Version 2 có thể tái sử dụng trực tiếp nền tảng của Version 1.

- Version 1: điều khiển CartPole bằng keyboard trên web.
- Version 2: dùng `cart_pole_env.py` để train và chạy policy RL.
- Cả hai version đều visualize qua web viewer hiện tại.

## Quyết định kiến trúc

Giữ CartPole là một model riêng, không chuyển sang `DiffDrive` và không biến nó thành AMR trong giai đoạn này.

Giữ ổn định các contract mà RL example đang sử dụng:

- Model name: `vehicle_green`
- Link: `chassis`, `pole`, `pole_mass`
- Joint: `pole_joint`
- Pole quay quanh trục Y
- Lực điều khiển tác động lên `chassis` theo trục X
- Action 0: lực `-2000 N`
- Action 1: lực `+2000 N`
- Physics timestep: `0.001 s`

Các bánh hình cầu hiện tại được giữ nguyên để không thay đổi dynamics của Gazebo RL example. Chưa thêm `DiffDrive` vào CartPole.

## Luồng chung

```text
Version 1:
Keyboard trên Browser
        │
        ▼
Control Bridge
        │  gz.msgs.EntityWrench
        ▼
ApplyLinkWrench
        │
        ▼
vehicle_green::chassis

Version 2:
RL action
        │
        ▼
cart_pole_env.py
        │  add_world_force()
        ▼
vehicle_green::chassis

Gazebo ── WebSocket 9002 ──► Browser viewer
```

## Version 1: Manual control trên web

### Trạng thái triển khai

Version 1 đã có các thành phần nền tảng:

- `worlds/main.sdf` load system plugin `ApplyLinkWrench`.
- `control_bridge.py` serve web viewer và publish lực qua `gz.transport`.
- `index.html` có keyboard/button controls.
- `pyproject.toml` khai báo protobuf runtime cho Gazebo Python bindings.

### Gazebo

Thêm system plugin có sẵn vào `worlds/main.sdf`, bên trong `<world name="cart_pole">`:

```xml
<plugin
  filename="gz-sim-apply-link-wrench-system"
  name="gz::sim::systems::ApplyLinkWrench">
  <topic>/model/vehicle_green/force_cmd</topic>
</plugin>
```

`/model/vehicle_green/force_cmd` là topic được `ApplyLinkWrench` subscribe sau khi plugin được load. Đây không phải custom Gazebo plugin.

Message type của topic là `gz.msgs.EntityWrench`, không phải `gz.msgs.Double`:

```text
entity: {
  name: "vehicle_green::chassis",
  type: LINK
}
wrench: {
  force: { x: 2000, y: 0, z: 0 }
}
```

Topic instantaneous chỉ áp dụng lực trong một physics step. Vì vậy Control Bridge phải publish lặp lại khoảng 30–50 Hz.

### Browser

Thêm keyboard/button control tối giản:

- Arrow Left: publish lực `-2000 N`
- Arrow Right: publish lực `+2000 N`
- Space hoặc keyup: publish lực `0`
- Có nút Emergency Stop
- Nút `Reset`: reset world về trạng thái ban đầu.
- Khi giữ phím, gửi heartbeat mỗi `100 ms` để duy trì lực.

WebSocket viewer hiện tại chủ yếu subscribe pose từ Gazebo. Browser cần gửi command qua một Control Bridge nhỏ, vì không nên phụ thuộc vào việc browser tự nói chuyện trực tiếp với Gazebo Transport.

### Control Bridge

Bridge nhận command từ browser và publish `EntityWrench` tới `/model/vehicle_green/force_cmd`.

Chạy Version 1:

```bash
gz sim -v 4 -r -s worlds/main.sdf

uv venv --system-site-packages .venv
uv sync
uv run control_bridge.py --host 0.0.0.0 --port 8009
```

Mở `http://<server-ip>:8009/`. Có thể kiểm tra bridge bằng:

```bash
curl http://localhost:8009/api/health

curl -X POST http://localhost:8009/api/cartpole/reset
```

Yêu cầu tối thiểu:

- Giữ command hiện tại trong memory.
- Publish command theo chu kỳ 30–50 Hz.
- Tự publish lực `0` nếu browser mất kết nối hoặc không có heartbeat quá khoảng 200 ms.
- Clamp lực trong giới hạn an toàn.

## Version 2: CartPole RL

Tái sử dụng nguyên model và dynamics của Version 1. Không thêm `DiffDrive` và không cần `ForceController` trong SDF.

`cart_pole_env.py` hiện tại tự tạo `TestFixture`, tìm model/link và gọi:

```python
if action == 1:
    self.chassis.add_world_force(ecm, Vector3d(2000, 0, 0))
elif action == 0:
    self.chassis.add_world_force(ecm, Vector3d(-2000, 0, 0))
```

Script hiện load SDF example riêng tại:

```python
self.fixture = TestFixture(os.path.join(file_path, 'cart_pole.sdf'))
```

Để RL sử dụng world/model trong project, cần đổi đường dẫn này sang world của project, hoặc tạo một RL world riêng dùng chung `models/cartpole/model.sdf`.

Khuyến nghị tạo:

```text
worlds/main.sdf          # Web viewer, có WebSocketServer
worlds/cart_pole_rl.sdf  # RL world, dùng cùng model CartPole
models/cartpole/model.sdf
```

Nếu muốn visualize trực tiếp quá trình RL trên web, `cart_pole_rl.sdf` cũng phải có `SceneBroadcaster` và `WebsocketServer` port `9002`. Chỉ chạy một Gazebo server tại một thời điểm để tránh conflict port.

Trong web mode, không cần gọi `run_gui()` của example; browser là giao diện visualize chính.

## Thứ tự triển khai

1. Freeze `models/cartpole/model.sdf` theo contract ở trên.
2. Thêm `ApplyLinkWrench` vào `worlds/main.sdf`.
3. Test force bằng `gz topic` trước khi làm UI.
4. Thêm Control Bridge.
5. Thêm keyboard/button control trên browser.
6. Xác nhận CartPole manual di chuyển và dừng an toàn.
7. Tách hoặc cập nhật RL world để dùng chung `models/cartpole/model.sdf`.
8. Cập nhật `cart_pole_env.py` dùng world trong project.
9. Train PPO với action space hiện tại là `Discrete(2)`.
10. Hiển thị evaluation policy trên web viewer.

## Ranh giới với AMR tương lai

CartPole và AMR nên là hai model riêng:

```text
models/cartpole/  # manual force + CartPole RL
models/amr/       # DiffDrive + /model/<name>/cmd_vel
```

Nếu sau này AMR có thêm pole, có thể compose hai controller:

```text
DiffDrive       -> điều khiển thân AMR
Pole controller -> điều khiển joint của pole
```

Không dùng `force_cmd` làm API di chuyển chính của AMR. `ApplyLinkWrench` chỉ phù hợp cho CartPole benchmark hoặc external disturbance như va chạm, gió và tác động kiểm thử.
