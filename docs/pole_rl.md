# CartPole manual control và RL

## Mục tiêu

Chia CartPole thành hai version, nhưng giữ nguyên model và dynamics để Version 2 có thể tái sử dụng trực tiếp nền tảng của Version 1.

- Version 1: điều khiển CartPole bằng keyboard trên web.
- Version 2: load PPO checkpoint có sẵn, chạy inference và visualize trên web.
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
PPO checkpoint
        │
        ▼
`cartpole.runtime` + `cartpole.env`
        │  add_world_force()
        ▼
vehicle_green::chassis

Gazebo ── WebSocket 9002 ──► Browser viewer
```

## Version 1: Manual control trên web

### Trạng thái triển khai

Version 1 đã có các thành phần nền tảng:

- `src/cartpole/sim/worlds/main.sdf` load system plugin `ApplyLinkWrench`.
- `cartpole.server` serve web viewer và publish lực qua `gz.transport`.
- `src/cartpole/sim/visualize/index.html` có keyboard/button controls.
- `pyproject.toml` khai báo protobuf runtime cho Gazebo Python bindings.

### Gazebo

Thêm system plugin có sẵn vào `src/cartpole/sim/worlds/main.sdf`, bên trong `<world name="cart_pole">`:

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
GZ_SIM_RESOURCE_PATH="$PWD/src/cartpole/sim" \
  gz sim -v 4 -r -s src/cartpole/sim/worlds/main.sdf

uv venv --system-site-packages .venv
uv sync
uv run python -m cartpole.server --host 0.0.0.0 --port 8009
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

### Kết quả cần đạt

Version 2 tạo PPO checkpoint bằng một flow offline riêng, sau đó chạy CartPole bằng inference và hiển thị demo trên WebView.

- Toàn bộ code chuyên dùng để tạo checkpoint nằm trong `src/cartpole/train.py`.
- Training và runtime là hai command độc lập; runtime không import hoặc kích hoạt code training.
- Runtime chỉ gọi `PPO.load(...)` và `model.predict(...)`; không train hoặc fine-tune.
- Browser là giao diện duy nhất.
- Không gọi `run_gui()`, không chạy `gz-sim-gui-client` và không mở Gazebo Sim desktop.
- Dùng chung `src/cartpole/sim/models/cartpole/model.sdf` và tách world theo mục đích: `train.sdf` cho training headless, `main.sdf` cho runtime/WebView.
- Không thay đổi dynamics, action force hoặc flow manual đã hoàn thành của Version 1.

### Contract RL

Checkpoint đầu vào phải tương thích contract sau:

| Thành phần | Giá trị |
|---|---|
| Observation | `[cart_position_x_m, cart_velocity_x_mps, pole_pitch_rad, pole_angular_velocity_y_radps]` |
| Observation dtype | `numpy.float32` |
| Observation space | `Box(low=[-10, -inf, -0.96, -FLOAT_MAX], high=[10, inf, 0.96, FLOAT_MAX], dtype=float32)` |
| Action space | `Discrete(2)` |
| Action `0` | `-2000 N` lên `vehicle_green::chassis` theo trục X |
| Action `1` | `+2000 N` lên `vehicle_green::chassis` theo trục X |
| Physics timestep | `0.001 s` |
| Action repeat | 5 physics steps, tương đương `0.005 s/action` |
| `terminated` | `abs(pole_pitch) > 0.48 rad` hoặc `abs(cart_position) > 4.8 m` |
| `truncated` | Đủ 6000 action steps, tương đương 30 giây simulation |
| Reward | `1` khi còn hợp lệ, `0` tại terminal step |
| Inference | `model.predict(observation, deterministic=True)` |

`FLOAT_MAX` là giá trị lớn nhất của `float32`. Pole bound dùng `±0.96 rad` để sửa mismatch của example gốc: example khai báo `±0.418 rad` nhưng terminate ở `±0.48 rad`.

Mốc 30 giây là time limit của demo. Training giữ nguyên hai giới hạn vật lý. Runtime viewer bỏ qua các giới hạn này để pole có thể ngã tự nhiên, nhưng dừng nếu cart vượt mép ground tại `±48 m`; khi đạt 6000 steps, runtime chuyển sang `SUCCESS` với thông báo simulation đã kết thúc.

Sau `reset_all()`, environment chạy 5 physics steps với `action=None` để khởi tạo observation. Bước khởi tạo không tính vào episode step hoặc mốc 30 giây.

### Tham chiếu Gazebo example

Sử dụng file sau làm behavioral reference:

```text
/Users/admin/quangnv/gz-sim/examples/scripts/reinforcement_learning/simple_cart_pole/cart_pole_env.py
```

Kế thừa cách dùng `TestFixture`, callback pre/post update, cách đọc bốn observation, lực `±2000 N`, 5 physics iterations/action, reward và hai giới hạn vật lý trong training. Viewer có cấu hình riêng để bỏ qua các giới hạn này khi demo.

Không copy hard-coded SDF path, code train ở module scope, `run_gui()`, subprocess GUI, `sleep()`, VecEnv evaluation loop hoặc observation bound `±0.418 rad` bị lệch.

Đã đối chiếu model của project với model trong example: mass, inertia, geometry, friction, joint và pose tương đương. Các khác biệt `-0`/`0` hoặc `0.0`/`0` không làm thay đổi dynamics.

### Plan tạo checkpoint offline

Training chỉ có một nhiệm vụ: tạo `src/checkpoints/ppo_cartpole.zip` tương thích với RL contract. Không có UI và không mở Gazebo desktop.

```text
src/cartpole/train.py
          │
          ├── CartPoleEnv(src/cartpole/sim/worlds/train.sdf)
          │
          ├── PPO("MlpPolicy", env)
          │
          ├── model.learn(total_timesteps=25000)
          │
          └── model.save("src/checkpoints/ppo_cartpole")
                         │
                         ▼
              src/checkpoints/ppo_cartpole.zip
```

Training dùng `src/cartpole/sim/worlds/train.sdf`, một world headless không có `WebsocketServer`, nên không cần bind port `9002`. Process training cũng dùng `GZ_PARTITION` riêng để không xung đột transport namespace với runtime. Runtime dùng `src/cartpole/sim/worlds/main.sdf` để phục vụ WebView. Cả hai world dùng chung model, physics và các thông số dynamics; `TestFixture` đọc observation trực tiếp từ Gazebo nên việc bỏ WebSocket không ảnh hưởng training.

`src/cartpole/train.py` thực hiện đúng các bước:

1. Resolve source root và đường dẫn `src/cartpole/sim/worlds/train.sdf`.
2. Tạo `CartPoleEnv` dùng shared implementation trong `src/cartpole/env.py`.
3. Đọc toàn bộ tham số từ cấu hình tập trung trong `src/cartpole/config.py`.
4. Tạo PPO `MlpPolicy` với seed mặc định `123`, rollout `2048`, batch `64` và `gamma=0.999` để phù hợp control interval `0.005 s`.
5. Dùng linear actor được khởi tạo bằng balancing prior đối xứng; mỗi training episode có thể mirror toàn bộ observation và đổi action `0 ↔ 1`. Cách này loại bỏ left/right bias từ pose khởi tạo cố định mà không đổi reward hoặc contract runtime.
6. Train mặc định `25000` timesteps; cho phép override bằng `--steps`.
7. Đưa action bias về `0`, lưu checkpoint vào `src/checkpoints/ppo_cartpole.zip` hoặc đường dẫn `--output`.
8. Load lại checkpoint vừa lưu và chạy lại các evaluation episode.
9. Chạy ba episode deterministic headless: không disturbance, Push Left và Push Right. Cả ba phải đạt 30 giây.

Training script chỉ tạo checkpoint cuối cùng; không thêm TensorBoard, hyperparameter search, GUI hoặc biểu đồ. Nếu command lỗi, checkpoint không load lại được hoặc một trong ba evaluation episode fail, script trả exit code khác `0` và không coi artifact là hợp lệ.

Mọi thay đổi sau này đối với observation/action contract đều phải cập nhật trong `src/cartpole/config.py` và `src/cartpole/env.py`, không tạo một environment implementation thứ hai bên trong training module.

### Cấu trúc module

CartPole và AMR là hai module cùng cấp; CartPole không nằm bên trong AMR. Hiện
project chỉ triển khai CartPole, nhưng cấu trúc package dành chỗ để thêm AMR
sau này mà không phải đổi ranh giới CartPole.

```text
src/
├── cartpole/
│   ├── config.py                   # central application configuration
│   ├── env.py                      # Gymnasium environment + Gazebo backend
│   ├── runtime.py                  # load checkpoint + inference
│   ├── server.py                   # HTTP API + WebView server
│   ├── train.py                    # offline checkpoint training
│   └── sim/
│       ├── worlds/main.sdf         # Runtime world + WebSocket :9002
│       ├── worlds/train.sdf        # Headless training world
│       ├── models/cartpole/        # CartPole model
│       └── visualize/              # WebView static bundle
├── checkpoints/                    # generated PPO artifacts
├── tests/                          # contract and runtime tests
└── amr/                            # module peer trong tương lai
```

Static WebView và vendor Gazebo viewer nằm trong `src/cartpole/sim/visualize/`.

### Kiến trúc runtime

```text
src/checkpoints/ppo_cartpole.zip
             │
             ▼
python -m cartpole.server --mode rl --checkpoint ...
             │
             ├── RLRuntime
             │     ├── PPO.load(...)
             │     ├── model.predict(...)
             │     └── cartpole.env/TestFixture(src/cartpole/sim/worlds/main.sdf)
             │
             ├── HTTP API :8009 ─────────────► Browser top bar
             │
             └── WebsocketServer :9002 ──────► Browser 3D WebView
```

Version 2 chạy một process `cartpole.server`. `cartpole.runtime` tạo Gazebo `TestFixture` từ `src/cartpole/sim/worlds/main.sdf`; không chạy thêm `gz sim`. `<gui>` trong SDF chỉ là cấu hình client và không mở desktop khi không spawn GUI client. Runtime chờ `RLRuntime` khởi tạo xong trước khi phục vụ WebView; WebSocket `9002` được mở bởi world runtime.

Mọi thao tác `TestFixture`, `env.step()` và `env.reset()` chạy trên một RL worker thread. HTTP handler chỉ enqueue command và đọc telemetry snapshot.

Policy chạy mỗi `0.005 s` simulation time và được pace gần real time. Browser poll telemetry khoảng `10 Hz`; scene publish khoảng `30 Hz` qua port `9002`. Khi Start chuyển sang `BALANCING`, WebView reconnect pose stream và nội suy các pose 30 Hz theo render frame để tránh mất stream lúc rời `IDLE` và giảm chuyển động giật.

Checkpoint thiếu hoặc không load được làm runtime chuyển sang `ERROR`; không fallback sang random action.

### Runtime state và API

```text
LOADING ──checkpoint OK──► IDLE ──Start──► BALANCING
   │                         ▲                │
   └──checkpoint error──► ERROR              ├──Reset──► IDLE
                                             ├──limit──► FAILED
                                             └──30 s───► SUCCESS
```

| Method | Endpoint | Mục đích |
|---|---|---|
| `GET` | `/api/health` | Kiểm tra mode, Gazebo và checkpoint |
| `GET` | `/api/rl/status` | Lấy trạng thái mới nhất cho top bar |
| `POST` | `/api/rl/start` | Reset nếu cần và bắt đầu episode |
| `POST` | `/api/cartpole/reset` | Dừng episode và trở về `IDLE` |
| `POST` | `/api/rl/disturbance` | Push trái hoặc phải |

Trong RL mode, endpoint manual `/api/cartpole/command` trả `409 Conflict` để không tranh chấp với policy.

Payload tối thiểu của `/api/rl/status`:

```json
{
  "phase": "BALANCING",
  "ready": true,
  "pole_angle_deg": 3.2,
  "action": 0,
  "force": -2000,
  "episode_time": 6.2,
  "result": null
}
```

Disturbance được enqueue cho RL worker và apply như lực ngoài policy trong 10 action intervals, tương đương 50 ms. Lực mặc định là `3000 N` theo hướng trái hoặc phải và được clamp ở backend.

### WebView

WebView chỉ có một top bar; toàn bộ phần còn lại dành cho scene 3D. Camera được đặt lùi ra để quan sát vùng CartPole rộng hơn theo chiều ngang và thấy rõ ground. Lưới viewer rộng `100 × 100`, cùng kích thước với ground, với mỗi ô rộng `5 m`.

```text
┌──────────────────────────────────────────────────────────────────────────┐
│ ● BALANCING     Pole +3.2°     ◀ -2000 N     06.2s     [▶] [↺] [←] [→] │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│                                                                          │
│                         Gazebo 3D WebView                                │
│                                                                          │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

- Status dot: xanh là ready, vàng là loading, đỏ là error. Tooltip hiển thị chi tiết.
- Live data: phase, pole angle, action/force và episode time.
- `▶`: Start, `↺`: Reset, `←`: Push Left, `→`: Push Right.
- Mỗi nút có tooltip và `aria-label`.
- Không có panel, modal, chart hoặc training metric.

Khi kết thúc, live data được thay bằng kết quả ngay trên top bar:

```text
│ ● SUCCESS      Simulation ended after 30.0s              [▶] [↺] [←] [→] │
```

```text
│ ● FAILED       Episode failed                            [▶] [↺]          │
```

### Kịch bản demo

1. Đặt checkpoint tương thích tại `src/checkpoints/ppo_cartpole.zip`.
2. Khởi động Version 2 và mở `http://<server-ip>:8009/`.
3. Xác nhận status dot màu xanh rồi nhấn `Start`.
4. Quan sát pole angle và action trái/phải trong 5–10 giây.
5. Nhấn `Push Left`, quan sát pole lệch và agent phục hồi.
6. Nhấn `Push Right` và quan sát agent phục hồi lần nữa.
7. Episode đạt 30 giây thì top bar hiển thị `SUCCESS`.
8. Nhấn `Reset` để trở về `IDLE`.

Nếu pole hoặc cart vượt limit, top bar hiển thị `FAILED` và giữ nguyên kết quả cho đến khi người dùng nhấn `Reset`.

### Deliverables

#### Artifact được tạo offline

| File | Deliverable |
|---|---|
| `src/checkpoints/ppo_cartpole.zip` | PPO checkpoint do training command tạo và runtime load read-only |

#### File thêm mới

| File | Deliverable |
|---|---|
| `src/cartpole/config.py` | Central paths, simulation, server và training configuration |
| `src/cartpole/env.py` | Shared environment/Gazebo callbacks; nhận world path, không chứa train hoặc GUI code |
| `src/cartpole/runtime.py` | Load checkpoint, inference loop, state, telemetry snapshot và command queue |
| `src/cartpole/server.py` | HTTP API, process startup và static WebView server |
| `src/cartpole/train.py` | Entry point offline duy nhất được phép gọi `PPO.learn()` và lưu checkpoint |
| `src/tests/test_rl_contract.py` | Test observation/action mapping, limits, checkpoint loading và runtime |

#### File sửa

| File | Thay đổi |
|---|---|
| `src/cartpole/sim/visualize/index.html` | Markup của RL top bar và 3D viewer |
| `src/cartpole/sim/visualize/app.js` | Viewer logic, API polling, controls và pose interpolation |
| `src/cartpole/sim/visualize/styles.css` | Layout top bar, controls và viewer |
| `src/cartpole/sim/visualize/vendor/*` | WebSocket/Gazebo viewer dependencies |
| `pyproject.toml` | Khai báo package `src/cartpole` và dependencies |
| `uv.lock` | Cập nhật theo `pyproject.toml` |
| `README.md` | Thêm command tạo checkpoint và command chạy Version 2 WebView |
| `docs/pole_rl.md` | Plan và scope Version 2 này |

#### File giữ nguyên

| File | Lý do |
|---|---|
| `src/cartpole/sim/models/cartpole/model.sdf` | Giữ nguyên dynamics |
| `src/cartpole/sim/models/cartpole/model.config` | Giữ nguyên model identity |
| `src/cartpole/sim/worlds/main.sdf` | World cho V1 và V2 runtime, có WebSocket :9002 |
| `src/cartpole/sim/worlds/train.sdf` | World headless cho offline training, không bind WebSocket |
| `src/cartpole/sim/visualize/vendor/gz.js`, `gz3d.js`, `eventemitter2.min.js` | Tiếp tục dùng WebView stack hiện tại |

### Command mục tiêu

Tạo checkpoint offline từ project root:

```bash
uv run python -m cartpole.train \
  --steps 25000 \
  --output src/checkpoints/ppo_cartpole
```

Command tạo `src/checkpoints/ppo_cartpole.zip` rồi kết thúc. Không giữ Gazebo server, WebSocket hoặc HTTP server chạy nền.

Chạy contract và runtime tests:

```bash
uv run python -m unittest discover -s src/tests -v
```

Chạy runtime sau khi checkpoint đã tồn tại. Không chạy `gz sim` hoặc Gazebo GUI riêng:

```bash
uv run python -m cartpole.server \
  --mode rl \
  --checkpoint src/checkpoints/ppo_cartpole.zip \
  --host 0.0.0.0 \
  --port 8009
```

Sau đó mở:

```text
http://<server-ip>:8009/
```

### Thứ tự triển khai

1. Centralize cấu hình trong `src/cartpole/config.py`.
2. Tạo shared `src/cartpole/env.py` từ phần environment của Gazebo example.
3. Tạo trainer trong `src/cartpole/train.py`, dùng `src/cartpole/sim/worlds/train.sdf` không có WebSocket.
4. Chạy trainer, load lại và chạy evaluation checkpoint.
5. Tạo `RLRuntime` load-only và deterministic inference loop.
6. Tích hợp runtime và API vào `src/cartpole/server.py`.
7. Thêm RL top bar vào `src/cartpole/sim/visualize/index.html`.
8. Thêm environment và runtime test.
9. Chạy WebView-only smoke test theo kịch bản demo.
10. Cập nhật README bằng hai command đã verify.

### Acceptance criteria

- Version 1 manual vẫn chạy như trước.
- Training chạy headless bằng code trong `src/cartpole/train.py`; checkpoint load được và pass đủ ba evaluation episode.
- Kết thúc training không để lại Gazebo, WebSocket hoặc HTTP process chạy nền.
- Version 2 khởi động bằng một command và chỉ cần browser.
- Không có Gazebo desktop process hoặc cửa sổ Gazebo GUI.
- Runtime không chứa hoặc gọi `PPO.learn()`.
- Checkpoint thiếu hoặc không load được chuyển runtime sang `ERROR`.
- Browser hiển thị scene, pole angle, action, episode time và trạng thái.
- Hai nút Push tạo disturbance nhìn thấy được; policy vẫn là nguồn control action.
- Episode viewer cho pole ngã tự nhiên và chuyển sang `SUCCESS` sau 30 giây.
- Reset đưa simulation và top bar về `IDLE` mà không restart process.

### Ngoài phạm vi

- Fine-tune từ WebView hoặc train trong runtime process.
- Hyperparameter search, training UI, TensorBoard hoặc biểu đồ training.
- Upload/download hoặc chọn checkpoint trên browser.
- Gazebo Sim desktop GUI.
- World/model mới hoặc thay đổi dynamics.
- Algorithm khác PPO hoặc continuous action.
- AMR, `DiffDrive`, sensor, mapping hoặc navigation.
- Authentication, database, cloud deployment hoặc multi-user.
