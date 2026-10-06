# MyCobot Isaac Sim 최소 제어 환경

이 프로젝트는 최소 제어 환경과 기구학 기반 Cartesian point-goal MPPI를 포함한다. 저장된 USD scene을 열고 MyCobot의
6개 arm joint를 PhysX articulation target으로 제어한다. Python에서 Ground Plane이나
Physics Scene을 매번 만들지 않는다.

## 구성

```text
mycobot/
├── assets/
│   ├── mycobot.usd
│   └── mycobot_asset/
├── scenes/
│   └── mycobot_default.usd
├── controllers/
│   ├── joint_commands.py
│   ├── kinematics.py
│   ├── mppi.py
│   └── torch_mppi.py
├── runtime/
│   ├── isaac_robot.py
│   └── control_session.py
├── remote/
│   ├── open_scene.py
│   ├── read_joint_state.py
│   ├── start_position_control.py
│   ├── start_velocity_control.py
│   ├── start_mppi_control.py
│   ├── control_status.py
│   └── stop_control.py
└── scripts/
    ├── read_joint_state.py
    ├── test_position_control.py
    ├── test_velocity_control.py
    ├── test_mppi_offline.py
    └── test_torch_mppi_offline.py
```

- `scenes/mycobot_default.usd`: `/World/PhysicsScene`, `/World/GroundPlane`,
  `/World/Light`, `/World/MyCobot`을 포함한다.
- `controllers/`: simulator와 무관한 명령 생성 계층이며 DH FK와 MPPI를 포함한다.
  MPPI rollout은 simulator와 분리되어 있다.
- `runtime/`: 이미 실행 중인 Isaac Sim 안에서 articulation과 비동기 제어 작업을
  유지한다.
- `remote/`: 서버 터미널에서 현재 실행 중인 Isaac Sim으로 보낼 짧은 진입점이다.
- `scripts/`: Isaac Sim의 시작부터 종료까지 한 프로세스에서 수행하는 독립 실행형
  smoke test이다.

제어 흐름은 다음과 같다.

```text
Isaac Sim Robot -> q, dq -> Controller -> q_cmd 또는 dq_cmd
                -> Articulation target -> PhysX simulation
```

상태를 순간 이동시키는 API는 사용하지 않는다. 위치 제어는 position target,
속도 제어는 velocity target을 전달한다.

## 권장 실행: 스트리밍 Isaac Sim + 터미널 제어

이 방식에서는 Isaac Sim을 한 번 실행해 계속 유지한다. 데스크톱은 영상과 UI만
WebRTC로 보고, 서버의 별도 터미널에서 Python 파일을 실행 중인 동일한 Isaac Sim에
전송한다. Script Editor에 코드를 붙여넣을 필요가 없다.

### 1. 서버 터미널 A: Isaac Sim 스트리밍 실행

`<SERVER_IP>`를 서버의 완전한 IPv4 주소로 바꾼다. Python Server는 기본적으로
localhost의 TCP 8226에서만 명령을 받으므로 외부에 8226을 열지 않는다.

```bash
cd /home/arclab/workspace/IsaacSim/_build/linux-aarch64/release

./isaac-sim.streaming.sh \
  --no-ros-env \
  --enable isaacsim.code_editor.python_server \
  --/exts/omni.kit.livestream.app/primaryStream/publicIp="$SERVER_IP" \
  --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100 \
  --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998
```

기존 스트리밍 명령에 `--enable isaacsim.code_editor.python_server`만 핵심적으로
추가된 것이다. ROS 환경 충돌을 피하려고 `--no-ros-env`도 넣었다.

### 2. 데스크톱: WebRTC 클라이언트 접속

```bash
cd ~/apps/isaac-webrtc-client/opt/"Isaac Sim WebRTC Streaming Client"
./isaacsim-webrtc-streaming-client
```

기존과 같은 IP와 포트를 입력해 접속한다.

### 3. 서버 터미널 B: scene 열기와 제어

sender는 시스템 Python으로 실행해도 되며 별도 패키지가 필요 없다.

```bash
PROJECT=/home/arclab/workspace/urop2_manipulator/mycobot
SEND=/home/arclab/workspace/IsaacSim/skills/isaac-sim-remote/scripts/isaacsim_send.py

python3 "$SEND" --file "$PROJECT/remote/open_scene.py"
python3 "$SEND" --file "$PROJECT/remote/read_joint_state.py"
```

위치 명령을 시작한다. 기본 목표는 degree 단위
`[20, 15, 10, -15, -10, 20]`이고 기본 실행 길이는 100 frame이다.

```bash
python3 "$SEND" --file "$PROJECT/remote/start_position_control.py"
python3 "$SEND" --file "$PROJECT/remote/control_status.py"
```

목표와 실행 길이를 바꾸려면:

```bash
python3 "$SEND" --file "$PROJECT/remote/start_position_control.py" \
  --arg 'q_target_deg=[0,10,20,0,-10,0]' --arg steps=150
```

초 단위로 지정하려면 `duration_sec`를 사용한다. 현재 physics timestep이
`0.02 s`이므로 아래 명령은 내부적으로 125 step으로 변환된다.

```bash
python3 "$SEND" --file "$PROJECT/remote/start_position_control.py" \
  --arg 'q_target_deg=[0,10,20,0,-10,0]' --arg duration_sec=2.5
```

`steps`와 `duration_sec`는 둘 중 하나만 지정할 수 있다. 둘 다 생략하면 위치
제어는 2초(100 step), 속도 제어는 1초(50 step)를 기본값으로 사용한다.

속도 명령의 기본값은 degree/s 단위 `[10, 0, 0, 0, 0, 0]`이며, 정해진 frame이
끝나거나 중지하면 velocity target을 0으로 되돌린다.

```bash
python3 "$SEND" --file "$PROJECT/remote/start_velocity_control.py" \
  --arg 'dq_target_deg_s=[10,0,0,0,0,0]' --arg duration_sec=3.0
python3 "$SEND" --file "$PROJECT/remote/control_status.py"
python3 "$SEND" --file "$PROJECT/remote/stop_control.py"
```

새 위치/속도 제어를 시작하면 이전 제어 작업은 자동으로 중지된다. 제어 loop는
background async task로 실행되므로 sender 명령은 즉시 반환하고 스트리밍 UI는 계속
응답한다. 진행 상태와 마지막 `q`, `dq`, 오차는 `control_status.py`로 확인한다.
상태에는 `physics_dt_sec`, 변환된 `total_steps`, 실제 `duration_sec`도 포함된다.

### Cartesian point-goal MPPI

기본 목표 `(194.36, 56.79, 309.06) mm`로 15초간 기구학 MPPI를 실행한다. 이 위치는
안전한 기준 자세 `q=[30,-20,-30,0,20,0] deg`를 DH FK로 변환한 값이다. 좌표는
MyCobot base frame 기준 flange 위치이다. 실제 runtime optimizer는 `pytorch_mppi.MPPI`이며,
Isaac에 포함된 PyTorch/CUDA를 이용한다. NumPy 구현은 비교용 baseline으로 남겨 두었다.

scene의 `/World/Markers/MppiGoal`에는 collision이 없는 반지름 12 mm의 빨간 구체가 있다.
MPPI를 시작하면 전달한 `p_goal_mm`를 MyCobot base frame에서 world frame으로 변환해 구체
위치를 자동으로 갱신하고 표시한다. MPPI가 정상 완료되거나 중지·오류로 끝나면 다시
숨긴다. 따라서 idle 상태에서는 보이지 않고, 실행 중에는 로봇 flange가 이동할 목표
위치를 나타낸다.

최초 한 번 프로젝트 로컬 의존성을 설치한다. PyTorch 자체는 중복 설치하지 않는다.

```bash
cd /home/arclab/workspace/IsaacSim/_build/linux-aarch64/release
./python.sh -m pip install --no-deps \
  --target /home/arclab/workspace/urop2_manipulator/mycobot/third_party \
  -r /home/arclab/workspace/urop2_manipulator/mycobot/requirements-mppi.txt
```

```bash
python3 "$SEND" --file "$PROJECT/remote/start_mppi_control.py"
python3 "$SEND" --file "$PROJECT/remote/control_status.py"
```

Isaac/WebRTC 프로세스는 유지하고 실행 중인 controller task만 중지하려면 다른 서버
터미널에서 다음을 실행한다. MPPI position mode는 취소 시 측정한 현재 q를 새 position
target으로 설정해 그 자리에서 hold한다.

```bash
python3 "$SEND" --file "$PROJECT/remote/stop_control.py"
```

스트리밍을 실행한 터미널에서 `Ctrl+C`를 누르면 controller가 아니라 Isaac Sim 전체가
종료되므로 제어 중지 용도로 사용하지 않는다.

목표와 실행 시간을 바꾸는 예:

```bash
python3 "$SEND" --file "$PROJECT/remote/start_mppi_control.py" \
  --arg 'p_goal_mm=[194.36,56.79,309.06]' --arg duration_sec=20
```

정확한 control step 수로 실행하려면 `steps`를 전달한다. `steps`가 있으면
`duration_sec`보다 우선한다.

```bash
python3 "$SEND" --file "$PROJECT/remote/start_mppi_control.py" --arg steps=750
```

정해진 step을 모두 완료하면 `results/`에 같은 stem의 PNG, NPZ, JSON을 저장한다.
PNG 위쪽은 Cartesian error, 아래쪽은 MPPI `command()`와 optimizer solve 연산
시간이다. JSON에는 `p_goal`, 전체 `MPPIConfig`, backend와 `WorstT`, `meanT`,
`p95T`, `stdT`가 기록된다. 중단되거나 오류가 난 실행은 결과 파일을 만들지 않는다.
터미널에도 완료 직후 timing 요약과 세 파일의 전체 경로가 출력된다.

독립 프로세스에서 같은 실험을 실행하는 명령은 다음과 같다.

```bash
cd /home/arclab/workspace/IsaacSim/_build/linux-aarch64/release
./python.sh /home/arclab/workspace/urop2_manipulator/mycobot/scripts/run_mppi.py \
  --headless --steps 750
```

MPPI는 50 Hz이고 매 control step마다 측정 q에서 `q_cmd=q+0.02*dq_cmd`를 만든다.
현재 기구학 MPPI 단계에서는 기본 USD의 physics도 50 Hz로 두며, control마다 PhysX step과
WebRTC 화면 갱신을 한 번 수행한다. 기본 `render_every=1`이므로 모든 제어 step이 화면에
반영된다. 상태의 `error`는
mm 단위 EE 오차이고 `command`는 rad/s 단위 `dq_cmd`이다. 설계, 비용, DH 및 제한의
근거는 [docs/mppi_design.md](docs/mppi_design.md)에 정리했다.

기본 최적화 설정은 `H=10`, `K=136`, `lambda=1.0`이다. dynamics/running cost는
Triton으로 compile하며 이 서버의 GB10에서 warm-up 이후 offline 평균 command 계산
시간은 약 13 ms로 측정됐다. 필요하면 원격 실행 인자
`horizon`, `num_samples`로 바꿀 수 있다.

렌더 부담을 줄여야 할 때만 `--arg render_every=2` 또는 더 큰 값을 사용할 수 있다.
이때 중간 step은 Isaac의 physics-only API로 진행하므로 화면 갱신 빈도도 같이 낮아진다.

MPPI 실행 중에는 실제 로봇 튜닝값과 구분되는 시뮬레이션 임시 position drive gain
`Kp=[120,120,100,80,50,30]`, `Kd=[4,4,3.5,3,2,1.5]`를 적용한다. 기존 USD의 매우
낮은 gain으로는 한 step 앞의 작은 position target이 중력을 이기지 못해 팔이 처졌기
때문이다. 실제 로봇용 gain으로 간주하면 안 된다.

또한 rollout에 중력 항이 없는 현재 기구학 모델과 일치시키기 위해 기본 scene의 gravity는
0으로 설정되어 있다. 동역학 기반 rollout 또는 실제 로봇 actuator 검증 단계에서는 이를
`9.81 m/s^2`로 되돌리고 physics substep, 관성, 마찰 및 gain을 함께 검증해야 한다.

최초 실행에서는 Torch/Triton graph compile 때문에 시작이 느릴 수 있다. compile은
제어 loop 전에 수행되고 workspace의 `.torch_cache/`에 저장된다. 이후 터미널 로그에는
`solve`, `command`, 전체 `cycle`, 누적 wall/sim 시간과 `RTF`가 출력된다. status의
`backend=cuda`, `gpu_name`, `cuda_memory_allocated_mb`로 실제 CUDA 사용도 확인한다.

## PhysX Drive

PhysX Drive는 joint에 연결된 내장 actuator이자 spring-damper 제어기이다. 위치
모드에서는 목표 위치와 현재 위치의 오차, 목표 속도와 현재 속도의 오차를 사용해
각 physics step에서 구동 토크를 계산한다.

```text
torque ≈ stiffness * (q_target - q)
       + damping * (dq_target - dq)
```

위치 모드의 `dq_target`은 0이다. 속도 모드에서는 stiffness를 0으로 만들어 위치
오차 항을 제거하고 속도 오차만 사용한다. 계산된 토크는 USD의 maxForce 제한과
joint limit의 영향을 받고, 실제 관절 가속도와 위치 변화는 PhysX가 질량, 관성,
중력, 접촉 및 관절 간 결합을 함께 풀어 결정한다. 따라서 position target은
한 step에 일정 각도씩 이동시키는 trajectory 명령이 아니다.

Isaac Sim을 다시 시작하면 순서도 `open_scene.py`부터 다시 실행한다. USD를 UI에서
직접 열어도 되지만, 이 프로젝트의 scene과 runtime handle을 일치시키려면
`open_scene.py` 사용을 권장한다.

## 독립 실행형 smoke test

스트리밍 세션과 분리해서 CI나 빠른 검증을 할 때 사용한다. 앞의 세 명령은 자체
Isaac Sim 프로세스를 시작하고 테스트 후 종료하며, MPPI offline test는 Isaac 앱 없이
기구학 controller만 검사한다. 이미 떠 있는 스트리밍 화면은 제어하지 않는다.

```bash
cd /home/arclab/workspace/urop2_manipulator/mycobot
ISAAC_PYTHON=/home/arclab/workspace/IsaacSim/_build/linux-aarch64/release/python.sh

"$ISAAC_PYTHON" scripts/read_joint_state.py --headless
"$ISAAC_PYTHON" scripts/test_position_control.py --headless
"$ISAAC_PYTHON" scripts/test_velocity_control.py --headless
"$ISAAC_PYTHON" scripts/test_mppi_offline.py
"$ISAAC_PYTHON" scripts/test_torch_mppi_offline.py
```

즉, 평소 개발은 `streaming.sh + remote/`, 자동 검증은 `scripts/`를 사용한다.
