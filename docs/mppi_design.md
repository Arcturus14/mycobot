# 기구학 MPPI 설계 기록

## 좌표와 모델

- 상태: `q [rad]`, 제어 입력: `dq_cmd [rad/s]`
- 제어 주기: 50 Hz (`dt = 0.02 s`)
- rollout: `q[k+1] = q[k] + dt * dq_cmd[k]`
- 목표: MyCobot **base frame의 flange 위치** `p_goal [m]`
- 기본값: `(194.36, 56.79, 309.06) mm`. 임의의 Cartesian 점이 아니라
  `q_goal=[30,-20,-30,0,20,0] deg`를 DH FK로 변환한 값이다. 모든 joint limit에서
  여유가 있고 현재 self-collision 근사 비용은 0이다.
- 실행 명령: `q_cmd = q_measured + 0.02 * dq_cmd`; 이 position target을
  하나의 0.02 s PhysX step에서 추종한다. 관절 상태를 teleport하지 않는다.
- 현재 기구학 검증 단계에서는 physics와 control을 모두 50 Hz로 맞춘다. 동역학 검증으로
  넘어갈 때는 physics substep을 다시 늘리고 actuator/gain을 함께 재검증해야 한다.
- rollout에 중력 항이 없으므로 이 단계의 기본 scene도 gravity magnitude를 0으로 둔다.
  동역학 단계에서는 9.81 m/s^2로 복원한다.
- optimizer: `pytorch_mppi.MPPI` 0.9.1, 기본 `H=10`, `K=136`, `lambda=1.0`

FK는 Elephant Robotics의 `MyCobotBasic/ParameterList.h`에 있는 MyCobot 280 DH
상수와 theta offset을 standard DH 순서
`Rz(theta) Tz(d) Tx(a) Rx(alpha)`로 적용한다.

| joint | a (m) | d (m) | alpha | theta offset |
|---|---:|---:|---:|---:|
| 1 | 0 | 0.13156 | pi/2 | 0 |
| 2 | -0.1104 | 0 | 0 | -pi/2 |
| 3 | -0.0960 | 0 | 0 | 0 |
| 4 | 0 | 0.06462 | pi/2 | -pi/2 |
| 5 | 0 | 0.07318 | -pi/2 | pi/2 |
| 6 | 0 | 0.04860 | 0 | 0 |

출처: <https://github.com/elephantrobotics/MyCobotBasic/blob/main/ParameterList.h>

`p_goal`은 gripper finger 끝이 아니라 flange 원점이다. 이후 tool/TCP를 쓰려면
flange-to-tool 고정 transform을 FK 뒤에 곱하고 목표 좌표의 의미도 같이 바꿔야 한다.

## 비용함수

각 샘플의 총 비용은 다음 항의 합이다.

1. horizon 전체 EE 위치 오차와 terminal 위치 오차
2. `dq_cmd` 크기와 연속 command 사이의 변화량
3. joint limit에서 5도 안쪽으로 들어오면 증가하는 soft penalty
4. 서로 인접하지 않은 DH frame sphere 사이의 self-collision soft penalty
5. 목표 60 mm 안에서 커지는 속도 비용

실행 단계에서도 현재 EE 오차가 60 mm보다 작으면 command를 선형 축소하고, 5 mm
이하면 `dq_cmd=0`으로 만든다. Damped-least-squares Cartesian servo trajectory 일부를
proposal로 섞어 점 목표 문제의 샘플 퇴화를 줄이지만, 모든 proposal은 동일한 MPPI
비용으로 평가되고 importance weight로 갱신된다.

현재 self-collision 항은 정확한 mesh distance가 아니라 빠른 sphere-chain
근사이다. 명백한 arm-on-arm 자세를 피하는 1차 비용이며, PhysX collision shape가
최종 접촉을 처리한다. 실제 안전용 collision checker로 간주하면 안 된다.

## 속도 hard constraint

모든 MPPI sample, 갱신된 nominal command, 실제 출력에 관절별
`|dq| <= 2.0944 rad/s (120 deg/s)`를 적용한다. 현재 USD의 원본 URDF에는 일부 velocity
limit가 0으로 들어 있어 그대로 쓸 수 없었고, 이 값은 이 workspace의 기존 asset
repair 설정(`arc-mycobot/.../mycobot_urdf.py`)에 기록된 MyCobot 280 스펙 값이다.

제조사 Python API의 `speed=1..100`은 퍼센트 명령이라 rad/s 물리 상한의 직접 근거가
아니다. 따라서 2.0944 rad/s는 시뮬레이션용 잠정 상한이다. 실기가 도착하면 firmware,
모델 변형, payload별 허용 속도를 확인하고 관절별 값으로 교체해야 한다.

## 현재 검증 범위

`scripts/test_torch_mppi_offline.py`는 production Torch backend를 q=0에서 기본
목표까지 실행한다. 고정 seed 기준 초기 469.77 mm 오차를 약 5 mm까지 줄였고 모든
command가 hard constraint를 지키는지 검사한다. GB10에서 Torch/Triton compile과
warm-up 이후 평균 command 계산 시간은 약 13 ms였다. Isaac/PhysX의 추종 지연과
DH-USD flange 좌표 일치는
스트리밍 세션에서 별도로 확인해야 한다.

구조와 timing 방식은 참고 프로젝트
<https://github.com/Arcturus14/manipulator_control>에서 필요한 부분만 따랐다. 구체적으로
MPPI library와 robot wrapper의 분리, step-dependent running/terminal cost, 직전 입력
대비 smoothness, loop 밖 warm-up, CUDA synchronize를 이용한 solve time 측정을 적용했다.
