# ROS 2 Hybrid A* & MPPI Navigation with Smart Recovery

이 프로젝트는 ROS 2 환경에서 **Hybrid A*** 알고리즘을 이용한 전역 경로 계획(Global Planning)과 **MPPI(Model Predictive Path Integral)** 제어를 이용한 지역 경로 추종(Local Control)을 구현한 자율주행 패키지입니다.

특히 좁은 공간이나 코너에서 로봇이 갇혔을 때, 단순 회전이 아닌 **후진 조향(Backing & Turning)**을 통해 주행 공간과 진입 각도를 스스로 확보하는 고급 회복 알고리즘(Recovery Behavior)이 포함되어 있습니다.

## 🚀 Key Features

### 1. Global Planner: Hybrid A*
- **기구학적 제약 고려**: 로봇의 최소 회전 반경과 조향 각도(Steering set: -20°, 0°, +20°)를 고려하여 경로를 생성합니다.
- **부드러운 경로**: 격자 기반의 일반 A*와 달리, 연속적인 좌표계(x, y, theta)를 사용하여 로봇이 실제로 주행 가능한 부드러운 곡선 경로를 제공합니다.
- **Heuristic**: 유클리드 거리를 사용하여 목표점까지의 비용을 최적화했습니다.

### 2. Local Controller: MPPI (Model Predictive Path Integral)
- **샘플링 기반 예측 제어**: 매 제어 주기(0.05s)마다 80개의 가상 주행 경로를 생성하고, 미래 1.2초(12 steps)를 예측합니다.
- **Cost Function 최적화**:
  - **Tracking**: 전역 경로와의 거리 오차 최소화
  - **Smoothing**: 급격한 제어 입력 변화 억제
  - **Collision**: 로봇의 실제 Footprint를 고려한 정밀 장애물 회피
- 로봇의 동역학적 한계를 고려하여 최적의 선속도(v)와 각속도(w)를 도출합니다.

---

## 🛠️ Requirements

- **OS**: Ubuntu 22.04 (Jammy Jellyfish) recommended
- **ROS 2 Distro**: Humble Hawksbill
- **Python Dependencies**:
  ```bash
  pip install numpy

📦 Installation
1. ROS 2 Workspace 이동
code
Bash
cd ~/ros2_ws/src
2. Repository Clone
(패키지 이름은 코드 상의 import 경로에 맞춰 my_autonomous_pkg로 설정하거나, 코드 내의 import 문을 수정해야 합니다.)
code
Bash
git clone [YOUR_GIT_URL] my_autonomous_pkg
3. Build
code
Bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash


🏃 Usage
1. 시뮬레이션 및 맵 실행
먼저 TurtleBot3 Gazebo 시뮬레이션이나 실제 로봇, 그리고 Map Server가 실행되어 있어야 합니다.
code
Bash
# 예시: TurtleBot3 Gazebo 실행
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
2. 네비게이션 노드 실행
code
Bash
ros2 run my_autonomous_pkg main_controller
3. RViz 제어
RViz2를 실행합니다.
2D Pose Estimate 버튼을 클릭하여 맵 상에서 로봇의 초기 위치를 지정합니다.
2D Goal Pose 버튼을 클릭하여 목표 지점을 설정합니다.
Hybrid A*가 초록색 경로(Path)를 생성합니다.
MPPI가 빨간색/파란색 예측 경로를 그리며 주행을 시작합니다.


📂 Project Structure
code
Bash
my_autonomous_pkg/
├── hybrid_a_star.py    # Hybrid A* 알고리즘 (Node, Heuristic, Collision Check)
├── mppi.py             # MPPI 제어기 (Sampling, Cost Calculation, Softmax Aggregation)
├── main_controller.py  # 메인 노드 (Sensor Fusion, State Machine, Path Managing)
└── ...


⚙️ Parameters (Key Configs)
File	Parameter	Value	Description
hybrid_a_star.py	resolution	0.1 m	그리드 해상도
hybrid_a_star.py	step_size	0.5 m	노드 확장 단위 거리
mppi.py	num_samples	80	예측 제어 샘플 개수
mppi.py	horizon	12	예측 구간 (dt=0.1s * 12 = 1.2s)
main_controller.py	recovery	Logic	전방 30cm, 각도 60도 이상 차이 시 후진 발동
