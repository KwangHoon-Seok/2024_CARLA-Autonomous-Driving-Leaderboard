# 2024_CARLA-Autonomous-Driving-Leaderboard
2024 CVPR Autonomous Grand Chanllenge
# CARLA 자율주행 리더보드

[CARLA 자율주행 리더보드](https://leaderboard.carla.org/)는 CARLA 시뮬레이터를 활용해 자율주행 시스템을 평가하는 공개 플랫폼입니다. 이 리더보드는 자율주행 에이전트의 성능과 안전성을 평가하기 위한 표준화된 환경을 제공하여 복잡한 도시 시뮬레이션에서 다양한 알고리즘을 비교할 수 있도록 설계되었습니다.

### 개요

CARLA 리더보드는 연구자와 개발자가 자율주행 알고리즘을 통제된 현실감 있는 시뮬레이션 환경에서 테스트할 수 있게 합니다. 경로 계획, 장애물 회피, 주변 에이전트와의 상호작용 등 자율주행 시스템의 다양한 능력을 비교할 수 있는 통일된 기준을 제공합니다.

### 프로세스

- **표준화된 벤치마크 제공**: 리더보드는 자율주행 모델의 안전성, 신뢰성 및 효율성을 평가할 수 있는 일관된 평가 기준을 제공합니다.
- **복잡한 시뮬레이션 시나리오**: 동적 교통 요소, 다양한 날씨 조건, 예측할 수 없는 보행자 행동 등이 포함된 다양한 도시 시나리오를 제공합니다.
- **지속적인 평가**: 참가자들은 언제든지 모델을 제출하고 평가를 받을 수 있어, 방법론의 지속적인 개선 및 경쟁 비교가 가능합니다.
- **평가 지표 및 점수 산정**: 주행 목표 달성 여부, 충돌 최소화, 주행의 부드러움 등을 기준으로 시스템을 평가합니다.

## 알고리즘 소개

#### Behavior_trajectory_planner
각 경로에 따른 csv파일을 유연하게 처리하는 module

#### Collision_check
앞 차량과의 충돌 여부를 TTC 알고리즘을 통해 확인하는 module

#### DWA_ros_node
장애물 회피 시 작동하는 local planning module

#### lead_vehicle_flag
수 많은 장애물들 중에 ACC 알고리즘을 적용할 lead_vehicle을 찾는 module

#### LiDAR_processing 
LiDAR을 통해 object detection과 object tracking을 실행하는 module

#### Localization 
GPS와 IMU를 통해 자차의 현재 위치 추종하는 module

#### Velocity Planner
자차의 위치에 따라 reference velocity를 조절해주는 module

#### Camera detection
Yolo v8 모델 사용 -> src 코드 사라짐 ...(추후 업데이트)



