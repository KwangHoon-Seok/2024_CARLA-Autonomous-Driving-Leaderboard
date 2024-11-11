# 2024 CARLA Autonomous Driving Leaderboard
2024 CVPR Autonomous Grand Challenge

## CARLA 자율주행 리더보드

[CARLA 자율주행 리더보드](https://leaderboard.carla.org/)는 CARLA 시뮬레이터를 활용해 자율주행 시스템을 평가하는 공개 플랫폼입니다. 이 리더보드는 자율주행 에이전트의 성능과 안전성을 평가하기 위한 표준화된 환경을 제공하여 복잡한 도시 시뮬레이션에서 다양한 알고리즘을 비교할 수 있도록 설계되었습니다.

### Architecture
<img width="607" alt="image" src="https://github.com/user-attachments/assets/1cbc46e6-2f68-4468-ae83-39ae63fc3ffd">

---

## 개요

CARLA 리더보드는 연구자와 개발자가 자율주행 알고리즘을 통제된 현실감 있는 시뮬레이션 환경에서 테스트할 수 있도록 합니다. 경로 계획, 장애물 회피, 주변 에이전트와의 상호작용 등 자율주행 시스템의 다양한 능력을 비교할 수 있는 통일된 기준을 제공합니다.

### 주요 기능

- **표준화된 벤치마크 제공**: 자율주행 모델의 안전성, 신뢰성 및 효율성을 평가할 수 있는 일관된 평가 기준을 제공합니다.
- **복잡한 시뮬레이션 시나리오**: 동적 교통 요소, 다양한 날씨 조건, 예측할 수 없는 보행자 행동 등이 포함된 도시 시나리오를 제공합니다.
- **지속적인 평가**: 참가자들은 언제든지 모델을 제출하고 평가를 받을 수 있어, 지속적인 개선과 경쟁 비교가 가능합니다.
- **평가 지표 및 점수 산정**: 주행 목표 달성 여부, 충돌 최소화, 주행의 부드러움 등을 기준으로 시스템을 평가합니다.

---

## 알고리즘 소개

- **Behavior Trajectory Planner**: 각 경로에 따른 CSV 파일을 유연하게 처리하는 모듈.
- **Collision Check**: 앞 차량과의 충돌 여부를 TTC(TIME TO COLLISION) 알고리즘을 통해 확인하는 모듈.
- **DWA ROS Node**: 장애물 회피 시 작동하는 로컬 플래닝 모듈.
- **Lead Vehicle Flag**: 여러 장애물 중 ACC 알고리즘을 적용할 선행 차량을 식별하는 모듈.
- **LiDAR Processing**: LiDAR를 통해 객체 탐지 및 추적을 실행하는 모듈.
- **Localization**: GPS와 IMU를 통해 자차의 현재 위치를 추종하는 모듈.
- **Velocity Planner**: 자차의 위치에 따라 기준 속도를 조절하는 모듈.
- **Camera Detection**: Yolo v8 모델을 사용하여 객체를 탐지하는 모듈 (추후 업데이트 예정).

---

## 결과

- **3D Object Detection & Tracking**  
  ![3D Object Detection](https://github.com/user-attachments/assets/1bf0606f-3c6e-4367-896e-cf4741e41e82)

- **Local Planning**  
  ![Local Planning](https://github.com/user-attachments/assets/f00c80ba-9eb7-4913-b02c-30e0a351b001)

- **Longitudinal Control**  
  ![Longitudinal Control](https://github.com/user-attachments/assets/23d21732-7367-42b1-b113-6bddff448102)

- **Lateral Control**  
  ![Lateral Control](https://github.com/user-attachments/assets/785cfb11-9f31-455e-8da1-467b6cdfa17a)

- **Lead Vehicle Selection & ACC**  
  ![Lead Vehicle Selection](https://github.com/user-attachments/assets/89afde37-b93a-489d-b113-76c2912e71da)
