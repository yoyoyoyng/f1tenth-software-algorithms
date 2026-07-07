# Theory Notes

이 문서는 업로드한 F1TENTH 강의 자료를 참고하여 각 구현 알고리즘의 이론적 배경을 포트폴리오용으로 정리한 것입니다.

## 1. Wall Following - Feedback Control & PID

Wall Following은 경로 추종 문제를 feedback control로 해석한 예입니다. 목표는 차량이 벽과 일정한 거리를 유지하고, 동시에 벽과 평행하게 주행하도록 하는 것입니다. 이를 위해 LiDAR로 현재 벽까지의 거리를 측정하고, 목표 거리와의 차이를 error로 정의합니다.

```text
Reference Distance
      │
      ▼
Distance Error
      │
      ▼
PID Controller
      │
      ▼
Steering Angle
```

- P term: 현재 error에 비례하여 빠르게 보정
- D term: error 변화율을 이용하여 overshoot와 진동 감소
- I term: 누적 error를 보정하여 steady-state error 감소

본 구현에서는 조향각이 커지는 경우 속도를 낮추도록 하여 코너와 벽 근접 상황에서 안정성을 높였습니다.

## 2. Gap Following - Reactive Obstacle Avoidance

Gap Following은 map 없이 현재 LiDAR scan만으로 장애물을 회피하는 reactive navigation입니다. 핵심은 LiDAR range array에서 일정 거리 threshold 이상인 연속 구간을 gap으로 보고, 그중 차량이 실제로 통과 가능한 방향을 선택하는 것입니다.

단순히 가장 먼 point를 선택하면 차량 폭과 장애물 모서리를 고려하지 못하기 때문에 다음 보정이 필요합니다.

```text
LiDAR Scan
      │
      ▼
Range Filtering / Smoothing
      │
      ▼
Safety Bubble
      │
      ▼
Disparity Extension
      │
      ▼
Best Gap Selection
      │
      ▼
Steering & Speed
```

- Safety Bubble: 가장 가까운 장애물 주변 beam을 제거하여 충돌 가능 영역을 제외
- Disparity Extension: 인접 beam의 거리 차이가 큰 부분을 장애물 경계로 보고 차량 폭만큼 위험 영역 확장
- Wiggling Reduction: 가장 먼 단일 point가 아니라 gap 중심/안정적인 best point를 선택하여 좌우 진동 감소

## 3. Pure Pursuit - Geometric Path Tracking

Pure Pursuit는 waypoint 기반 path tracking 알고리즘입니다. 현재 차량 위치에서 일정 lookahead distance `L`만큼 떨어진 target point를 찾고, 차량이 해당 점을 따라가도록 원호를 구성하여 steering angle을 계산합니다.

```text
Waypoints
      │
      ▼
Current Vehicle Pose
      │
      ▼
Lookahead Target Point
      │
      ▼
Curvature Calculation
      │
      ▼
Steering Angle
```

Lookahead distance는 주행 특성을 크게 바꿉니다.

- 작은 lookahead: 빠른 반응, 공격적인 코너링, 진동 또는 과조향 가능
- 큰 lookahead: 부드러운 주행, tracking error 증가 가능

본 구현에서는 속도와 곡률에 따라 lookahead와 속도를 조절하여 직선에서는 빠르게, 코너에서는 안정적으로 주행하도록 구성했습니다.

## 4. RRT* - Sampling-based Motion Planning

RRT*는 continuous planning problem을 해결하기 위한 sampling-based planner입니다. 본 구현에서는 LiDAR로부터 local occupancy grid를 만들고, 충돌 가능성이 낮은 sample을 점진적으로 연결하여 local path를 생성했습니다.

```text
LiDAR Scan
      │
      ▼
Local Occupancy Grid
      │
      ▼
Random Sampling
      │
      ▼
Nearest Node Search
      │
      ▼
Collision Checking
      │
      ▼
Rewiring / Path Optimization
      │
      ▼
Local Path
```

RRT는 빠르게 feasible path를 찾는 데 강점이 있고, RRT*는 rewiring을 통해 더 낮은 cost의 path로 개선할 수 있습니다. 본 프로젝트에서는 생성된 local path를 다시 Pure Pursuit 방식으로 추종하도록 연결했습니다.

## Extracted Theory Figures

필요한 이론 그림은 `docs/theory-assets/`에 정리했습니다.
