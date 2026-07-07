# Algorithm Notes

## Wall Following

Wall Following은 LiDAR의 특정 각도 range를 이용해 벽과 차량의 상대적인 거리 및 기울기를 추정하고, 목표 거리와의 오차를 PID 제어로 줄이는 방식입니다. 주행 안정성을 위해 조향각이 커질수록 속도를 낮추는 구조를 사용했습니다.

## Gap Following

Gap Following은 주어진 LiDAR scan에서 장애물 주변을 안전하게 확장한 뒤 가장 주행 가능성이 높은 gap을 선택하는 reactive 방식입니다. Disparity Extension은 장애물 경계에서 발생하는 급격한 range 차이를 이용해 차량 폭만큼 위험 영역을 넓히고, Safety Bubble은 가까운 장애물 주변을 제거하여 충돌 가능성을 줄입니다.

## Pure Pursuit

Pure Pursuit는 waypoint 경로 위의 lookahead target을 선택하고, 차량 좌표계 기준 target 위치를 이용해 조향각을 계산하는 경로 추종 알고리즘입니다. 본 구현에서는 속도 기반 lookahead, 곡률 기반 감속, 조향 기반 속도 제한을 함께 사용했습니다.

## RRT*

RRT*는 local occupancy grid 위에서 collision-free path를 탐색하는 sampling-based planning 알고리즘입니다. 본 구현에서는 주행 중 경로가 막혔는지 확인하고, 막힌 경우 RRT* local path를 생성한 뒤 Pure Pursuit 방식으로 해당 path를 추종하도록 구성했습니다.


---

## Theory References Added

이론 설명은 F1TENTH 강의 자료의 다음 내용을 바탕으로 보강했습니다.

- Wall Following: feedback control, PID objectives, PID error term
- Gap Following: gap definition, safety bubble, disparity extension, wiggling reduction
- Pure Pursuit: waypoint assumption, lookahead target, steering geometry, goal update
- RRT*: occupancy grid, motion planning problem, RRT tree expansion, RRT* rewiring

관련 그림은 `docs/theory-assets/`에 저장했습니다.
