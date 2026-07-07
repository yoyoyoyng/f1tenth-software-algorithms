# Wall Following 설계 메모

```text
< left_wallfollowing> 
 2022130029이찬용 
1.우측 라이더값 → 좌측 라이더값 
# before 
angle_b = math.radians(-80.0) 
angle_a = math.radians(-35.0) 

# after 
angle_b = math.radians(+80.0) 
angle_a = math.radians(+35.0) 
-오른쪽 벽을 측정하던 음수 각도를 왼쪽 벽 측정을 위해 양수로 변경했습니다. 

2.theta 
# before 
theta = angle_a - angle_b 

# after 
theta = angle_b – angle_a 
 - 측정 각도 기준이 왼쪽으로 바뀌었기 때문에 두 각도 사이의 차이 방향도 반전했습니다. 

 3. steering_angle 
# before 
steering_angle = self.kp * error + self.ki * self.integral + self.kd * d_error 

# after 
steering_angle = -(self.kp * error + self.ki * self.integral + self.kd * d_error) 
-왼쪽 각도 기준으로 바뀌면서 error 부호가 반전되므로, 조향 방향을 올바르게 유지하기 위해 마이너스를 붙였습니다. 
 4. . scan_callback 교차로 조건 
# before 
right_85 = self.get_range(msg, math.radians(-85.0)) 
if front > self.FRONT_THRESHOLD and 1.3 < right_85 < 1.8: 
 self.publish_drive(+0.12, 6.0) 
 return 

# after → 제거 
left_85 = self.get_range(msg, math.radians(+85.0)) 
self.pid_control(self.get_error(msg, self.desired_distance_left)) 
- pid로만 월팔로잉을 제대로 진행해보고싶어서 교차로 판별 코드를 제거했습니다.
```
