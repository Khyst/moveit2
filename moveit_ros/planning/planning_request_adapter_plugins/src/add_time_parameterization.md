# AddTimeParameterization - Planning Request Adapter

## 📋 개요

`add_time_parameterization.cpp`는 MoveIt2의 **Planning Request Adapter** 플러그인으로, 경로 계획 후 생성된 궤적에 **시간 정보(타임스탬프)를 추가**하는 역할을 합니다.

- **파일 위치**: `moveit2/moveit_ros/planning/planning_request_adapter_plugins/src/add_time_parameterization.cpp`
- **플러그인 이름**: `default_planner_request_adapters/AddTimeParameterization`
- **저자**: Ioan Sucan (Willow Garage)

---

## 🎯 주요 기능

### 1. **Time Parameterization (시간 매개변수화)**

경로 계획기(OMPL, CHOMP 등)는 일반적으로 **기하학적 경로**(Geometric Path)만 생성합니다. 즉:
- 로봇이 어디로 가야 하는지(위치)는 알지만
- **언제, 얼마나 빠르게** 가야 하는지(속도, 가속도)는 모릅니다

이 플러그인은 다음을 수행합니다:
1. 기하학적 경로의 각 웨이포인트에 **타임스탬프 추가**
2. 각 관절의 **속도(velocity)** 계산
3. 각 관절의 **가속도(acceleration)** 계산
4. 속도/가속도 제한을 준수하면서 **부드러운 궤적** 생성

---

## 🔧 핵심 구현

### 클래스 구조

```cpp
class AddTimeParameterization : public planning_request_adapter::PlanningRequestAdapter
{
public:
  AddTimeParameterization();
  
  void initialize(const rclcpp::Node::SharedPtr& node, 
                  const std::string& parameter_namespace) override;
  
  std::string getDescription() const override;
  
  bool adaptAndPlan(const PlannerFn& planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override;

private:
  trajectory_processing::IterativeParabolicTimeParameterization time_param_;
};
```

### 동작 방식

#### 1. **adaptAndPlan() 메서드**
```cpp
bool adaptAndPlan(...) const override
{
  // 1. 먼저 플래너를 실행하여 기하학적 경로 생성
  bool result = planner(planning_scene, req, res);
  
  // 2. 계획이 성공하고 궤적이 있으면
  if (result && res.trajectory_)
  {
    // 3. 시간 매개변수화 수행
    if (!time_param_.computeTimeStamps(
          *res.trajectory_, 
          req.max_velocity_scaling_factor,      // 속도 스케일링 (0.0~1.0)
          req.max_acceleration_scaling_factor)) // 가속도 스케일링 (0.0~1.0)
    {
      RCLCPP_WARN(LOGGER, "Time parametrization for the solution path failed.");
      result = false;
    }
  }
  
  return result;
}
```

#### 2. **IterativeParabolicTimeParameterization**

- **알고리즘**: Iterative Parabolic Time Parameterization
- **특징**:
  - 관절의 속도/가속도 제한 준수
  - 부드러운(parabolic) 속도 프로파일 생성
  - 시작/종료 시 정지 상태 가정
  - **Jerk(가속도의 변화율) 제한 없음**

---

## 🔄 MoveIt2 Pipeline에서의 역할

### Planning Pipeline 구조

```
┌─────────────────────────────────────────────────────────────┐
│                     Planning Pipeline                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Planning Request Adapters (Pre-Processing)              │
│     ├─ FixWorkspaceBounds                                   │
│     ├─ FixStartStateBounds                                  │
│     ├─ FixStartStateCollision                               │
│     └─ FixStartStatePathConstraints                         │
│                                                              │
│  2. Motion Planner (OMPL/CHOMP/Pilz)                        │
│     └─ 기하학적 경로 생성 (without time)                     │
│                                                              │
│  3. Planning Request Adapters (Post-Processing)             │
│     ├─ ResolveConstraintFrames                              │
│     └─ AddTimeParameterization  ◄─── 이 파일!               │
│         └─ 시간 정보 추가 (with velocities & timestamps)     │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### move_group에서의 사용

#### 1. **설정 파일 (ompl_planning.yaml)**
```yaml
# moveit_configs_utils/default_configs/ompl_planning.yaml
planning_plugin: ompl_interface/OMPLPlanner
start_state_max_bounds_error: 0.1
jiggle_fraction: 0.05
request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/ResolveConstraintFrames
    default_planner_request_adapters/FixWorkspaceBounds
    default_planner_request_adapters/FixStartStateBounds
    default_planner_request_adapters/FixStartStateCollision
    default_planner_request_adapters/FixStartStatePathConstraints
```

#### 2. **플러그인 등록 (XML)**
```xml
<!-- planning_request_adapters_plugin_description.xml -->
<library path="moveit_default_planning_request_adapter_plugins">
  <class name="default_planner_request_adapters/AddTimeParameterization" 
         type="default_planner_request_adapters::AddTimeParameterization" 
         base_class_type="planning_request_adapter::PlanningRequestAdapter">
    <description>
      Adds time parameterization using iterative parabolic algorithm
    </description>
  </class>
</library>
```

#### 3. **동적 로딩 과정**
```cpp
// planning_pipeline.cpp에서
void PlanningPipeline::configure()
{
  // 1. 어댑터 플러그인 로더 생성
  adapter_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<
      planning_request_adapter::PlanningRequestAdapter>>(
          "moveit_core", "planning_request_adapter::PlanningRequestAdapter");
  
  // 2. 설정 파일에서 지정된 어댑터들을 로드
  for (const std::string& adapter_plugin_name : adapter_plugin_names_)
  {
    auto ad = adapter_plugin_loader_->createUniqueInstance(adapter_plugin_name);
    ad->initialize(node_, parameter_namespace_);
    adapter_chain_->addAdapter(ad);
    RCLCPP_INFO(LOGGER, "Using planning request adapter '%s'", 
                ad->getDescription().c_str());
  }
}
```

---

## 📊 실행 흐름

### 전체 프로세스

```
사용자가 move_group.move() 호출
         ↓
┌────────────────────────────────────┐
│   MoveGroupInterface               │
│   - setJointValueTarget()          │
│   - move()                          │
└────────────────────────────────────┘
         ↓
┌────────────────────────────────────┐
│   MoveGroup Action Server          │
│   (MoveGroupMoveAction)             │
└────────────────────────────────────┘
         ↓
┌────────────────────────────────────┐
│   Planning Pipeline                │
│                                     │
│   1. Pre-processing adapters       │
│      ├─ FixWorkspaceBounds         │
│      ├─ FixStartStateBounds        │
│      ├─ FixStartStateCollision     │
│      └─ FixStartStatePathConstraints│
│                                     │
│   2. Motion Planner (OMPL)         │
│      └─ RRTConnect/RRT*/PRM 등     │
│         출력: 경로 웨이포인트       │
│         (위치만, 속도/시간 없음)    │
│                                     │
│   3. Post-processing adapters      │
│      ├─ ResolveConstraintFrames    │
│      └─ AddTimeParameterization ◄──┼─ 여기!
│          입력: 기하학적 경로        │
│          처리: computeTimeStamps()  │
│          출력: 시간 정보가 추가된   │
│                완전한 궤적           │
│                                     │
└────────────────────────────────────┘
         ↓
┌────────────────────────────────────┐
│   Trajectory Execution Manager     │
│   - 궤적을 컨트롤러에 전송          │
└────────────────────────────────────┘
         ↓
┌────────────────────────────────────┐
│   Robot Controller                 │
│   - 로봇 실제 이동                  │
└────────────────────────────────────┘
```

### computeTimeStamps() 내부 동작

```cpp
// 의사 코드
bool computeTimeStamps(RobotTrajectory& trajectory, 
                       double velocity_scaling_factor,
                       double acceleration_scaling_factor)
{
  // 1. 각 관절의 속도/가속도 한계 가져오기
  for (각 관절 in 로봇) {
    max_velocity[joint] = joint_limits[joint].max_velocity * velocity_scaling_factor;
    max_acceleration[joint] = joint_limits[joint].max_acceleration * acceleration_scaling_factor;
  }
  
  // 2. 각 웨이포인트 구간에 대해
  for (int i = 0; i < waypoints.size() - 1; i++) {
    // a. 구간의 거리 계산
    distance = calculateDistance(waypoints[i], waypoints[i+1]);
    
    // b. 속도/가속도 제한을 고려한 최소 시간 계산
    min_time = calculateMinTime(distance, max_velocity, max_acceleration);
    
    // c. 타임스탬프 할당
    waypoints[i+1].time = waypoints[i].time + min_time;
    
    // d. 속도 계산 (포물선 프로파일)
    velocity = calculateParabolicVelocity(waypoints[i], waypoints[i+1], min_time);
  }
  
  // 3. 계산된 궤적 유효성 검사
  return validateTrajectory(trajectory);
}
```

---

## 🆚 다른 Time Parameterization 알고리즘 비교

MoveIt2는 여러 시간 매개변수화 알고리즘을 제공합니다:

| 알고리즘 | 파일 | 특징 | 장점 | 단점 |
|---------|------|------|-----|-----|
| **IterativeParabolic** | add_time_parameterization.cpp | 포물선 속도 프로파일 | 구현 간단, 빠름 | Jerk 제한 없음 |
| **TimeOptimal** | add_time_optimal_parameterization.cpp | 시간 최적화 | 최단 시간 달성 | Jerk 제한 없음 |
| **IterativeSpline** | add_iterative_spline_parameterization.cpp | 스플라인 보간 | 부드러운 궤적 | 느림 |
| **Ruckig** | add_ruckig_traj_smoothing.cpp | Jerk 제한 평활화 | Jerk 제한 준수 | 추가 의존성 |

### 일반적인 사용 패턴

```yaml
# 기본 설정 (빠르지만 부드럽지 않음)
request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization

# 권장 설정 (시간 최적 + Jerk 제한)
request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/AddRuckigTrajectorySmoothing
```

---

## 🔍 실제 사용 예시

### C++ 코드에서

```cpp
// MoveGroupInterface를 통한 사용 (자동 적용)
moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

// 속도/가속도 스케일링 설정
move_group.setMaxVelocityScalingFactor(0.5);      // 50% 속도
move_group.setMaxAccelerationScalingFactor(0.3);  // 30% 가속도

// move() 호출 시 자동으로 AddTimeParameterization 적용됨
move_group.setJointValueTarget(target_positions);
auto result = move_group.move();

// 내부에서 다음이 일어남:
// 1. OMPL이 기하학적 경로 생성
// 2. AddTimeParameterization이 시간 정보 추가
// 3. 완전한 궤적을 컨트롤러에 전송
```

### Planning Pipeline 직접 사용

```cpp
// 더 세밀한 제어가 필요한 경우
planning_pipeline::PlanningPipeline pipeline(
    node, robot_model, 
    "ompl",  // 플래너
    {"default_planner_request_adapters/AddTimeParameterization"}  // 어댑터
);

planning_interface::MotionPlanRequest req;
req.max_velocity_scaling_factor = 0.5;
req.max_acceleration_scaling_factor = 0.3;

planning_interface::MotionPlanResponse res;
bool success = pipeline.generatePlan(planning_scene, req, res);

if (success && res.trajectory_) {
    // res.trajectory_에는 이제 시간 정보가 포함됨
    auto& trajectory = res.trajectory_;
    
    for (size_t i = 0; i < trajectory->getWayPointCount(); ++i) {
        auto waypoint = trajectory->getWayPoint(i);
        double time = trajectory->getWayPointDurationFromStart(i);
        
        std::cout << "Waypoint " << i << " at time " << time << "s\n";
    }
}
```

---

## ⚙️ 설정 파라미터

### 주요 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `max_velocity_scaling_factor` | double | 1.0 | 최대 속도 스케일링 (0.0~1.0) |
| `max_acceleration_scaling_factor` | double | 1.0 | 최대 가속도 스케일링 (0.0~1.0) |

### 관절 제한 정의 (joint_limits.yaml)

```yaml
joint_limits:
  panda_joint1:
    has_velocity_limits: true
    max_velocity: 2.1750  # rad/s
    has_acceleration_limits: true
    max_acceleration: 15.0  # rad/s²
  panda_joint2:
    has_velocity_limits: true
    max_velocity: 2.1750
    has_acceleration_limits: true
    max_acceleration: 7.5
```

---

## 🐛 문제 해결

### 일반적인 에러

#### 1. "Time parametrization for the solution path failed"
```
원인: 관절 제한을 만족하는 시간 매개변수화 불가능
해결:
  - max_velocity_scaling_factor 감소
  - max_acceleration_scaling_factor 감소
  - 경로의 웨이포인트 간격이 너무 가까운지 확인
```

#### 2. 로봇이 너무 느리게 움직임
```
원인: 스케일링 팩터가 너무 작음
해결:
  move_group.setMaxVelocityScalingFactor(0.8);  // 증가
  move_group.setMaxAccelerationScalingFactor(0.8);
```

#### 3. 로봇이 너무 빠르고 떨림
```
원인: Jerk 제한이 없음
해결:
  - Ruckig 평활화 추가
  request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/AddRuckigTrajectorySmoothing
```

---

## 📚 관련 파일

### 핵심 파일
- **현재 파일**: `planning_request_adapter_plugins/src/add_time_parameterization.cpp`
- **알고리즘 구현**: `moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp`
- **베이스 클래스**: `moveit_core/planning_request_adapter/include/planning_request_adapter.h`

### 설정 파일
- `moveit_configs_utils/default_configs/ompl_planning.yaml`
- `planning_request_adapters_plugin_description.xml`

### 관련 코드
- `planning_pipeline/src/planning_pipeline.cpp` - 파이프라인 실행
- `move_group/src/move_group.cpp` - move_group 노드

---

## 📖 참고 자료

### 논문 및 문서
- **MoveIt2 Documentation**: https://moveit.picknik.ai/
- **Planning Request Adapters**: https://moveit.picknik.ai/main/doc/examples/planning_adapters/planning_adapters_tutorial.html
- **Time Parameterization**: Trajectory time parameterization 관련 논문

### 유용한 링크
- [MoveIt2 GitHub](https://github.com/moveit/moveit2)
- [Planning Pipeline Tutorial](https://moveit.picknik.ai/main/doc/examples/planning_pipeline/planning_pipeline_tutorial.html)

---

## 💡 핵심 요약

1. **역할**: 기하학적 경로에 시간 정보(속도, 가속도, 타임스탬프) 추가
2. **위치**: Planning Pipeline의 후처리(post-processing) 단계
3. **알고리즘**: Iterative Parabolic Time Parameterization
4. **자동 실행**: move_group 사용 시 자동으로 적용됨
5. **제어 가능**: velocity/acceleration scaling factor로 속도 조절
6. **한계**: Jerk 제한 없음 (Ruckig와 조합하여 해결 가능)

---

*이 문서는 MoveIt2의 `add_time_parameterization.cpp` 파일에 대한 상세 설명입니다.*
*최종 수정일: 2025년 12월 19일*
