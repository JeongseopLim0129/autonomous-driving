import rclpy
from rclpy.node import Node
import numpy as np
import math
import copy
import time
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from my_autonomous_pkg.hybrid_a_star import HybridAStarPlanner
from my_autonomous_pkg.nmpc import NMPCController

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        
        # 맵 관련 변수 초기화 (원점, 해상도, 크기)
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0

        # 구독(Subscription) 설정
        map_qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos_profile) # 맵 구독
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 10) # 초기 위치 (Rviz 2D Pose Estimate)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10) # 목표 위치 (Rviz 2D Goal Pose)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10) # 라이다 센서 데이터
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10) # 로봇의 현재 위치 (AMCL 등)

        # 발행(Publisher) 설정
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 로봇 속도 명령 발행
        self.path_pub = self.create_publisher(Path, '/global_plan', 10) # 생성된 전역 경로 시각화용 발행

        # 내부 상태 변수 설정
        self.static_map_data = None   # 정적 맵 데이터 (변경되지 않는 원본)
        self.current_map_data = None  # 동적 장애물이 반영될 현재 맵 데이터
        self.robot_pose = None        # 로봇의 현재 위치 [x, y, yaw]
        self.scan_data = []           # 라이다로 감지된 장애물 좌표 리스트
        self.global_path = None       # 생성된 전역 경로
        self.current_goal = None      # 현재 목표 지점

        # 상태 머신 및 고정 변수들 (회피 기동용)
        self.recovery_state = 'IDLE'  # 현재 상태 (IDLE: 일반 주행, BACKING: 후진 조준, TURNING: 제자리 회전)
        self.recovery_start_time = 0.0 # 회피 기동 시작 시간 기록
        self.locked_turn_dir = 0.0   # 회전 방향 고정 (+1: 좌, -1: 우) - 잦은 방향 전환 방지
        self.fixed_target_yaw = 0.0  # 목표 각도 고정 (스냅샷)

        # 경로 계획기 및 제어기 객체 생성
        self.planner = HybridAStarPlanner() # 전역 경로 생성기 (Hybrid A*)
        self.controller = NMPCController()  # 지역 경로 추종기 (NMPC)

        # 타이머 설정
        self.create_timer(0.05, self.control_loop) # 0.05초(20Hz)마다 제어 루프 실행
        self.create_timer(0.2, self.replan_loop)   # 0.2초(5Hz)마다 경로 재계산 판단
        
        self.get_logger().info("방향 고정(Direction Lock) 회피 노드 준비 완료")

    def map_callback(self, msg):
        # 맵 메타데이터 저장
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        # 1차원 배열인 맵 데이터를 2차원 배열로 변환
        raw_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        # 맵의 x, y 축을 맞추기 위해 전치
        self.static_map_data = raw_data.T
        self.current_map_data = copy.deepcopy(self.static_map_data) # 초기화

    def pose_callback(self, msg):
        # 쿼터니언을 오일러 각으로 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # 로봇 위치 업데이트 [x, y, yaw]
        self.robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

    def init_pose_callback(self, msg):
        # 초기 위치 설정도 pose_callback과 동일하게 처리
        self.pose_callback(msg)

    def scan_callback(self, msg):
        # 로봇 위치가 없으면 라이다 데이터를 처리할 수 없음
        if self.robot_pose is None: return
        
        obstacles = []
        angle = msg.angle_min # 스캔 시작 각도
        rx, ry, rtheta = self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]
        lidar_offset_x = -0.064 # 로봇 중심 대비 라이다 설치 위치
        
        # 모든 라이다 포인트에 대해 반복
        for r in msg.ranges:
            # 유효하지 않은 거리 값 필터링
            if r < msg.range_min or r > msg.range_max or r == 0.0:
                angle += msg.angle_increment
                continue
            
            # 3.5m 이내의 장애물만 고려
            if r < 3.5: 
                # 라이다 기준 좌표 (극좌표 -> 직교좌표)
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                
                # 로봇 기준 좌표 -> 월드 좌표 변환 (회전 행렬 적용)
                mx = rx + (lidar_offset_x * math.cos(rtheta)) + (ox * math.cos(rtheta) - oy * math.sin(rtheta))
                my = ry + (lidar_offset_x * math.sin(rtheta)) + (ox * math.sin(rtheta) + oy * math.cos(rtheta))
                obstacles.append([mx, my])
            angle += msg.angle_increment # 다음 각도로 증가
            
        self.scan_data = np.array(obstacles) # numpy 배열로 저장

    def update_map_with_scan(self):
        # 정적 맵이 없거나 스캔 데이터가 없으면 리턴
        if self.static_map_data is None or len(self.scan_data) == 0: return
        
        # 현재 맵을 정적 맵으로 초기화 (이전 스캔 잔상 제거)
        self.current_map_data = np.copy(self.static_map_data)
        
        # 스캔 데이터(월드 좌표)를 그리드 인덱스로 변환
        ix = ((self.scan_data[:, 0] - self.map_origin_x) / self.map_resolution).astype(int)
        iy = ((self.scan_data[:, 1] - self.map_origin_y) / self.map_resolution).astype(int)
        
        # 맵 범위를 벗어나는 인덱스 제거
        valid_mask = (ix >= 0) & (ix < self.map_width) & (iy >= 0) & (iy < self.map_height)
        ix = ix[valid_mask]
        iy = iy[valid_mask]
        
        # 장애물 팽창: 점 하나가 아니라 주변 픽셀도 장애물로 표시
        margin = 3 # 주변 3칸
        offsets = np.arange(-margin, margin + 1)
        for dx in offsets:
            for dy in offsets:
                nx = ix + dx
                ny = iy + dy
                # 맵 범위 내에 있는지 확인
                valid = (nx >= 0) & (nx < self.map_width) & (ny >= 0) & (ny < self.map_height)
                # 해당 위치를 장애물(100)로 설정
                self.current_map_data[nx[valid], ny[valid]] = 100

    def goal_callback(self, msg):
        # 새로운 목표가 들어오면 저장하고 상태 초기화
        self.current_goal = [msg.pose.position.x, msg.pose.position.y, 0.0]
        self.recovery_state = 'IDLE' # 회피 모드 해제
        self.perform_planning() # 경로 계획 실행

    def replan_loop(self):
        # 회피 기동 중일 때는 경로를 재계산하지 않음
        if self.recovery_state != 'IDLE': return

        # 목표가 있고 로봇 위치가 있으면 주기적으로 재계산 시도
        if self.current_goal is not None and self.robot_pose is not None:
            dist = math.hypot(self.current_goal[0] - self.robot_pose[0], 
                              self.current_goal[1] - self.robot_pose[1])
            # 목표 지점에 거의 다다랐을 때는 굳이 재계산하지 않음 (오실레이션 방지)
            if dist > 0.5: self.perform_planning()

    def perform_planning(self):
        # 필수 데이터가 없으면 리턴
        if self.static_map_data is None or self.robot_pose is None or self.current_goal is None: return
        
        self.update_map_with_scan() # 맵에 최신 장애물 반영
        
        # 월드 좌표를 맵 관련 좌표로 변환
        start_node = [(self.robot_pose[0] - self.map_origin_x), (self.robot_pose[1] - self.map_origin_y), self.robot_pose[2]]
        goal_node = [(self.current_goal[0] - self.map_origin_x), (self.current_goal[1] - self.map_origin_y), 0.0]
        
        # Hybrid A* 알고리즘 실행
        path_in_map = self.planner.plan(start_node, goal_node, self.current_map_data)
        
        if path_in_map:
            self.controller.reset() # 경로가 새로 생겼으므로 제어기 상태 리셋
            final_path = []
            # 맵 좌표를 다시 월드 좌표로 변환
            for point in path_in_map:
                final_path.append([point[0] + self.map_origin_x, point[1] + self.map_origin_y])
            self.global_path = final_path # 전역 경로 갱신
        else:
            self.global_path = None # 경로 생성 실패

    def get_path_segment(self):
        # 전역 경로 중 로봇 근처의 일부 구간만 잘라내는 함수
        if self.global_path is None: return []
        
        # 로봇과 가장 가까운 경로점 찾기
        min_idx = -1
        min_dist = float('inf')
        for i, point in enumerate(self.global_path):
            dist = math.hypot(point[0] - self.robot_pose[0], point[1] - self.robot_pose[1])
            if dist < min_dist:
                min_dist = dist
                min_idx = i
                
        # 가까운 점부터 앞쪽 15개 점을 추출
        lookahead = 15
        segment = []
        for i in range(min_idx, min(min_idx+lookahead, len(self.global_path))):
            segment.append(self.global_path[i])
            
        # 경로 끝부분 처리
        while len(segment) < lookahead:
            if len(segment)>0: segment.append(segment[-1])
            else: break
        return segment

    def get_local_goal(self):
        # Pure Pursuit처럼 로봇보다 일정 거리 앞에 있는 경로점을 찾음
        if self.global_path is None: return None
        search_dist = 0.8 # 전방 0.8m 지점을 찾음
        
        # 가장 가까운 인덱스 찾기
        min_idx = -1
        min_dist = float('inf')
        for i, point in enumerate(self.global_path):
            dist = math.hypot(point[0] - self.robot_pose[0], point[1] - self.robot_pose[1])
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        
        # 경로를 따라가며 거리를 누적하여 0.8m 떨어진 점 반환
        curr_dist = 0
        for i in range(min_idx, len(self.global_path)-1):
            curr_dist += math.hypot(self.global_path[i+1][0]-self.global_path[i][0], self.global_path[i+1][1]-self.global_path[i][1])
            if curr_dist > search_dist: return self.global_path[i+1]
        
        return self.global_path[-1] # 못 찾으면 마지막 점 반환

    def start_turning_mode(self):
        """회전 모드 진입 시 딱 한 번 호출되어 방향과 목표를 고정함 (이 코드에서는 직접 호출 안 됨)"""
        local_goal = self.get_local_goal()
        if local_goal is None: return
        
        # 1. 목표 각도 스냅샷 - 현재 목표점 방향 계산
        target_yaw = math.atan2(local_goal[1]-self.robot_pose[1], local_goal[0]-self.robot_pose[0])
        self.fixed_target_yaw = target_yaw
        
        # 2. 현재 각도와의 차이 계산
        curr_yaw = self.robot_pose[2]
        yaw_diff = target_yaw - curr_yaw
        # 각도 정규화 (-PI ~ PI)
        while yaw_diff > math.pi: yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi: yaw_diff += 2*math.pi
        
        # 3. 회전 방향 결정 및 고정 (Lock)
        self.locked_turn_dir = 1.0 if yaw_diff > 0 else -1.0
        
        self.recovery_state = 'TURNING'
        self.get_logger().info(f"회전 시작: 방향({self.locked_turn_dir}), 목표각({target_yaw:.2f})")

    def control_loop(self):
        if self.robot_pose is None: return
        # 경로가 없으면 정지
        if self.global_path is None:
            if self.current_goal: self.cmd_pub.publish(Twist())
            return

        # 1. 도착 판정: 최종 목표점과 20cm 이내면 정지
        if math.hypot(self.global_path[-1][0]-self.robot_pose[0], self.global_path[-1][1]-self.robot_pose[1]) < 0.2:
            self.get_logger().info("도착!")
            self.global_path = None
            self.cmd_pub.publish(Twist()) # 정지 명령
            self.recovery_state = 'IDLE'
            return

        # 데이터 계산
        local_goal = self.get_local_goal() # 로컬 목표점 가져오기
        curr_yaw = self.robot_pose[2]      # 현재 로봇 각도
        # 목표 방향 계산
        target_yaw = math.atan2(local_goal[1]-self.robot_pose[1], local_goal[0]-self.robot_pose[0])
        
        # 각도 차이(에러) 계산 및 정규화
        yaw_diff = target_yaw - curr_yaw
        while yaw_diff > math.pi: yaw_diff -= 2*math.pi
        while yaw_diff < -math.pi: yaw_diff += 2*math.pi

        # 장애물 거리 초기화
        min_front_dist = float('inf')
        min_rear_dist = float('inf')

        # 라이다 데이터를 이용해 전방/후방 장애물 거리 계산
        if len(self.scan_data) > 0:
            dx = self.scan_data[:,0] - self.robot_pose[0]
            dy = self.scan_data[:,1] - self.robot_pose[1]
            dists = np.hypot(dx, dy)
            # 로봇 기준 장애물 각도 계산
            angles = np.arctan2(dy, dx) - curr_yaw
            angles = (angles + np.pi) % (2 * np.pi) - np.pi
            
            # 전방: ±45도 이내
            front_mask = np.abs(angles) < np.deg2rad(45)
            # 후방: 115도(~2.0rad) 이상
            rear_mask = np.abs(angles) > 2.0 
            
            if np.any(front_mask): min_front_dist = np.min(dists[front_mask])
            if np.any(rear_mask): min_rear_dist = np.min(dists[rear_mask])
        
        # 1. 후진 모드 실행 중일 때
        if self.recovery_state == 'BACKING':
            # [탈출 조건] - 언제 후진을 멈출 것인가?
            # A. 경로가 내 앞쪽 60도(1.0 rad) 안으로 들어왔는가? (각도 확보 완료)
            # B. 또는 앞 공간이 충분히 넓어져서(60cm) NMPC가 움직일 공간이 생겼는가?
            # C. 뒤가 막혔거나(20cm), 후진 시간이 너무 길어졌는가(3초)?
            
            cond_aimed = abs(yaw_diff) < 1.0  # 각도 확보 여부
            cond_space = min_front_dist > 0.60 # 앞 공간 확보 여부
            cond_blocked = min_rear_dist < 0.20 # 뒤쪽 막힘 여부
            cond_timeout = (time.time() - self.recovery_start_time > 3.0) # 시간 초과
            
            # 탈출 조건 중 하나라도 만족하면 IDLE로 복귀
            if cond_aimed or cond_space or cond_blocked or cond_timeout:
                self.get_logger().info(f"후진 종료 (조준됨:{cond_aimed}, 공간:{cond_space}) -> NMPC 복귀")
                self.recovery_state = 'IDLE'
                self.controller.reset() # NMPC 리셋
                
                # 복귀 직후 NMPC가 바로 멈추거나 버벅대지 않게 살짝 전진 명령 부여
                cmd = Twist()
                cmd.linear.x = 0.05
                self.cmd_pub.publish(cmd)
            else:
                # [곡선 후진] 경로 방향에 맞춰 엉덩이를 트는 로직
                # 경로가 왼쪽에 있으면(yaw_diff > 0), 머리를 왼쪽으로 보내야 하므로 -> 엉덩이는 오른쪽으로 틀어야 함
                # 후진 시 각속도 부호는 전진과 반대 효과를 냄을 이용
                
                cmd = Twist()
                cmd.linear.x = -0.15 # 0.15 m/s로 후진
                
                # yaw_diff 부호에 따라 회전 방향 결정
                # 부호가 양수(왼쪽 목표)일 때 -각속도를 주면 후진하면서 머리가 왼쪽으로 돌아감
                cmd.angular.z = -0.5 * np.sign(yaw_diff) 
                
                self.cmd_pub.publish(cmd)
            return # 후진 중에는 아래 NMPC 실행 안 함

        # 2. 평상시 (IDLE) - 상황 감시 및 NMPC 주행
        if self.recovery_state == 'IDLE':
            # 진입 조건 체크: 앞이 막혔고(30cm) + 경로 각도가 너무 큼(60도 이상)
            # 좁은 곳에서 각도가 안 맞으면 NMPC가 힘들어하므로 미리 후진 모드로 전환
            is_stuck = (min_front_dist < 0.30) and (abs(yaw_diff) > 1.0)
            is_emergency = min_front_dist < 0.15 # 너무 가까우면 비상
            
            if is_stuck or is_emergency:
                # 뒤쪽 공간이 있다면 후진 시작
                if min_rear_dist > 0.20:
                    self.get_logger().warn("각도 확보를 위한 곡선 후진 시작")
                    self.recovery_state = 'BACKING'
                    self.recovery_start_time = time.time()
                    return
                else:
                    # 뒤도 막혔으면(오도가도 못함) 여기서는 별도 처리 없이 NMPC에 맡기거나 정지
                    pass 

            # NMPC 실행 (경로 추종 제어)
            path_seg = self.get_path_segment() # 전방 경로 조각 가져오기
            obs = self.scan_data if len(self.scan_data)>0 else np.array([]) # 장애물 데이터
            
            # NMPC 컨트롤러로 최적의 속도(v)와 회전각속도(w) 계산
            v, w = self.controller.compute_control(self.robot_pose, path_seg, obs)
            
            # 계산된 제어 입력 발행
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()