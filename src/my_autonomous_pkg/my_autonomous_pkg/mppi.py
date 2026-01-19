import numpy as np
import math

class MPPIController:
    def __init__(self):
        # 1. 로봇 스펙 설정
        self.max_v = 0.22   # 최대 전진 속도
        
        # 최소 속도를 음수로 설정하여 로봇이 막혔을 때 스스로 후진할 수 있게 함
        self.min_v = -0.15  
        
        self.max_w = 1.5    # 최대 회전 속도
        
        # 2. MPPI 알고리즘 파라미터
        self.horizon = 12       # 예측 구간: 미래 12스텝 앞을 내다봄
        self.dt = 0.1           # 시간 간격: 1스텝당 0.1초 (즉, 총 1.2초 예측)
        self.num_samples = 80   # 샘플 개수: 매번 80개의 가상 경로를 생성해 비교
        
        # 3. 비용 함수 가중치
        self.w_track = 1.5      # 경로 추종 가중치
        self.w_smooth = 0.2     # 부드러운 주행 가중치
        self.w_obs = 100.0      # 장애물 회피 가중치
        
        # 4. 로봇 풋프린트
        # Waffle Pi의 실제 크기를 고려하여 직사각형 형태의 점 8개를 정의
        # 중심을 기준으로 로봇의 모서리와 변의 중점 좌표들 [x, y]
        self.footprint = np.array([
            [ 0.15,  0.15], [ 0.15, -0.15], # 앞쪽 좌/우 모서리
            [-0.15, -0.15], [-0.15,  0.15], # 뒤쪽 우/좌 모서리
            [ 0.15,  0.0 ], [-0.15,  0.0 ], # 앞/뒤 변의 중점
            [ 0.0 ,  0.15], [ 0.0 , -0.15]  # 좌/우 변의 중점
        ])

        # 이전 제어 입력 저장소
        # 초기에는 0으로 채워진 (Horizon x 2) 배열 생성
        self.prev_controls = np.zeros((self.horizon, 2))

    def reset(self):
        # 제어기를 리셋할 때 호출
        self.prev_controls = np.zeros((self.horizon, 2))

    def compute_control(self, robot_pose, ref_path, obstacles):
        """
        MPPI 알고리즘을 사용하여 현재 상태에서 최적의 속도(v, w)를 계산하는 핵심 함수
        input: robot_pose(현위치), ref_path(따라갈 경로), obstacles(장애물 좌표들)
        """
        
        # ---------------------------------------------------------
        # 1. 샘플링: 다양한 가상 제어 입력 생성
        # ---------------------------------------------------------
        
        # 평균 0, 표준편차 0.1인 정규분포 노이즈 생성 (선속도용) -> (80개 샘플 x 12스텝)
        noise_v = np.random.normal(0, 0.1, (self.num_samples, self.horizon))
        # 평균 0, 표준편차 0.8인 정규분포 노이즈 생성 (각속도용) -> 회전은 변동폭을 크게 줌
        noise_w = np.random.normal(0, 0.8, (self.num_samples, self.horizon))
        
        # 이전 주기에서 계산했던 최적 제어 입력을 가져옴
        base_v = self.prev_controls[:, 0]
        base_w = self.prev_controls[:, 1]
        
        # 기준 값에 노이즈를 더해 새로운 후보군 생성
        # np.clip을 사용하여 로봇 하드웨어 한계(속도/각속도 제한)를 넘지 않도록 자름
        samples_v = np.clip(base_v + noise_v, self.min_v, self.max_v)
        samples_w = np.clip(base_w + noise_w, -self.max_w, self.max_w)

        # 2. 궤적 예측 및 비용 계산
        
        # 각 샘플별 총 비용을 저장할 배열 초기화
        costs = np.zeros(self.num_samples)
        
        # 모든 샘플의 초기 상태를 현재 로봇 위치로 설정
        curr_x = np.full(self.num_samples, robot_pose[0])
        curr_y = np.full(self.num_samples, robot_pose[1])
        curr_th = np.full(self.num_samples, robot_pose[2])

        # 장애물 필터링: 로봇 주변 1.0m 이내의 장애물만 추림
        # 전체 장애물을 다 검사하면 연산이 너무 느려지므로 가까운 것만 봄
        local_obs = []
        if len(obstacles) > 0:
            dx = obstacles[:, 0] - robot_pose[0]
            dy = obstacles[:, 1] - robot_pose[1]
            dists = np.hypot(dx, dy)
            local_obs = obstacles[dists < 1.0]

        # 참조 경로를 numpy 배열로 변환
        ref_path = np.array(ref_path)

        # 미래의 시간 스텝만큼 반복하며 시뮬레이션
        for t in range(self.horizon):
            # t번째 스텝의 속도/각속도 명령
            v = samples_v[:, t]
            w = samples_w[:, t]

            # [Motion Model] 로봇 이동 방정식
            curr_x += v * np.cos(curr_th) * self.dt # X 좌표 업데이트
            curr_y += v * np.sin(curr_th) * self.dt # Y 좌표 업데이트
            curr_th += w * self.dt                  # 각도 업데이트

            # [비용 A] 경로 추종 비용 (Tracking Cost)
            if len(ref_path) > 0:
                # 현재 예측 시점(t)과 매칭되는 경로점 인덱스 선택
                # 경로가 짧을 경우 마지막 점을 계속 목표로 삼음
                target_idx = min(t, len(ref_path) - 1)
                target = ref_path[target_idx]
                
                # 예측된 위치와 목표점 사이의 거리 계산
                dist_track = np.hypot(curr_x - target[0], curr_y - target[1])
                # 거리에 가중치를 곱해 비용에 누적
                costs += self.w_track * dist_track

        # [비용 B] 장애물 충돌 비용 (Collision Cost)
        # 여기서는 정확한 충돌 체크를 위해 벡터 연산 대신 루프를 사용하여 정밀 검사
        if len(local_obs) > 0:
            obs_x = local_obs[:, 0]
            obs_y = local_obs[:, 1]
            
            # 각 샘플마다 개별적으로 검사
            for k in range(self.num_samples):
                # 궤적을 다시 시뮬레이션하기 위해 초기 위치 리셋
                cx, cy, cth = robot_pose
                min_dist_to_obs = 100.0 # 장애물과의 최소 거리 기록용
                
                # 속도 최적화 Horizon의 모든 스텝을 검사하지 않고 3스텝 건너뛰며 검사 (0, 3, 6, 9...)
                for t in range(0, self.horizon, 3):
                    # 해당 샘플의 t번째 제어 입력
                    step_v = samples_v[k, t]
                    step_w = samples_w[k, t]
                    
                    # 0.3초만큼 한 번에 이동했다고 가정
                    cx += step_v * np.cos(cth) * self.dt * 3 
                    cy += step_v * np.sin(cth) * self.dt * 3
                    cth += step_w * self.dt * 3
                    
                    # 로봇 풋프린트를 현재 로봇 각도만큼 회전 변환하기 위한 행렬
                    cos_th = math.cos(cth)
                    sin_th = math.sin(cth)
                    rot_mat = np.array([[cos_th, -sin_th], [sin_th, cos_th]])
                    
                    # 풋프린트 회전 적용 후 현재 위치(cx, cy) 더해서 월드 좌표로 변환
                    rotated_foot = (rot_mat @ self.footprint.T).T
                    global_foot = rotated_foot + np.array([cx, cy])
                    
                    # 변환된 풋프린트 점들과 모든 지역 장애물 간의 거리 계산
                    for fp in global_foot:
                        dists = np.hypot(fp[0] - obs_x, fp[1] - obs_y) # 장애물들과의 거리 배열
                        min_d = np.min(dists) # 가장 가까운 장애물 거리
                        if min_d < min_dist_to_obs:
                            min_dist_to_obs = min_d

                # 최소 거리에 따른 비용 부과
                if min_dist_to_obs < 0.05:
                    # 5cm 이내면 충돌로 간주 -> 엄청 큰 비용 부과 (이 경로는 선택 안 됨)
                    costs[k] += 10000.0 
                elif min_dist_to_obs < 0.3:
                    # 30cm 이내면 위험 구역 -> 거리에 반비례하여 비용 증가 (가까울수록 비용 높음)
                    costs[k] += self.w_obs * (0.3 - min_dist_to_obs)

        # 3. 최적 제어 입력 선택 (Softmax Aggregation)
        
        min_cost = np.min(costs) # 가장 낮은 비용 찾기
        
        # 모든 샘플이 충돌 등으로 비용이 너무 크면, 안전을 위해 정지 명령 반환
        if min_cost > 5000: 
            return 0.0, 0.0

        # Softmax 연산: 비용이 낮을수록 높은 확률(가중치)을 갖도록 변환
        # -5.0은 Temperature 파라미터로, 클수록 좋은 샘플에 더 집중함
        # (costs - min_cost)를 하는 이유는 오버플로우 방지 및 상대적 차이 강조
        exp_costs = np.exp(-5.0 * (costs - min_cost)) 
        
        # 확률의 합이 1이 되도록 정규화 (분모가 0이 되는 것을 막기 위해 1e-6 더함)
        sum_exp = np.sum(exp_costs) + 1e-6
        weights = exp_costs / sum_exp

        # 가중 평균 계산: 각 샘플의 입력값에 가중치를 곱해 합산
        # 비용이 낮은(좋은) 샘플의 입력값이 결과에 많이 반영됨
        final_v = np.sum(weights[:, None] * samples_v, axis=0)
        final_w = np.sum(weights[:, None] * samples_w, axis=0)

        # 4. Warm Start 업데이트
        
        # 이번에 계산된 최적 제어 시퀀스를 다음 주기의 초기값으로 사용하기 위해 저장
        # np.roll로 배열을 한 칸 앞으로 당김
        self.prev_controls = np.roll(np.vstack((final_v, final_w)).T, -1, axis=0)
        
        # 마지막 칸은 비어있으므로 바로 전 단계 값으로 채움
        self.prev_controls[-1] = self.prev_controls[-2]

        # 현재 시점(t=0)에 해당하는 제어 입력(v, w) 반환
        return final_v[0], final_w[0]