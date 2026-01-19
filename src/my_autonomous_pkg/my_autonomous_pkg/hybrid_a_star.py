import numpy as np
import math
import heapq

class HybridAStarPlanner:
    def __init__(self, resolution=0.1, grid_width=100, grid_height=100):
        self.reso = resolution  # 맵의 해상도
        
        # 로봇의 이동 가능한 조향 각도 설정 (좌회전, 직진, 우회전)
        self.steer_set = [-20, 0, 20]  # 조향 각도 후보군 (도 단위 degree)
        
        self.step_size = 0.5           # 한 스텝당 이동하는 거리 (미터)
        self.grid_w = grid_width       # 그리드 맵의 전체 가로 폭 (셀 개수)
        self.grid_h = grid_height      # 그리드 맵의 전체 세로 높이 (셀 개수)

    class Node:
        # 경로 탐색 트리의 각 노드를 정의하는 내부 클래스
        def __init__(self, x_ind, y_ind, theta_ind, parent, cost, x, y, theta):
            self.x_ind = x_ind          # 그리드 상의 X 인덱스 (정수)
            self.y_ind = y_ind          # 그리드 상의 Y 인덱스 (정수)
            self.theta_ind = theta_ind  # 각도 인덱스 (방문 여부 체크용)
            self.parent = parent        # 현재 노드에 도달하기 직전의 부모 노드 (경로 역추적용)
            self.cost = cost            # G cost: 시작점부터 현재까지 온 거리 비용
            self.x = x                  # 실제 물리적 X 좌표 (실수)
            self.y = y                  # 실제 물리적 Y 좌표 (실수)
            self.theta = theta          # 로봇의 현재 바라보는 각도 (라디안)
            self.f = 0                  # F cost: G cost + H cost (총 예상 비용)

        def __lt__(self, other):
            # 힙(Heap) 정렬을 위한 비교 연산자 재정의
            # F cost가 더 작은 노드가 우선순위를 가짐
            return self.f < other.f

    def calc_heuristic(self, node, goal_node):
        # 휴리스틱 함수: 현재 노드에서 목표까지의 추정 거리 계산
        # 여기서는 단순 유클리드 거리(직선 거리)를 사용
        dx = node.x - goal_node.x
        dy = node.y - goal_node.y
        return math.hypot(dx, dy)  # sqrt(dx^2 + dy^2) 반환

    def is_collision(self, x, y, grid_map):
        # 특정 위치(x, y)가 장애물과 충돌하는지 확인하는 함수
        
        # 1. 맵 범위 체크: 좌표를 해상도로 나누어 그리드 인덱스로 변환
        ix = int(x / self.reso)
        iy = int(y / self.reso)
        
        # 인덱스가 맵 범위를 벗어나면 충돌(True)로 간주
        if ix < 0 or ix >= self.grid_w or iy < 0 or iy >= self.grid_h:
            return True
            
        # 점 하나만 보는 것이 아니라 주변 영역(Inflation)을 검사
        # 로봇의 크기를 고려하여 여유 공간(margin)을 둡니다.
        
        margin = 6  # 검사할 주변 픽셀 범위 (해상도에 따라 실제 거리는 달라짐)
        
        # 검사할 사각형 영역의 최소/최대 인덱스 계산 (맵 밖으로 나가지 않도록 max, min 사용)
        x_min = max(0, ix - margin)
        x_max = min(self.grid_w, ix + margin + 1)
        y_min = max(0, iy - margin)
        y_max = min(self.grid_h, iy + margin + 1)
        
        # 설정된 범위 내의 모든 픽셀을 순회하며 장애물 확인
        for i in range(x_min, x_max):
            for j in range(y_min, y_max):
                if grid_map[i][j] > 50: # 그리드 값이 50 초과면 장애물로 간주
                    return True
                    
        return False # 충돌 없음

    def plan(self, start, goal, grid_map):
        # 경로 계획 메인 함수
        # start: 시작점 [x, y, theta], goal: 목표점 [x, y, theta]
        
        # 시작 노드 생성 (실제 좌표 -> 그리드 인덱스로 변환 포함)
        start_node = self.Node(int(start[0]/self.reso), int(start[1]/self.reso), 0,
                               None, 0, start[0], start[1], start[2])
        # 목표 노드 생성
        goal_node = self.Node(int(goal[0]/self.reso), int(goal[1]/self.reso), 0,
                              None, 0, goal[0], goal[1], goal[2])

        open_set = []   # 탐색할 노드들을 저장하는 우선순위 큐
        closed_set = {} # 이미 방문한 노드를 저장하는 딕셔너리

        # 시작 노드를 Open Set에 추가
        heapq.heappush(open_set, start_node)

        # Open Set이 빌 때까지 반복 (경로 탐색 루프)
        while open_set:
            # F cost가 가장 낮은 노드를 꺼냄 (현재 가장 유망한 노드)
            current = heapq.heappop(open_set)

            # 목표 도달 확인: 현재 위치와 목표 위치 사이의 거리가 0.5m 미만이면 도착으로 간주
            dist_to_goal = math.hypot(current.x - goal_node.x, current.y - goal_node.y)
            if dist_to_goal < 0.5:
                print("Goal Found!") # 목표 발견 메시지 출력
                return self.reconstruct_path(current) # 경로를 역추적하여 반환

            # Closed Set 확인: 현재 상태를 고유 키로 생성
            # x인덱스, y인덱스, 그리고 각도(10도 단위로 이산화)를 키로 사용
            c_id = (current.x_ind, current.y_ind, int(current.theta / math.radians(10)))
            
            # 이미 방문한 상태라면 스킵
            if c_id in closed_set:
                continue
            
            # 방문하지 않았다면 Closed Set에 등록
            closed_set[c_id] = current

            # 다음 노드 확장 (Motion Primitives): 설정한 조향 각도들로 이동 시도
            for steer in self.steer_set:
                steer_rad = math.radians(steer) # 각도를 라디안으로 변환
                
                # 로봇 운동 모델 적용 (간단한 이동 모델)
                # 다음 각도 = 현재 각도 + 조향 각도
                next_theta = current.theta + steer_rad
                # 다음 X 좌표 = 현재 X + 이동거리 * cos(다음 각도)
                next_x = current.x + self.step_size * math.cos(next_theta)
                # 다음 Y 좌표 = 현재 Y + 이동거리 * sin(다음 각도)
                next_y = current.y + self.step_size * math.sin(next_theta)

                # 이동할 위치가 충돌하는지 확인
                if self.is_collision(next_x, next_y, grid_map):
                    continue # 충돌하면 이 경로는 무시

                # 새로운 노드 생성 준비
                # G cost 업데이트 (이전 비용 + 이동 거리)
                next_g = current.cost + self.step_size
                
                # 새로운 노드 객체 생성 (부모는 현재 노드인 current)
                new_node = self.Node(int(next_x/self.reso), int(next_y/self.reso), 0,
                                     current, next_g, next_x, next_y, next_theta)
                
                # F cost 계산 = G cost + H cost (휴리스틱)
                new_node.f = new_node.cost + self.calc_heuristic(new_node, goal_node)
                
                # Open Set에 새로운 노드 추가
                heapq.heappush(open_set, new_node)

        return None # Open Set이 비었는데도 목표에 도달하지 못함 (경로 없음)

    def reconstruct_path(self, current):
        # 목표점에서 시작점까지 부모 노드를 따라가며 경로를 복원하는 함수
        path = [] # 경로를 저장할 리스트
        while current: # 부모 노드가 없을 때까지(시작점까지) 반복
            path.append([current.x, current.y]) # 현재 노드의 좌표 저장
            current = current.parent # 부모 노드로 이동
        return path[::-1] # 역순으로 저장되었으므로 뒤집어서 반환 (시작 -> 끝 순서)