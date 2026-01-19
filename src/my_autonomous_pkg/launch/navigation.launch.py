# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():

#     pkg_path = get_package_share_directory('my_autonomous_pkg')

#     map_file = '/home/student/my_map.yaml'
#     params_file = os.path.join(pkg_path, 'config', 'teb_params.yaml')

#     nav2_bringup_dir = get_package_share_directory('nav2_bringup')

#     nav2_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
#         ),
#         launch_arguments={
#             'slam': 'false',
#             'map': map_file,
#             'params_file': params_file,
#             'use_sim_time': 'true',
#             'autostart': 'true'
#         }.items()
#     )

#     return LaunchDescription([
#         nav2_launch
#     ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 내 패키지 경로 찾기
    pkg_path = get_package_share_directory('my_autonomous_pkg')

    map_file = '/home/teamone/map.yaml'
    
    # 2. 파라미터 파일 경로 지정
    params_file = os.path.join(pkg_path, 'config', 'mppi_params.yaml')
    
    # 3. Nav2 실행 (이미 설치된 패키지 활용)
    # Nav2 Bringup은 지도 서버, 컨트롤러(TEB) 등을 한방에 켜줍니다.
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file, # 내가 만든 TEB 설정 적용
            'use_sim_time': 'False',
            'map': map_file,
            'autostart': 'True'
        }.items()
    )

    # # 4. 내가 만든 Hybrid A* 노드 실행
    # my_planner = Node(
    #     package='my_autonomous_pkg',
    #     executable='planner_node',
    #     name='hybrid_a_star_planner',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # 5. 실행 목록 리턴
    return LaunchDescription([
        nav2_launch,
        # my_planner
    ])