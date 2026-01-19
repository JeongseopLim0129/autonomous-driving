import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 찾기
    # (TurtleBot3 예제와 Nav2 설정을 활용합니다)
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. Gazebo 실행 (월드 + 로봇)
    # TurtleBot3의 기본 월드를 불러옵니다.
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 3. RViz 실행
    # (Nav2에서 제공하는 Navigation용 RViz 설정을 빌려다 쓰면 편합니다)
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], # 설정 파일(-d) 로드
        parameters=[{'use_sim_time': True}], # 중요! Gazebo 시간과 동기화
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_node
    ])