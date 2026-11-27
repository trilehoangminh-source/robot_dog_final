import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    # 1. Lấy đường dẫn của package
    package_dir = get_package_share_directory('robot_dog_final')
    world_path = os.path.join(package_dir, 'worlds', 'quad_arena.wbt')

    # 2. Khởi động Webots (Mở cái sân đã có sẵn file PROTO con chó)
    webots = WebotsLauncher(world=world_path)

    # 3. Khởi động Node điều khiển SMC
    # Tui đã thêm additional_env để Node Python biết con robot tên là 'robot_dog'
    smc_node = Node(
        package='robot_dog_final',
        executable='smc_controller_node',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'robot_dog'}
    )

    return LaunchDescription([
        webots,
        smc_node,
        
        # Tự động tắt Node khi đóng cửa sổ Webots
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])