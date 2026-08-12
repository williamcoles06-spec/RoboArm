"""
Launches the real arm: robot_state_publisher, move_group, and RViz - but NOT
ros2_control_node or the controller spawners (see ROBOT_ARM_HANDOFF.md §7 for
why: ros2_control_node segfaults loading JointTrajectoryController on this
setup). In place of ros2_control, arm_driver/canbus_bridge.py is launched
directly as the FollowJointTrajectory action server MoveIt talks to.
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

ARM_DRIVER_DIR = os.path.expanduser("~/ros2_ws/src/arm_driver")


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "robo_arm_simplified_for_urdf_v2_1", package_name="robo_arm_moveit_config"
    ).to_moveit_configs()
    pkg_path = moveit_config.package_path

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_path / "launch/rsp.launch.py")))
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_path / "launch/move_group.launch.py")))
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_path / "launch/moveit_rviz.launch.py")))

    canbus_bridge = ExecuteProcess(
        cmd=["python3", "canbus_bridge.py"],
        cwd=ARM_DRIVER_DIR,
        output="screen",
    )

    return LaunchDescription([rsp, move_group, rviz, canbus_bridge])
