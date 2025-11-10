#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file for Aurora noise calibration.

    This launch file starts:
    1. Aurora publisher node (data acquisition)
    2. Noise estimator node (statistical analysis)

    The noise estimator collects data with the sensor stationary,
    computes noise statistics, and outputs recommended Kalman filter parameters.
    """

    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================

    aurora_config_arg = DeclareLaunchArgument(
        'aurora_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('aurora_ndi_ros2_driver'),
            'config',
            'aurora_config.yaml'
        ]),
        description='Path to Aurora driver configuration YAML file'
    )

    noise_config_arg = DeclareLaunchArgument(
        'noise_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('aurora_ndi_ros2_driver'),
            'config',
            'noise_estimator_config.yaml'
        ]),
        description='Path to noise estimator configuration YAML file'
    )

    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='noise_calibration_results.yaml',
        description='Output file for calibration results'
    )

    max_samples_arg = DeclareLaunchArgument(
        'max_samples',
        default_value='1000',
        description='Number of samples to collect for calibration'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='ROS logging level'
    )

    # =============================================================================
    # AURORA PUBLISHER NODE
    # =============================================================================

    aurora_publisher_node = Node(
        package='aurora_ndi_ros2_driver',
        executable='aurora_publisher_node',
        name='aurora_publisher_node',
        parameters=[LaunchConfiguration('aurora_config')],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        emulate_tty=True,
        respawn=False  # Don't respawn during calibration
    )

    # =============================================================================
    # NOISE ESTIMATOR NODE (with slight delay to ensure Aurora is ready)
    # =============================================================================

    noise_estimator_node = Node(
        package='aurora_ndi_ros2_driver',
        executable='noise_estimator_node',
        name='noise_estimator_node',
        parameters=[
            LaunchConfiguration('noise_config'),
            {
                'output_file': LaunchConfiguration('output_file'),
                'max_samples': LaunchConfiguration('max_samples')
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        emulate_tty=True,
        respawn=False
    )

    # Delay noise estimator start to allow Aurora to initialize
    delayed_noise_estimator = TimerAction(
        period=3.0,
        actions=[noise_estimator_node]
    )

    # =============================================================================
    # LAUNCH DESCRIPTION
    # =============================================================================

    return LaunchDescription([
        # Arguments
        aurora_config_arg,
        noise_config_arg,
        output_file_arg,
        max_samples_arg,
        log_level_arg,

        # Startup banner
        LogInfo(msg=""),
        LogInfo(msg="======================================================="),
        LogInfo(msg="     AURORA NOISE CALIBRATION"),
        LogInfo(msg="======================================================="),
        LogInfo(msg=""),
        LogInfo(msg="IMPORTANT: Place the Aurora sensor in a STATIONARY position!"),
        LogInfo(msg=""),
        LogInfo(msg=["  - Collecting: ", LaunchConfiguration('max_samples'), " samples"]),
        LogInfo(msg=["  - Output file: ", LaunchConfiguration('output_file')]),
        LogInfo(msg=""),
        LogInfo(msg="The calibration will run automatically and exit when complete."),
        LogInfo(msg="Results will be saved to the output file."),
        LogInfo(msg=""),
        LogInfo(msg="======================================================="),
        LogInfo(msg=""),

        # Launch Aurora driver immediately
        aurora_publisher_node,

        # Launch noise estimator after 3 seconds delay
        delayed_noise_estimator,

        # Completion message
        LogInfo(msg="Noise calibration launch sequence complete!"),
        LogInfo(msg="Waiting for data collection to finish...")
    ])


# =============================================================================
# USAGE EXAMPLES
# =============================================================================
#
# 1. Basic usage (default 1000 samples):
#    ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py
#
# 2. Quick calibration (500 samples):
#    ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py max_samples:=500
#
# 3. Long calibration (2000 samples):
#    ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py max_samples:=2000
#
# 4. Custom output file:
#    ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py \
#        output_file:=/path/to/my_calibration.yaml
#
# 5. Debug mode:
#    ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py \
#        log_level:=debug
#
# 6. Custom configs:
#    ros2 launch aurora_ndi_ros2_driver aurora_noise_calibration.launch.py \
#        aurora_config:=/path/to/aurora_config.yaml \
#        noise_config:=/path/to/noise_config.yaml
#
# =============================================================================
