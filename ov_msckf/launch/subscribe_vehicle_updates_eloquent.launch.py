from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

# OVVU create a new launch python file for vehicle related updates

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="true", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="atcity",
        description="Custom autonomous driving dataset...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="DEBUG",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="false",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="1",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    # Some more arguments
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="true",
        description="Save the estimated trajectory in txt files for further evaluation"),
    DeclareLaunchArgument(
        name="dosave",
        default_value="true",
        description="Save the estimated trajectory in txt files for further evaluation"),
    DeclareLaunchArgument(
        name="dotime",
        default_value="true",
        description="Save the time it takes for each processing step."),
    DeclareLaunchArgument(
        name="filepath_pose_est_rpg",
        default_value="/tmp/stamped_traj_estimate.txt",
        description="Path of the saved trajectory in RPG trajectory evaluation format excluding parts of the covariance."),
    DeclareLaunchArgument(
        name="path_time",
        default_value="/tmp/traj_timing.txt",
        description="Path of the timing information file."),
    DeclareLaunchArgument(
        name="init_window_time",
        default_value="0.75",
        description="Time window of gathered IMU messages for initialization."),
    DeclareLaunchArgument(
        name="init_imu_thresh",
        default_value="0.12",
        description="IMU treshold for detecting start of the trajectory."),
    DeclareLaunchArgument(
        name="calib_cam_timeoffset",
        default_value="false",
        description="Whether to estimate the camera IMU time offset."),
    # declare additional launch arguments
    # DeclareLaunchArgument(
    #     name="topic_imu",
    #     default_value="/imu0",
    #     description="ROS topic of our IMU data."
    # ),
    # DeclareLaunchArgument(
    #     name="topic_ackermann_drive",
    #     default_value="/ackermann0",
    #     description="ROS topic of our Ackermann drive data."
    # ),
    # DeclareLaunchArgument(
    #     name="topic_wheel_speeds",
    #     default_value="/wheel_speeds0",
    #     description="ROS topic of our wheel speeds data."
    # ),
    DeclareLaunchArgument(
        name="vehicle_update_mode",
        default_value="VEHICLE_UPDATE_PREINTEGRATED_SINGLE_TRACK",
        description="Which variant of vehicle related updates to be used."),
    DeclareLaunchArgument(
        name="use_yaw_odom_second_order",
        default_value="true",
        description="Whether to use 2nd order yaw kinematics when calculating the vehicle's motion in preintegrated odometry update."),
    DeclareLaunchArgument(
        name="use_yaw_jacobi_second_order",
        default_value="false",
        description="Whether to use 2nd order yaw kinematics when calculating the Jacobians in the preintegrated odometry update."),
    DeclareLaunchArgument(
        name="speed_update_mode",
        default_value="SPEED_UPDATE_VECTOR",
        description="SPEED_UPDATE_VECTOR introduces pseudo measurements for y- and z-component and sets the to zero. SPEED_UPDATE_X only updates speed in x-direction."),
    DeclareLaunchArgument(
        name="sigma_speed_x",
        default_value="0.1",
        description="Vehicle speed x noise (m/s)."),
    DeclareLaunchArgument(
        name="sigma_zero_speed_y",
        default_value="0.3",
        description="Vehicle zero speed y noise (m/s)."),
    DeclareLaunchArgument(
        name="sigma_zero_speed_z",
        default_value="0.3",
        description="Vehicle zero speed y noise (m/s)."),
    DeclareLaunchArgument(
        name="vehicle_speed_chi2_multiplier",
        default_value="1.0",
        description="Vehicle speed chi2 multiplier."),
    DeclareLaunchArgument(
        name="sigma_steering_angle",
        default_value="0.017453293",
        description="Steering angle variance noise (rad)"),
    DeclareLaunchArgument(
        name="vehicle_steering_chi2_multiplier",
        default_value="1.0",
        description="Vehicle steering chi2 multiplier."),
    DeclareLaunchArgument(
        name="wheel_base",
        default_value="2.791",
        description="Steering updates at very slow speeds lead to performance decrease (m/s)."),
    DeclareLaunchArgument(
        name="steering_angle_update_min_speed",
        default_value="3.0",
        description=""),
    DeclareLaunchArgument(
        name="ackermann_drive_msg_contains_steering_wheel_angle",
        default_value="true",
        description="If set to true, the steering information in Ackermann drive data needs to be divided by the steering_ratio."),
    DeclareLaunchArgument(
        name="steering_ratio",
        default_value="15.2",
        description="Steering ratio for steering wheel angle to steering angle conversion: steering_angle "),
    DeclareLaunchArgument(
        name="max_steering_angle",
        default_value="100.0",
        description=""),
    DeclareLaunchArgument(
        name="track_length",
        default_value="1.568",
        description="")
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(
                get_package_share_directory("ov_msckf"),
                "config", config, "estimator_config.yaml"
            )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                )
            ]
    node1 = Node(
        package="ov_msckf",
        node_executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        output='screen',
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"config_path": config_path},
            # some more parameters
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"dosave": LaunchConfiguration("dosave")},
            {"dotime": LaunchConfiguration("dotime")},
            {"path_est": LaunchConfiguration("path_est")},
            {"filepath_pose_est_rpg": LaunchConfiguration("filepath_pose_est_rpg")},
            {"path_time": LaunchConfiguration("path_time")},
            {"init_window_time": LaunchConfiguration("init_window_time")},
            {"init_imu_thresh": LaunchConfiguration("init_imu_thresh")},
            {"calib_cam_timeoffset": LaunchConfiguration("calib_cam_timeoffset")},
            # {"topic_imu": LaunchConfiguration("topic_imu")},
            # {"topic_ackermann_drive": LaunchConfiguration("topic_ackermann_drive")},
            # {"topic_wheel_speeds": LaunchConfiguration("topic_wheel_speeds")},
            {"vehicle_update_mode": LaunchConfiguration("vehicle_update_mode")},
            {"use_yaw_odom_second_order": LaunchConfiguration("use_yaw_odom_second_order")},
            {"use_yaw_jacobi_second_order": LaunchConfiguration("use_yaw_jacobi_second_order")},
            {"speed_update_mode": LaunchConfiguration("speed_update_mode")},
            {"sigma_speed_x": LaunchConfiguration("sigma_speed_x")},
            {"sigma_zero_speed_y": LaunchConfiguration("sigma_zero_speed_y")},
            {"sigma_zero_speed_z": LaunchConfiguration("sigma_zero_speed_z")},
            {"vehicle_speed_chi2_multiplier": LaunchConfiguration("vehicle_speed_chi2_multiplier")},
            {"sigma_steering_angle": LaunchConfiguration("sigma_steering_angle")},
            {"vehicle_steering_chi2_multiplier": LaunchConfiguration("vehicle_steering_chi2_multiplier")},
            {"wheel_base": LaunchConfiguration("wheel_base")},
            {"steering_angle_update_min_speed": LaunchConfiguration("steering_angle_update_min_speed")},
            {"ackermann_drive_msg_contains_steering_wheel_angle": LaunchConfiguration("ackermann_drive_msg_contains_steering_wheel_angle")},
            {"steering_ratio": LaunchConfiguration("steering_ratio")},
            {"max_steering_angle": LaunchConfiguration("max_steering_angle")},
            {"track_length": LaunchConfiguration("track_length")},
        ],
    )

    node2 = Node(
        package="rviz2",
        node_executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_driving_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    return [node1, node2]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
