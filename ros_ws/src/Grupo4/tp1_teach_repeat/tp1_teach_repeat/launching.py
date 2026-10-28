"""Configuración idéntica de localización en ambas etapas."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def stage_launch(stage):
    share = get_package_share_directory('tp1_teach_repeat')
    defaults = dict(mode='mixed', use_sim_time='true', route_file='routes/route.json',
                    overwrite='false', rviz='false', publish_stamped='true',
                    initial_x='0.0', initial_y='0.0', initial_yaw='0.0',
                    scan_topic='/scan', joint_states_topic='/joint_states', cmd_vel_topic='/cmd_vel',
                    params_file=os.path.join(share, 'config', 'params.yaml'))
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    p = {k: ParameterValue(LaunchConfiguration(k), value_type=t) for k, t in (
        ('use_sim_time', bool), ('mode', str), ('route_file', str), ('overwrite', bool),
        ('initial_x', float), ('initial_y', float), ('initial_yaw', float))}
    p['stage'] = stage
    remaps = [
        ('/scan', LaunchConfiguration('scan_topic')),
        ('/joint_states', LaunchConfiguration('joint_states_topic')),
        ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
    ]
    nodes = [Node(package='tp1_teach_repeat', executable='localization', output='screen',
                  parameters=[LaunchConfiguration('params_file'), p],
                  remappings=remaps)]
    if stage == 'repeat':
        nodes.append(Node(package='tp1_teach_repeat', executable='controller', output='screen',
                          parameters=[LaunchConfiguration('params_file'),
                                      {'use_sim_time': p['use_sim_time'],
                                       'publish_stamped': ParameterValue(LaunchConfiguration('publish_stamped'), value_type=bool)}],
                          remappings=remaps))
    nodes.append(Node(package='rviz2', executable='rviz2', output='screen',
                      arguments=['-d', os.path.join(share, 'config', 'tp1_view.rviz')],
                      parameters=[{'use_sim_time': p['use_sim_time']}],
                      condition=IfCondition(LaunchConfiguration('rviz'))))
    return LaunchDescription(args + nodes)
