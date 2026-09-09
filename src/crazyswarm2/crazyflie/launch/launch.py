import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def _mocap_enabled(context):
    backend = LaunchConfiguration('backend').perform(context)
    mocap = LaunchConfiguration('mocap').perform(context)
    return backend != 'sim' and mocap.lower() in ('true', '1')


def check_vendored_mocap_driver(context):
    """Abort the launch if motion_capture_tracking would come from apt.

    The apt release 1.0.9 (Humble; queued for Jazzy) hard-codes a foreign
    interface IP (141.23.110.162) into the NatNet socket code: the node either
    aborts with exit -6 or publishes an empty /poses forever. This repo vendors
    a fixed copy in src/motion_capture_tracking (see its VENDORED.md); the
    workspace build must be the one that resolves in this shell.
    """
    if not _mocap_enabled(context):
        return
    try:
        prefix = get_package_prefix('motion_capture_tracking')
    except PackageNotFoundError:
        raise RuntimeError(
            "motion_capture_tracking not found. It is vendored in "
            "src/motion_capture_tracking: run ./scripts/build.sh and then "
            "`source install/setup.bash` in this shell (or launch with mocap:=False).")
    if prefix.startswith('/opt/ros'):
        raise RuntimeError(
            f"motion_capture_tracking resolves to the apt package at {prefix}. "
            "apt 1.0.9 hard-codes interface IP 141.23.110.162 and never receives "
            "NatNet frames, so /poses would stay silent. This repo vendors a fixed "
            "copy in src/motion_capture_tracking: run ./scripts/build.sh, "
            "`source install/setup.bash` in THIS shell, and remove the apt copy "
            "(sudo apt remove ros-$ROS_DISTRO-motion-capture-tracking "
            "ros-$ROS_DISTRO-motion-capture-tracking-interfaces).")


def parse_yaml(context):
    check_vendored_mocap_driver(context)
    # Load the crazyflies YAML file
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml, 'r') as file:
        crazyflies = yaml.safe_load(file)
    # store the fileversion
    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc

    # construct motion_capture_configuration
    motion_capture_yaml = LaunchConfiguration('motion_capture_yaml_file').perform(context)
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture_content['/motion_capture_tracking']['ros__parameters']
    motion_capture_params['rigid_bodies'] = dict()
    for key, value in crazyflies['robots'].items():
        type = crazyflies['robot_types'][value['type']]
        if value['enabled'] and \
            ((fileversion == 1 and type['motion_capture']['enabled']) or \
            ((fileversion >= 2 and type['motion_capture']['tracking'] == "librigidbodytracker"))):
            motion_capture_params['rigid_bodies'][key] =  {
                    'initial_position': value['initial_position'],
                    'marker': type['motion_capture']['marker'],
                    'dynamics': type['motion_capture']['dynamics'],
                }

    # copy relevent settings to server params
    server_params[1]['poses_qos_deadline'] = motion_capture_params['topics']['poses']['qos']['deadline']
    
    return [
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' != 'sim' and '", LaunchConfiguration('mocap'), "'.lower() in ('true', '1')"])),
            name='motion_capture_tracking',
            output='screen',
            parameters= [motion_capture_params],
        ),
        Node(
            package='crazyflie_server_py',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cflib'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cpp'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
            prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','sim'),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= server_params,
        )]

def generate_launch_description():
    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    default_motion_capture_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'config.rviz')

    telop_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('crazyflies_yaml_file', 
                              default_value=default_crazyflies_yaml_path),
        DeclareLaunchArgument('motion_capture_yaml_file', 
                              default_value=default_motion_capture_yaml_path),
        DeclareLaunchArgument('rviz_config_file', 
                              default_value=default_rviz_config_path),
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('gui', default_value='False'),
        DeclareLaunchArgument('preflight', default_value='True'),
        DeclareLaunchArgument('foxglove', default_value='True'),
        DeclareLaunchArgument('teleop', default_value='True'),
        DeclareLaunchArgument('mocap', default_value='True'),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        OpaqueFunction(function=parse_yaml),
        Node(
            condition=LaunchConfigurationEquals('teleop', 'True'),
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('arm', 'all/arm'),
                ('takeoff', 'all/takeoff'),
                ('land', 'all/land'),
                # uncomment to manually control (and update teleop.yaml)
                # ('cmd_vel_legacy', 'cf6/cmd_vel_legacy'),
                # ('cmd_full_state', 'cf6/cmd_full_state'),
                # ('notify_setpoints_stop', 'cf6/notify_setpoints_stop'),
            ],
            parameters= [PythonExpression(["'" + telop_yaml_path +"' if '", LaunchConfiguration('teleop_yaml_file'), "' == '' else '", LaunchConfiguration('teleop_yaml_file'), "'"])],
        ),
        Node(
            condition=LaunchConfigurationEquals('teleop', 'True'),
            package='joy',
            executable='joy_node',
            name='joy_node' # by default id=0
        ),
        Node(
            condition=LaunchConfigurationEquals('rviz', 'True'),
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('gui', 'True'),
            package='crazyflie',
            namespace='',
            executable='gui.py',
            name='gui',
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('preflight', 'True'),
            package='crazyflie',
            namespace='',
            executable='preflight_kalman_plotter.py',
            name='preflight_kalman_plotter',
            output='screen',
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('foxglove', 'True'),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                "port": 8765,
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
    ])
