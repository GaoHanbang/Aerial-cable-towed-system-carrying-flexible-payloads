from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from nav2_common.launch import RewrittenYaml
import yaml
import jinja2
import os

try:
    px4_folder = os.environ['PX4_SOURCE_DIR']
except KeyError:
    print("Please define PX4_SOURCE_DIR with your PX4 folder path to run the SITL simulation.")
    exit(os.EX_CONFIG)


def launch_setup(context, *args, **kwargs):
    px4_models_folder = os.path.join(px4_folder, "Tools/sitl_gazebo/models")
    px4_plugin_folder = os.path.join(px4_folder, 'build/px4_sitl_rtps/build_gazebo')
    drone_models_folder = os.path.join(get_package_share_directory('ls2n_drone_bridge'), 'sitl_px4_simulation/models')
    model = LaunchConfiguration("drone_model").perform(context)
    model_jinja_file = os.path.join(drone_models_folder, model + "/" + model + ".sdf.jinja")
    try:
        n_drones = int(LaunchConfiguration("number_of_drones").perform(context))
    except ValueError:
        print("Number of drones should be an integer.")
        exit(os.EX_USAGE)
    # Generate the sdf files
    model_out_file_main = os.path.join(drone_models_folder, model + "/" + model + ".sdf")
    model_out_file = [os.path.join(drone_models_folder, model + "/" + model + '_' + str(i) + ".sdf")
                      for i in range(n_drones)]
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(drone_models_folder))
    template = env.get_template(os.path.relpath(model_jinja_file, drone_models_folder))
    d = {'mavlink_tcp_port': str(4560),
         'mavlink_udp_port': str(14560),
         'mavlink_id': str(0),
         'serial_enabled': "0",
         'serial_device': "/dev/ttyACM0",
         'serial_baudrate': "921600",
         'hil_mode': "0",
         'drone_number': str(1)}
    result = template.render(d)
    with open(model_out_file_main, "w") as f_out:
        f_out.write(result)
    for i in range(n_drones):
        d = {'mavlink_tcp_port': str(4560 + i),
             'mavlink_udp_port': str(14560 + i),
             'mavlink_id': str(i + 1),
             'serial_enabled': "0",
             'serial_device': "/dev/ttyACM0",
             'serial_baudrate': "921600",
             'hil_mode': "0",
             'drone_number': str(1 + i)}
        result = template.render(d)
        with open(model_out_file[i], "w") as f_out:
            f_out.write(result)

    # Launch gazebo
    env_variables = {'GAZEBO_MODEL_PATH':
                         [LaunchConfiguration('gz_model_path'),
                          ":", px4_models_folder, ':', drone_models_folder],
                     'GAZEBO_PLUGIN_PATH':
                         [LaunchConfiguration('gz_plugin_path'), ":", px4_plugin_folder]}
    gazebo_server_cmd = ['gzserver', LaunchConfiguration('gz_world'), '--verbose']
    # Spawn drones in gazebo
    spawn_cmd = [ExecuteProcess(
        cmd=['gz', 'model', '--spawn-file=' + model_out_file[i], '--model-name=' + model + '_' + str(i+1), '-x',
             str(i), '-y', '0', '-z', '0.1'],
        prefix="sh -c 'sleep {}; $0 $@'".format(i+3))
        for i in range(n_drones)]
    gazebo_client_cmd = ['gzclient']
    gazebo_launch = [
                        ExecuteProcess(cmd=gazebo_server_cmd, output='screen', additional_env=env_variables),
                        ExecuteProcess(cmd=gazebo_client_cmd, output='screen', additional_env=env_variables,
                                       prefix="sh -c 'sleep 5; $0 $@'")] + spawn_cmd

    # Run the PX4 instances
    px4_executable = os.path.join(px4_folder, "build/px4_sitl_rtps/bin/px4")
    build_path = os.path.join(px4_folder, "build/px4_sitl_rtps")
    for i in range(n_drones):
        if not os.path.exists(os.path.join(build_path, "instance" + str(i))):
            os.makedirs(os.path.join(build_path, "instance" + str(i)))
    px4_env = {'PX4_SIM_MODEL': model + "_rtps"}
    px4_cmd = [ExecuteProcess(cmd=['pkill', '-x', 'px4'])]
    px4_cmd += [ExecuteProcess(
        cmd=['gnome-terminal', '-t', 'SITLDrone' + str(i + 1), '--', px4_executable, '-i', str(i), '-d',
             os.path.join(build_path, "etc"), '-w ', 'sitl' + model + "_rtps" + str(i),
             '-s', 'etc/init.d-posix/rcS'],
        cwd=os.path.join(build_path, "instance" + str(i)),
        additional_env=px4_env,
        prefix="sh -c 'sleep {}; $0 $@'".format(i+4))
        for i in range(n_drones)]

    # One bridge node per drone is started which a specific namespace
    params = [None] * n_drones
    for i in range(n_drones):
        params[i] = RewrittenYaml(
            source_file=os.path.join(get_package_share_directory('ls2n_drone_bridge'),
                                     'config', 'sitl_' + model + '_params.yaml'),
            root_key="Drone" + str(i + 1),
            param_rewrites={}
        )
    micro_rtps_nodes_launch = [Node(
        package='ls2n_px4_ros_com', executable='micrortps_agent',
        output='screen', namespace='Drone' + str(i + 1),
        parameters=[{'transport': 'UDP'},
                    {'receive_port': 2020 + i * 2},
                    {'send_port': 2019 + i * 2}])
        for i in range(n_drones)]
    bridge_nodes_launch = [Node(
        package='ls2n_drone_bridge', executable='drone_bridge',
        output='screen', namespace='Drone' + str(i + 1),
        parameters=[params[i]])
        for i in range(n_drones)]

    # Additional SITL "onboard" nodes
    with open(os.path.join(get_package_share_directory('ls2n_drone_bridge'),
                           'config', 'onboard_nodes.yaml'), 'r') as stream:
        add_nodes = yaml.safe_load(stream)
    onboard_nodes = []
    for node in add_nodes:
        if add_nodes[node]:
            onboard_nodes += [IncludeLaunchDescription(PythonLaunchDescriptionSource(
                [get_package_share_directory('ls2n_drone_onboard'), '/sitl_' + node + '.launch.py']),
                launch_arguments={'drone_namespace': 'Drone' + str(i + 1)}.items())
                for i in range(n_drones)]

    return gazebo_launch + px4_cmd + bridge_nodes_launch + micro_rtps_nodes_launch + onboard_nodes


def generate_launch_description():
    if not os.path.exists(os.path.join(px4_folder, 'build/px4_sitl_rtps/build_gazebo')):
        print("Please correct your PX4_SOURCE_DIR or "
              "build PX4 with the options 'px4_sitl_rtps gazebo' to run the SITL simulation.")
        exit()
    return LaunchDescription([
        DeclareLaunchArgument('drone_model', default_value='crazy2fly',
                              description='Drone model', choices=['crazy2fly', 'crazyflie2']),
        DeclareLaunchArgument('number_of_drones', default_value='1',
                              description='Number of drones'),
        DeclareLaunchArgument('gz_world',
                              default_value=[EnvironmentVariable('PX4_SOURCE_DIR'),
                                             '/Tools/sitl_gazebo/worlds/empty.world'],
                              description='Gazebo world to use'),
        DeclareLaunchArgument('gz_model_path', default_value='',
                              description='Gazebo additional models path'),
        DeclareLaunchArgument('gz_plugin_path', default_value='',
                              description='Gazebo additional plugins path'),
        OpaqueFunction(function=launch_setup)
    ])
