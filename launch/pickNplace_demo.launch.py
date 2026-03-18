import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
# from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # 1. Get Setup Path and Namespace
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path_str = setup_path.perform(context)

    # Get namespace (e.g., j100_0921)
    try:
        cp_config = ClearpathConfig(os.path.join(setup_path_str, 'robot.yaml'))
        namespace = cp_config.get_namespace()
    except Exception as e:
        print(f"Warning: Could not read robot.yaml, defaulting to empty namespace. Error: {e}")
        namespace = ""

    # 2. Robot Description (Use the Global one, not the manipulators one)
    # The global description includes the base AND the arm, which is what MTC needs.
    urdf_path = os.path.join(setup_path_str, 'robot.urdf.xacro')
    robot_description = {
        'robot_description': xacro.process_file(urdf_path).toxml()
    }

    srdf_path = os.path.join(setup_path_str, 'robot.srdf')
    if not os.path.exists(srdf_path):
        srdf_path = os.path.join(setup_path_str, 'robot.srdf.xacro')
    
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    
    # 3. Define the line to inject
    new_eef_line = '    <end_effector name="arm_0_eef" parent_link="arm_0_end_effector_link" group="arm_0_gripper"/>\n'

    modified_srdf = srdf_content.replace('</robot>', f'{new_eef_line}</robot>')

    robot_description_semantic = {
        'robot_description_semantic': modified_srdf
    }

    moveit_config_file =  os.path.join(setup_path_str,'manipulators','config','moveit.yaml')
    control_config_file =  os.path.join(setup_path_str,'manipulators','config','control.yaml')
    
    with open(moveit_config_file, 'r') as file:
        moveit_yaml_content = yaml.safe_load(file)

    moveit_params = moveit_yaml_content['a300_00036']['move_group']['ros__parameters']

    package = "kinova_task_manager"
    package_shared_path = get_package_share_directory(package)

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_params,
            move_group_capabilities,
            {"use_sim_time": False},
        ],
        remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states'),
            ],
    )

    # 6. MTC Task Node
    mtc_task_node = Node(
        package="kinova_task_manager",
        executable="pick_place_demo",
        namespace=namespace,
        # name="pick_place_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_params,
            os.path.join(package_shared_path, "config", "gen3lite_config.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[

            # 1. Connect MTC action client to the real move_group server
            # ('move_action', 'move_group'),

            # 2. Connect joint states to the PLATFORM topic (where arm_0_joint_1 lives)

            ('joint_states', 'platform/joint_states'),

            # 3. Ensure TF is visible (Your robot publishes to /j100_0921/tf)
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )



    return [mtc_task_node]


def generate_launch_description():
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath setup path'
    )
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='use_sim_time'
    )

    ld = LaunchDescription()
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld