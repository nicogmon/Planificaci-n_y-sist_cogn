# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_house_problem')
    print("example_dir: %s", example_dir)
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': example_dir + '/pddl/house_domain.pddl',
            'namespace': namespace
            }.items()
        )

    # Specify the actions

    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])
    ask_open_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='ask_open_door',
        namespace=namespace,
        output='screen',
        parameters=[   
          example_dir + '/config/params.yaml',{
            'action_name': 'ask_open_door',
            'publisher_port': 1670,
            'server_port': 1671,
            'bt_xml_file': example_dir + '/behavior_trees_xml/ask_open_door.xml'
          }    
        ])
             
    pass_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pass_door',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pass_door',
            'publisher_port': 1672,
            'server_port': 1673,
            'bt_xml_file': example_dir + '/behavior_trees_xml/pass_door.xml'
          }
        ])
    tie_bed_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='tie_bed',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'tie_bed',
            'publisher_port': 1674,
            'server_port': 1675,
            'bt_xml_file': example_dir + '/behavior_trees_xml/tie_bed.xml'
          }
        ])
    dish_wash_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='dish_wash',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'dish_wash',
            'publisher_port': 1676,
            'server_port': 1677,
            'bt_xml_file': example_dir + '/behavior_trees_xml/dish_wash.xml'
          }
        ])
    attend_person_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='attend_person',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'attend_person',
            'publisher_port': 1678,
            'server_port': 1679,
            'bt_xml_file': example_dir + '/behavior_trees_xml/attend_person.xml'
          }
        ])
    pick_up_trash_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick_up_trash',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'pick_up_trash',
            'publisher_port': 1680,
            'server_port': 1681,
            'bt_xml_file': example_dir + '/behavior_trees_xml/pick_up_trash.xml'
          }])
    setup_table_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='setup_table',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'setup_table',
            'publisher_port': 1682,
            'server_port': 1683,
            'bt_xml_file': example_dir + '/behavior_trees_xml/setup_table.xml'
          }
        ])
    
    # controller_cmd = Node(
    #     package='plansys2_house_problem',
    #     executable='house_controller_node',
    #     name='house_controller',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[example_dir + '/config/params.yaml']
    #     )
    

    
    
    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)
    
    

    ld.add_action(move_cmd)
    ld.add_action(ask_open_door_cmd)
    ld.add_action(pass_door_cmd)
    ld.add_action(tie_bed_cmd)
    ld.add_action(dish_wash_cmd)
    ld.add_action(attend_person_cmd)
    ld.add_action(pick_up_trash_cmd)
    ld.add_action(setup_table_cmd)

    # ld.add_action(controller_cmd)

    
    
    
    return ld
