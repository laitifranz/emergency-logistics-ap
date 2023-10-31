### Adapted from the original example ###
# https://github.com/PlanSys2/ros2_planning_system_examples/tree/master/plansys2_problem5 

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
    example_dir = get_package_share_directory('plansys2_problem5')
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
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_robot_cmd = Node(
        package='plansys2_problem5',
        executable='move_robot_action_node',
        name='move_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    move_robot_with_carrier_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='move_robot_with_carrier_action_node',
        name='move_robot_with_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    attach_carrier_to_robot_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='attach_carrier_to_robot_action_node',
        name='attach_carrier_to_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   
    
    detach_carrier_from_robot_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='detach_carrier_from_robot_action_node',
        name='detach_carrier_from_robot_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])   
    
    load_box_on_carrier_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='load_box_on_carrier_action_node',
        name='load_box_on_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    unload_box_from_carrier_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='unload_box_from_carrier_action_node',
        name='unload_box_from_carrier_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    fill_box_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='fill_box_action_node',
        name='fill_box_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    deliver_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='deliver_action_node',
        name='deliver_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 

    delivery_or_refactored_possible_action1_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='delivery_or_refactored_possible_action1_action_node',
        name='delivery_or_refactored_possible_action1_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    delivery_or_refactored_possible_action2_action_node_cmd = Node(
        package='plansys2_problem5',
        executable='delivery_or_refactored_possible_action2_action_node',
        name='delivery_or_refactored_possible_action2_action_node',
        namespace=namespace,
        output='screen',
        parameters=[]) 
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_robot_cmd)
    ld.add_action(move_robot_with_carrier_action_node_cmd)
    ld.add_action(attach_carrier_to_robot_action_node_cmd)
    ld.add_action(detach_carrier_from_robot_action_node_cmd)
    ld.add_action(load_box_on_carrier_action_node_cmd)
    ld.add_action(unload_box_from_carrier_action_node_cmd)
    ld.add_action(fill_box_action_node_cmd)
    ld.add_action(deliver_action_node_cmd)
    ld.add_action(delivery_or_refactored_possible_action1_action_node_cmd)
    ld.add_action(delivery_or_refactored_possible_action2_action_node_cmd)
    
    return ld
