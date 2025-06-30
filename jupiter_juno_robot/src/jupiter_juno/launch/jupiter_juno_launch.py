#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Jupiter Juno robot system"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('jupiter_juno')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration(
        'config_file',
        default=os.path.join(pkg_dir, 'config', 'drowsiness_config.yaml')
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        )
    )
    
    ld.add_action(
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to configuration file'
        )
    )
    
    # Log startup message
    ld.add_action(
        LogInfo(msg='Starting Jupiter Juno Drowsiness Detection System...')
    )
    
    # Eye Detector Node
    eye_detector_node = Node(
        package='jupiter_juno',
        executable='eye_detector_node',
        name='eye_detector_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Alert System Node
    alert_system_node = Node(
        package='jupiter_juno',
        executable='alert_system_node',
        name='alert_system_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # TTS Node
    tts_node = Node(
        package='jupiter_juno',
        executable='tts_node',
        name='tts_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Speech Recognition Node
    speech_recognition_node = Node(
        package='jupiter_juno',
        executable='speech_recognition_node',
        name='speech_recognition_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gemini Conversation Node
    gemini_conversation_node = Node(
        package='jupiter_juno',
        executable='gemini_conversation_node',
        name='gemini_conversation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Add nodes to launch description
    ld.add_action(eye_detector_node)
    ld.add_action(alert_system_node)
    ld.add_action(tts_node)
    ld.add_action(speech_recognition_node)
    ld.add_action(gemini_conversation_node)
    
    return ld 