from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Nodo motor_controller (comunica con Arduino)
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'wheel_radius': 0.033,
                'wheel_separation': 0.18,
                'max_rpm': 100.0
            }]
        ),
        
        # Nodo odometría
        Node(
            package='diff_drive',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'wheel_radius': 0.033,
                'wheel_separation': 0.18,
                'ticks_per_rev': 360.0
            }]
        ),
        
        # Iniciar teleoperación en terminal separado
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'teleop_twist', 'teleop_node'],
        #     output='screen'
        # ),
    ])
