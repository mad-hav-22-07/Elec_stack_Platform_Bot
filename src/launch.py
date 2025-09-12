from launch import LaunchDescription
from ros_launch.actions import Node

def generate_launch_description():

    return LaunchDerscription([

    Node(
        package = 'elecstack',
        executable = 'pid'
    ),

    Node(
        package = 'elecstack',
        executable = 'keystroke'
    ),


    Node(
        package = 'elecstack',
        executable = 'encoder'
    ),



    Node(
        package = 'elecstack',
        executable = 'driver'
    ),

    ])

