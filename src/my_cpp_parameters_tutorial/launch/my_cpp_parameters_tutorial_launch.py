from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package="my_cpp_parameters_tutorial",
      executable="my_minimal_param_node",
      name="my_custom_minimal_param_node",
      output="screen",
      emulate_tty=True,
      parameters=[
        {"my_parameter": "my parameter set on launch"}
      ]
    )
  ])

