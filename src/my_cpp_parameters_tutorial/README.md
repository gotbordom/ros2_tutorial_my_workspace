The aim here looks to be learning how to setup env variables and parameters from within a launch file in order to setup all nodes.

While I am aware what a launch file is, similar to how docker compose can be used to spin up N containers and sync them, a launch file does this for ROS2 nodes. 

What I am expecitng to learn from this is how to use this launch file to also be used as a configuration file for the system as a whole. Similar to what I have used in past projects that can more easily allow me to compile the code once, BUT asjust how it works using parameters without recompiling.

We shall see if that is what this is actually about.

Step one: build
- `colcon build --package-select my_cpp_parameters_tutorial`
- fix all bugs, I had a few

Step two: run the code
- `ros2 run my_cpp_parameters_tutorial my_minimal_param_node`
- ( Don't forget to source the workspace overlay I made a basrc_ros file that gets imported into my baserc with all my source commands for each workspace aliased to make this easier.)
- see that this did create a new param in ros ecosystem
  - `ros2 param list` (while the node is running)

Step three: adjust parameters and you choose
- `ros2 param set /name_of_node name_of_parameter value`

Step four: Launch files


Cool! so this did what I hoped it would!