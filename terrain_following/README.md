# Relative Terrain Following

```
gazebo_sim_models mavros_iris_depth.launch
rosrun terrain_following rtf_planner.py
rosrun terrain_following waypoints_action_client.py
```

1. [mavros_controller.py](../src/mavros_controller.py)  
It creates services and subscribers listening to uav states, and also provides methods of setting modes. 

2. [mavros_pos_controller.py](../src/mavros_pos_controller.py)  
It inherits mavros_controller.py and does the following tasks.
    - It has an action service server and a callback function to process the action. The callback function executes the waypoints, which are given in the action message, and return an action result.
    - The waypoints reaching is done by position control, ```def reach_position(self)```.

3. [mavros_vel_controller.py](../src/mavros_vel_controller.py)  
It inherits mavros_controller.py and does the following tasks.
    - It has an action service server and a callback function to process the action. The callback function executes the waypoints, which are given in the action message, and return an action result.
    - The waypoints reaching is done by velocity control.

4. [waypoints_action_client.py](../src/waypoints_action_client.py)  
It provides an action service client, which allows to define waypoints and then send an action request to rtf_planner.py.

5. [rtf_planner.py](../src/src/rtf_planner.py)  
It inherits either mavros_pos_controller.py or mavros_vel_controller.py and overwrites some functions to update the heights of waypoints according to the perception information.

6. [perception_model.py](../src/perception_model.py)  
It supports to process pointcloud data and get the environment representation for the planner.  