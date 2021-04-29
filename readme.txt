# Robot Chess

This ROS package spawns chess pieces that baxter will place on the board in the locations specified in the setup.  

Step 1: 
roslaunch baxter_gazebo baxter_world.launch 

Step 2:
rosrun baxter_tools enable_robot.py -e 
rosrun baxter_interface joint_trajectory_action_server.py 

Step 3:
roslaunch baxter_moveit_config baxter_grippers.launch

Step 4:
rosrun chess_baxter pick_and_place_moveit.py

Step 5:
rosrun chess_baxter spawn_chessboard_h.py

