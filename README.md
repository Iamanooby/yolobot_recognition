# yolobot_recognition

Installation Steps:  

Step 1) Install ROS2 Foxy (Galactic should be even better)  
Step 2) Install turtlebot3 PC Setup here: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/  
Step 3) Install turtlebot3 simulation here: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/  
Step 4) Create a virtual environment like here: https://www.cyberithub.com/how-to-install-virtualenv-on-ubuntu-20-04-lts-focal-fossa/  
Step 5) pip install ultralytics
Step 6) Add more models using here: https://github.com/leonhartyao/gazebo_models_worlds_collection   
Step 7) Insert "export GAZEBO_MODEL_PATH=/path/to/my/package/models/:$GAZEBO_MODEL_PATH" in ~/.bashrc  

Startup Steps, each step in new terminal (Assuming steps taken to generate map_yaml):  
Step 1)  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py  
Step 2) ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/house_map.yaml  
Step 3.1) Activate virtual environment  
Step 3.2)ros2 run yolobot_recognition yolov8_ros2_pt_custom.py  
Step 4) ros2 run yolobot_recognition yolov8_ros2_subscriber_obstacle_avoid.py   
Step 5) Insert desired model of choice (current used pot_flower and plant 1)  
Step 6) Set 2D Pose Estimate on Rviz  
Step 7) Give it the deisred goal on Rviz  


[![Watch the video](https://img.youtube.com/vi/MWKs34pADp0/maxresdefault.jpg)](https://youtu.be/MWKs34pADp0)

Note:  
Coco dataset numbering if you wish to change classes: https://docs.google.com/spreadsheets/d/1qnSM1fJaL26lYNFUW_gbfaOhxmU1uiitWdDOziH4MmM/edit?usp=sharing   

References:  
Obtained yolov8 from linked drive here:https://drive.google.com/drive/folders/1FPhKoNdOjgIh4To6dgvO8z0UgDsVEfGV   
