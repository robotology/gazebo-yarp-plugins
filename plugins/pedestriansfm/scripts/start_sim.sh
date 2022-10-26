yarpserver &
sleep 1;
ros2 launch gazebo_ros gazebo.launch.py world:=/home/user1/ros2_ws/src/gazebo_sfm_plugin/worlds/GAM.world &
sleep 1;
yarprun --server /console --log &
sleep 1;
#yarplogger --start &
#sleep 1;
yarpmanager
