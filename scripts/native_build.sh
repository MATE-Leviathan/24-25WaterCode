rosdep install -i --from-path /home/ubuntu/24-25WaterCode/fishROS_ws/src --rosdistro humble -y
cd /home/ubuntu/24-25WaterCode/fishROS_ws/ 
if [ $# -eq 0 ]; then
    colcon build --symlink-install --parallel-workers 4 
else
    colcon build --symlink-install --parallel-workers 4 --packages-select "$@"
fi
