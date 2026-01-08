source install/setup.sh

ros2 launch foxglove_bridge foxglove_bridge_launch.xml

ros2 run robot_sensor robot_sensor_node \
  --ros-args \
  -p data_file_path:=/home/zqj/Desktop/zhouquanjing_202504080705/task4/src/robot_sensor/data/data.txt \
  -p frequency:=2.0 \
  -p loop_data:=true
