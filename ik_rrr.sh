echo "this script will ros2 sim for panda and cspace\n "

gnome-terminal -x $SHELL -ic "ctt rviz; cd ~/ros2_ws; source install/setup.bash; ros2 run ik_benchmarking ik_pub.py -c KDL_ik_benchmarking_data.csv -u rrr -l -1"
gnome-terminal -x $SHELL -ic "ctt rviz; cd ~/ros2_ws; source install/setup.bash; ros2 launch ik_benchmarking launch.rviz.py urdf_file:=src/ik_benchmarking/urdf/rrr_arm_full_2_back.urdf rviz_config_file:=src/ik_benchmarking/rviz/rrr.rviz
"

