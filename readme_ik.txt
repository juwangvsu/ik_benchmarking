-----------11/30/24-----------------
visualize the result of bio_ik:
	ros2 run ik_benchmarking ik_pub.py
	ros2 launch ik_benchmarking launch.rviz.py
status:
	bio_ik result visualized for panda

----------11/19/24 ik_benchmark ----------
hpzbook:ros2_ws
        ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:=bio_ik
        ros2 run ik_benchmarking ik_benchmarking_data_generator.py
                this generate csv data file
tbd: to test panda urdf with bio_ik


