sticky:
ik_benchmarking_nv6.yaml:
	moveit_config_pkg and robot_name fields must match, i.e.
		moveit_config_pkg: moveit_resources_nv6_moveit_config
		robot_name: moveit_resources_nv6
copy and edit
	bio_ik_kinematics.yaml
	trac_ik_kinematics.yaml to moveit_resources_nv6_moveit_config/config
	panda_arm -> nv6_arm or arm depending on the planning group name of robot model

urdf loading path:
	rrr: ik_benchmarking ik_benchmarking_rrr.yaml ->rrr_arm_moveit_config/.setup_assistant -> moveit_resources_rrr_description/urdf/rrr_arm_full_2_back.urdf
	nv6: /home/student/ros2_ws/install/nv6/share/nv6/urdf/nv6.urdf
	panda: install/moveit_resources_panda_des...urdf/panda.urdf
	ros2_ws/install/moveit_resources_panda_moveit_config/ build from
		src/moveit_resources/panda_moveit_config

-----------12/9/24 bio_ik works now for rrr and nv6 ----------
previous error due to bio_ik_kinematics.yaml not modified from panda's

todo:
	more test on rrr robot, bio_ik, create rrr_fork.urdf and moveit_config
	pkg.

-----------11/30/24-----------------
visualize the result of bio_ik:
	ros2 run ik_benchmarking ik_pub.py -c bio_ik_ik_benchmarking_data.csv -u rrr
	ros2 run ik_benchmarking ik_pub.py -c bio_ik_ik_benchmarking_data.csv -u panda 
	ros2 launch ik_benchmarking launch.rviz.py urdf_file:=src/ik_benchmarking/urdf/panda.urdf rviz_config_file:=src/ik_benchmarking/rviz/panda.rviz 
	ros2 launch ik_benchmarking launch.rviz.py urdf_file:=src/ik_benchmarking/urdf/rrr_arm_full_2_back.urdf rviz_config_file:=src/ik_benchmarking/rviz/rrr.rviz 

status:
	bio_ik result visualized for panda

----------11/19/24 ik_benchmark ----------
hpzbook:ros2_ws
        ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:=bio_ik
        ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:=bio_ik ik_conf:=ik_benchmarking_rrr.yaml
        ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:=KDL ik_conf:=ik_benchmarking_rrr.yaml
	ros2 launch ik_benchmarking start_ik_benchmarking.launch.py ik_solver_name:=TRAC_IK ik_conf:=ik_benchmarking_rrr.yaml
        ros2 run ik_benchmarking ik_benchmarking_data_generator.py -c ik_benchmarking_rrr.yaml
                this generate csv data file
	ros2 run ik_benchmarking ik_benchmarking_data_visualizer.py

tbd: to test panda urdf with bio_ik


