#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#usage: ros2 run ik_benchmarking ik_pub.py -c KDL_ik_benchmarking_data.csv -u rrr -l -1
#-l xxx, xxx the line number to run, -1 for loop through all

import os
import glob
import yaml
from ament_index_python.packages import get_package_share_directory

import subprocess
import numpy
from numpy import loadtxt
import signal
import time

def get_joint_name_value(urdfname, line):
    if urdfname == 'panda':
        joint_t = line.split(',')[8:15]
        joint_t =[float(x) for x in joint_t]
        joint_ik = line.split(',')[15:22]
        joint_ik =[float(x) for x in joint_ik]
        joint_names = "panda_joint1,panda_joint2,panda_joint3, panda_joint4, panda_joint5,panda_joint6,panda_joint7"
        positions_ik=f"{joint_ik[0]},{joint_ik[1]},{joint_ik[2]},{joint_ik[3]},{joint_ik[4]},{joint_ik[5]},{joint_ik[6]}"
        positions_t=f"{joint_t[0]},{joint_t[1]},{joint_t[2]},{joint_t[3]},{joint_t[4]},{joint_t[5]},{joint_t[6]}"
        frame_id="panda_link0"
    elif urdfname == 'rrr': #_arm_full_2_back.urdf':
        joint_t = line.split(',')[8:11]
        joint_t =[float(x) for x in joint_t]
        joint_ik = line.split(',')[11:14]
        joint_ik =[float(x) for x in joint_ik]
        joint_names = "joint_1,joint_2,joint_3"
        positions_ik=f"{joint_ik[0]},{joint_ik[1]},{joint_ik[2]}"
        positions_t=f"{joint_t[0]},{joint_t[1]},{joint_t[2]}"
        frame_id="base_link"
    elif urdfname == 'rrrfork': #_arm_full_2_back.urdf':
        joint_t = line.split(',')[8:15]
        joint_t =[float(x) for x in joint_t]
        joint_ik = line.split(',')[15:22]
        joint_ik =[float(x) for x in joint_ik]
        joint_names = "joint_1,joint_2,joint_3,joint_4, joint_41, joint_42, joint_43"
        positions_ik=f"{joint_ik[0]},{joint_ik[1]},{joint_ik[2]},{joint_ik[3]},{joint_ik[4]},{joint_ik[5]},{joint_ik[6]}"
        positions_t=f"{joint_t[0]},{joint_t[1]},{joint_t[2]},{joint_t[3]},{joint_t[4]},{joint_t[5]},{joint_t[6]}"
        frame_id="base_link"

    return joint_t, joint_ik, joint_names, positions_ik, positions_t, frame_id
    #return ik sol and ground truth

def main(args):
    # Construct the configuration file path
    file_path = os.path.join('.',
        args.ik_csv
    )
    print('csv file path ', file_path)
    f = open(file_path, "r")
    lines = f.readlines()
    for i in range(len(lines)):
        lnum = i+1
        if not lnum==args.line and not args.line==-1:
            #skip if lnum is not the one specified
            continue
        line = lines[lnum].replace('[',' ').replace(']',' ').strip()
        succ = line.split(',')[1]
        if succ=='false':
            print('unsucc case # ', lnum)
            continue
        print('processing line ', line)
        joint_t, joint_ik, joint_names, positions_ik, positions_t, frame_id = get_joint_name_value(args.urdf,line)

        #xyz = positions_t #this is already a string, but it is the joint value, not xyz 9/15/25
        xyz="0.3,0.3,0.5"
        directory_path = os.getcwd()

    # Commands to run ik benchmarking with different IK solvers
        tmpstr0 = "ros2 topic pub /joint_states sensor_msgs/msg/JointState '{header: {frame_id: base_link, stamp: now}, name: ["
        #joint_names
        tmpstr1="], position: ["
        #positions_ik=f"{joint_ik[0]},{joint_ik[1]},{joint_ik[2]},{joint_ik[3]},{joint_ik[4]},{joint_ik[5]},{joint_ik[6]}"
        #positions_t=f"{joint_t[0]},{joint_t[1]},{joint_t[2]},{joint_t[3]},{joint_t[4]},{joint_t[5]},{joint_t[6]}"
        tmpstr2="], velocity: [], effort: []}' -r 2"
        tmpstr3="], velocity: [], effort: []}' -r 3"

        launch_command_ik = tmpstr0+joint_names+tmpstr1+positions_ik+tmpstr2
        launch_command_t = tmpstr0+joint_names+tmpstr1+positions_t+tmpstr3
    #launch_command = "ros2 topic pub /joint_state_plugin_controller/joint_commands sensor_msgs/msg/JointState '{header: {frame_id: base_link}, name: [j6], position: [1.8], velocity: [], effort: []}'"
    

        process = subprocess.Popen(launch_command_ik, shell=True, executable="/bin/bash")
        process_t = subprocess.Popen(launch_command_t, shell=True, executable="/bin/bash")
        command_mark = "python ~/ros2_ws/install/ik_benchmarking/lib/ik_benchmarking/marker.py --ros-args -p frame_id:="+frame_id +" -p xyz:="+xyz

        process_marker = subprocess.Popen(command_mark, shell=True, executable="/bin/bash")
        # Wait indefinitely for completion to ensure sequential processing
        #process.communicate()
        #process_t.communicate()
        time.sleep(2)
        print("killing the ros2 topic process----------------", process.pid)
        os.kill(process.pid, signal.SIGKILL)
        time.sleep(1)
        print("killing the ros2 topic process----------------", process_t.pid)
        time.sleep(1)
        os.kill(process_t.pid, signal.SIGKILL)
        print("killing the ros2 topic process----------------", process_marker.pid)
        os.kill(process_marker.pid, signal.SIGKILL)

import argparse
if __name__ == "__main__":
    print('xxx')
    parser = argparse.ArgumentParser(description="main script .")
    parser.add_argument('-c', '--ik_csv', type=str, default='bio_ik_ik_benchmarking_data.csv', help="headless ")
    parser.add_argument('-u', '--urdf', type=str, default='panda', help="headless ")
    parser.add_argument('-l', '--line', type=int, default=1, help="headless ")
    args = parser.parse_args()


    main(args)
