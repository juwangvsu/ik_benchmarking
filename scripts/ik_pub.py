#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import yaml
from ament_index_python.packages import get_package_share_directory

import subprocess
import numpy
from numpy import loadtxt


def main(args):
    # Construct the configuration file path
    file_path = os.path.join('.',
        args.ik_csv
    )
    print('csv file path ', file_path)
    f = open(file_path, "r")
    lines = f.readlines()
    line = lines[1].replace('[',' ').replace(']',' ').strip()
    joint_t = line.split(',')[5:12]
    joint_t =[float(x) for x in joint_t]
    #joint_t[0]=joint_t[0]+0.2
    joint_ik = line.split(',')[12:19]
    joint_ik =[float(x) for x in joint_ik]
    #pose = loadtxt(file_path, comments="#", delimiter=",", unpack=False)

    directory_path = os.getcwd()

    # Commands to run ik benchmarking with different IK solvers
    tmpstr0 = "ros2 topic pub /joint_states sensor_msgs/msg/JointState '{header: {frame_id: base_link, stamp: now}, name: ["
    names = "panda_joint1,panda_joint2,panda_joint3, panda_joint4, panda_joint5,panda_joint6,panda_joint7"
    tmpstr1="], position: ["
    positions_ik=f"{joint_ik[0]},{joint_ik[1]},{joint_ik[2]},{joint_ik[3]},{joint_ik[4]},{joint_ik[5]},{joint_ik[6]}"
    positions_t=f"{joint_t[0]},{joint_t[1]},{joint_t[2]},{joint_t[3]},{joint_t[4]},{joint_t[5]},{joint_t[6]}"
    tmpstr2="], velocity: [], effort: []}' -r 2"
    tmpstr3="], velocity: [], effort: []}' -r 3"

    launch_command_ik = tmpstr0+names+tmpstr1+positions_ik+tmpstr2
    launch_command_t = tmpstr0+names+tmpstr1+positions_t+tmpstr3
    #launch_command = "ros2 topic pub /joint_state_plugin_controller/joint_commands sensor_msgs/msg/JointState '{header: {frame_id: base_link}, name: [j6], position: [1.8], velocity: [], effort: []}'"
    

    process = subprocess.Popen(launch_command_ik, shell=True, executable="/bin/bash")
    process_t = subprocess.Popen(launch_command_t, shell=True, executable="/bin/bash")

        # Wait indefinitely for completion to ensure sequential processing
    process.communicate()
    process_t.communicate()

import argparse
if __name__ == "__main__":
    print('xxx')
    parser = argparse.ArgumentParser(description="main script .")
    parser.add_argument('-c', '--ik_csv', type=str, default='bio_ik_ik_benchmarking_data.csv', help="headless ")
    parser.add_argument('-p', '--position', type=float, default=100, help="headless ")
    args = parser.parse_args()


    main(args)
