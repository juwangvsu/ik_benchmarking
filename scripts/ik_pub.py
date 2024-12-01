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
    joint_ik = line.split(',')[12:19]
    joint_ik =[float(x) for x in joint_ik]
    #pose = loadtxt(file_path, comments="#", delimiter=",", unpack=False)

    directory_path = os.getcwd()

    # Commands to run ik benchmarking with different IK solvers
    tmpstr0 = "ros2 topic pub /joint_states sensor_msgs/msg/JointState '{header: {frame_id: base_link}, name: ["
    names = "panda_joint1,panda_joint2,panda_joint3, panda_joint4, panda_joint5,panda_joint6,panda_joint7"
    tmpstr1="], position: ["
    positions=f"{joint_ik[0]},{joint_ik[1]},{joint_ik[2]},{joint_ik[3]},{joint_ik[4]},{joint_ik[5]},{joint_ik[6]}"
    tmpstr2="], velocity: [], effort: []}'"

    launch_command = tmpstr0+names+tmpstr1+positions+tmpstr2
    #launch_command = "ros2 topic pub /joint_state_plugin_controller/joint_commands sensor_msgs/msg/JointState '{header: {frame_id: base_link}, name: [j6], position: [1.8], velocity: [], effort: []}'"
    

    process = subprocess.Popen(launch_command, shell=True, executable="/bin/bash")

        # Wait indefinitely for completion to ensure sequential processing
    process.communicate()

import argparse
if __name__ == "__main__":
    print('xxx')
    parser = argparse.ArgumentParser(description="main script .")
    parser.add_argument('-c', '--ik_csv', type=str, default='bio_ik_ik_benchmarking_data.csv', help="headless ")
    parser.add_argument('-p', '--position', type=float, default=100, help="headless ")
    args = parser.parse_args()


    main(args)
