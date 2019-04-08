#! /bin/bash
x-terminal-emulator -e roslaunch kuka_arm cafe.launch &
sleep 5 &&
x-terminal-emulator -e roslaunch kuka_arm spawn_target.launch &
sleep 3 &&
x-terminal-emulator -e roslaunch kuka_arm inverse_kinematics.launch
