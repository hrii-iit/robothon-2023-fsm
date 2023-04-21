# Robothon 2023
This repo contains the instructions to run the framework developed for the Robothon Grand Challenge 2023.

# Prerequisites
Check that the correct version of libfranka is installed:
```bash
cd ~/git/hrii_gitlab/robotics/franka/libfranka
```
If the selected branch is not "0.9.2" delete the build folder and execute this commands:
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```

_Note that to make the other hrii sw work you may need to recompile libfranka in the noetic-devel branch_
# Installation instructions
Install the *hrii_task_board_fsm* package and its dependencies within your *catkin_ws* source folder. To do so, first download the repos <a href="https://gitlab.iit.it/hrii/projects/robothon/hrii_task_board_fsm/-/raw/main/config/robothon_repos.yaml?inline=false" target="_blank">config file</a>.

Then enter the following commands:
```bash
create_catkin_ws
git_import_repos robothon_repos.yaml
cd $WORKSPACE_TO_SOURCE
rosdep_src_install
cb_full
```

# Usage
Test that everything works fine, launching the following command:
```bash
roslaunch hrii_task_board_fsm main_fsm.launch

# if you want to run a fake perception node
roslaunch hrii_task_board_fsm fake_perception.launch
```

# Tips
In the file config/fsm/default_task_order.yaml you can select the task execution order.
You can also define a customized file that will be ignored by git, just create it in the same folder under the name _custom_task_order.yaml_

To move the robot to a decent initial joint configuration run the command:
```bash
mon launch franka_example_controllers move_to_start.launch robot_ip:=192.168.0.102
```