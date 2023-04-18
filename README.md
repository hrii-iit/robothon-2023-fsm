# Robothon 2023
This repo contains the instructions to run the framework developed for the Robothon Grand Challenge 2023.

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
