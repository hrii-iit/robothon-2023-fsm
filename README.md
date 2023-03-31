# Finite State Machine template
Define your states, target poses and let the FSM do the job for you!
Finite State Machine template for a generic robot, to test the example provided hereafter you need to install also some other repos that are included in the config file listed belows (Option 1, 2, 3, and 4).

# Installation instructions
Install the *hrii_dummy_fsm* package and its dependencies within your *catkin_ws* source folder. To do so, first download the config file you are interested in:
- <a href="https://gitlab.iit.it/hrii/robotics/common/hrii_dummy_fsm/-/raw/noetic-devel/config/dummy_fixed_manipulator_repos.yaml?inline=false" target="_blank">Option 1: fixed manipulator (Franka)</a> 
- <a href="https://gitlab.iit.it/hrii/robotics/common/hrii_dummy_fsm/-/raw/noetic-devel/config/dummy_mobile_manipulator_repos.yaml?inline=false" target="_blank">Option 2: mobile manipulator (MOCA)</a> 
- <a href="https://gitlab.iit.it/hrii/robotics/common/hrii_dummy_fsm/-/raw/noetic-devel/config/dummy_mobile_manipulator_rbkairos_repos.yaml?inline=false" target="_blank">Option 3: mobile manipulator (RB-KAIROS)</a> 
- <a href="https://gitlab.iit.it/hrii/robotics/common/hrii_dummy_fsm/-/raw/noetic-devel/config/dummy_mobile_manipulator_autonomous_navigation_repos.yaml?inline=false" target="_blank">Option 4: mobile manipulator autonomous navigation (RB-KAIROS)</a> 
- <a href="https://gitlab.iit.it/hrii/robotics/common/hrii_dummy_fsm/-/raw/noetic-devel/config/dummy_fixed_position_control.yaml?inline=false" target="_blank">Option 5: fixed manipulator position control (Franka)</a> 


Then enter the following commands:
```bash
create_catkin_ws

#Option 1: fixed manipulator (Franka)
git_import_repos dummy_fixed_manipulator_repos.yaml
#Option 2: mobile manipulator (MOCA)
git_import_repos dummy_mobile_manipulator_repos.yaml
#Option 3: mobile manipulator (RB-KAIROS)
git_import_repos dummy_mobile_manipulator_rbkairos_repos.yaml
#Option 4: mobile manipulator autonomous navigation (RB-KAIROS)
git_import_repos dummy_mobile_manipulator_autonomous_navigation_repos.yaml
#Option 5: fixed manipulator position control (Franka)
git_import_repos dummy_fixed_position_control.yaml

cd $WORKSPACE_TO_SOURCE
rosdep_src_install
cb_full #or cm_full if you use catkin make
```

# Usage
Test that everything works fine, launching the following command:
```bash
#Option 1: fixed manipulator (Franka)
roslaunch hrii_dummy_fsm dummy_fixed_manipulator.launch
#Option 2: mobile manipulator (MOCA)
roslaunch hrii_dummy_fsm dummy_mobile_manipulator.launch
#Option 3: mobile manipulator (RB-KAIROS)
roslaunch hrii_dummy_fsm dummy_mobile_manipulator_rbkairos.launch #see instructions at the top of this file on how to move the robot
#Option 4: mobile manipulator autonomous navigation (RB-KAIROS)
roslaunch hrii_dummy_fsm dummy_mobile_platform_autonomous_navigation.launch
#Option 5: fixed manipulator position control (Franka)
roslaunch hrii_dummy_fsm dummy_fsm_franka_position_control.launch #not currently working in simulation
```
You should see the robot following a predefined trajectory in Gazebo (if you are running it in simulation) or in the real world!

### Troubleshooting
If in simulation you don't see the robot moving, it's because you need to press play in Gazebo. This is necessary to achieve stability at the simulation launching step.
