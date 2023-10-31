#setup bash
source /opt/ros/humble/setup.bash

# compile the repository
echo -e "\n ## Compiling the repo ## \n"
colcon build --symlink-install
rosdep install --from-paths ./ --ignore-src -r -y
colcon build --symlink-install

# run PlanSys2 framework
echo -e "\n ## Launching plansys2 ## \n"
source install/setup.bash
ros2 launch plansys2_problem5 plansys2_problem5_launch.py