# Installation and usage of the environment for PlanSys2

## How to build and run the Docker image

1. Move to folder [problem5](problem5) ``cd problem5``
2. Run ``docker build --rm  --tag ros-humble . --file Dockerfile``
3. Run ``docker run -it --name ros ros-humble bash``

## How to run PlanSys2
Two terminals of the same Docker container are needed.

> The steps provided are an adapted version of the ones provided by the official page https://plansys2.github.io/tutorials/docs/simple_example.html

### Terminal 1
1. Move to folder [problem5/plansys_problem5](problem5/plansys_problem5) ``cd problem5/plansys_problem5``
2. Compile and launch the repo by running ``bash launch_terminal1.sh``
3. Wait until the terminal stops to print verbose

### Terminal 2 
1. Move to folder [problem5/plansys_problem5](problem5/plansys_problem5) ``cd problem5/plansys_problem5``
2. Launch PlanSys2 terminal ``bash launch_terminal2.sh``. If you see a server error, please type ``quit`` and repeat this step
3. When ROS started, run the command ``source /root/plansys2_problem5/launch/problem 1`` to add the problem - you may need to adjust the path and here you can change the problem... and don't forget the 1 at the end!
4. Now you can inspect the goal, predicates, types... use ``help``to see the available commands
5. Creates plan and shows it ``get plan`` to retrieve a valid plan to be executed. The default planner is POPF
6. Creates plan and runs ``run`` to launch the plan! :rocket: