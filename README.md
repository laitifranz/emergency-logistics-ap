# <p align="center"> Automated Planning: Theory and Practice - project </p>
<p align="center">
  by <a href="https://github.com/davidemodolo/">Davide Modolo</a> & <a href="https://github.com/laitifranz">Francesco Laiti</a>
  <br>
  Academic Year 2022-2023, University of Trento
</p>
<p align="center">
  <img src="https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54" alt="Python" height="25px">
  <img src="https://raw.githubusercontent.com/davidemodolo/PDDL_SVG_Icons/c978884225eca6e5e887515b6db818f94ac3cd0a/pddl.svg" alt="PDDL" height="25px">
  <img src="https://raw.githubusercontent.com/davidemodolo/PDDL_SVG_Icons/c978884225eca6e5e887515b6db818f94ac3cd0a/hddl.svg" alt="HDDL" height="25px">
  <img src="https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white" alt="docker" height="25px"/> </a> 
</p>

---

<div align="center">
    <a href=#overview-bookmark>Overview</a>
    •
    <a href=#directory-contents-evergreen_tree>Directory contents</a>
    •
    <a href=#installation-and-run-hammer>Installation & Run</a>
    •
    <a href=#results-memo>Results</a>
</div>
<br>


# Overview :bookmark:
## PDDL/HDDL for Emergency Services Logistics Problems

The goal of this project was to solve five different scenarios involving injured/needy people and delivery robots using PDDL/HDDL planners.

The scenario considered is inspired by an emergency services logistics problem. The objective is to use robotic agents to deliver boxes containing emergency supplies to injured individuals who are located in fixed positions.

The assignment is structured in 5 sub-problems, the second build on the first, both the third and the fourth build on the second, the fifth builds on the fourth.

To summarize:
- Problem 1
  - Objective: Develop a structure using PDDL.
  - Focus: A robot delivers essential items to the injured.
  - Tool: Planutils.

- Problem 2
  - Objective: Builds on top of Problem 1 with some extesions for efficient transport.
  - Tool: Planutils.
  - Versions: 2 (with and without `:numeric-fluents`)

- Problem 3
  - Objective: Address Problem 2 with hierarchical task networks (HTN)
  - Details: Introduces tasks and methods.
  - Tool: PANDA planner.

- Problem 4
  - Objective: Builds on top of Problem 2, integrating durative actions.
  - Details: Assigns durations and time constraints to actions. Goal is to minimize time.
  - Tool: Planutils.
  - Versions: 2 (with and without `:negative-preconditions`)

- Problem 5
  - Objective: Implementing Problem 4 within PlanSys using fake actions.
  - Tool: ROS2 & PlanSys2.

Some assumptions has been made, please refer to the assignment and our report. The full assignment is available [here](assignment.pdf).


# Directory contents :evergreen_tree:

This repository has been structured to accomplish the contents requested for the delivery:
- PDDL/HDDL files, organized in folders and parsable correctly by at least one planner;
- Folder [plansys_problem5](problem5/plansys_problem5) contains all the code to execute the PlanSys2 problem;
- PDF report;

Other files have been added for completeness.

Below we report the full structure of this repo. Created with `tree /F` command

```bash
│   .gitignore
│   APTP_report.pdf
│   assignment.pdf
│   README.md
│
├───problem1
│       domain.pddl
│       problem.pddl
│       results.md
│       runnerSolverAPI.py
│
├───problem2
│   │   domain.pddl
│   │   problem.pddl
│   │   problem_extra_3boxes.pddl
│   │   results.md
│   │
│   └───numeric_fluents
│           domain.pddl
│           problem.pddl
│           runnerSolverAPI.py
│
├───problem3
│       domain.hddl
│       PANDA.jar
│       problem.hddl
│       results.md
│
├───problem4
│   │   domain.pddl
│   │   problem.pddl
│   │   problem_extra_3boxes.pddl
│   │   results.md
│   │
│   └───no_negative_preconditions
│           domain.pddl
│           problem.pddl
│
└───problem5
    │   Dockerfile
    │   setup.md
    │
    └───plansys_problem5
        │   CMakeLists.txt
        │   launch_terminal1.sh
        │   launch_terminal2.sh
        │   package.xml
        │   results.md
        │
        ├───launch
        │       plansys2_problem5_launch.py
        │       problem
        │
        ├───pddl
        │       domain.pddl
        │
        └───src
                attach_carrier_to_robot_action_node.cpp
                delivery_or_refactored_possible_action1_action_node.cpp
                delivery_or_refactored_possible_action2_action_node.cpp
                deliver_action_node.cpp
                detach_carrier_from_robot_action_node.cpp
                fill_box_action_node.cpp
                load_box_on_carrier_action_node.cpp
                move_robot_action_node.cpp
                move_robot_with_carrier_action_node.cpp
                unload_box_from_carrier_action_node.cpp
```
# Installation and Run :hammer:
In every problem folder we provide a markdown with the same information collected here.
>Remember to navigate to the correct folder in order to run the correct domain, problem or config file!
## Problem 1
### Machine setup:

1. Planner: LAMA
  - Command line: ``lama domain.pddl problem.pddl``
  - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
2. Planner: online solver
  - Command line: ``python runnerSolverAPI.py``
  - Python is required

## Problem 2
### Machine setup:

1. Planner: LAMA
  - Command line ``lama domain.pddl problem.pddl``
  - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
2. Planner: online solver
  - Command line: ``python runnerSolverAPI.py``
  - Python is required. To run it move to [problem2/numeric_fluents](problem2/numeric_fluents) folder

## Problem 3
### Machine setup:

- HTN Planner: PANDA
  - Available at:
    - [Resource](https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.090/panda/PANDA.jar) from the official website;
    - [PANDA.jar](problem3/PANDA.jar) file.
- Command line ``java -jar PANDA.jar -parser hddl domain.hddl problem.hddl``
  
- Run on Docker image ``docker pull openjdk:8u342-jre``
  
  > (why version 8? the authors recommend version 8 to build the planner. Reference https://github.com/galvusdamor/panda3core)

## Problem 4
### Machine setup:

1. Planner: tfd
  
  - Command line ``tfd domain.pddl problem.pddl``
  - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
2. Planner: OPTIC
  
  - Command line: ``optic domain.pddl problem.pddl``
  - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
  - Move to [problem4/no_negative_preconditions](problem4/no_negative_preconditions) folder to use it
  
  > OPTIC does not support :negative-preconditions

## Problem 5
### How to build and run the Docker image

1. Move to folder [problem5](problem5) ``cd problem5``
2. Run ``docker build --rm  --tag ros-humble . --file Dockerfile``
3. Run ``docker run -it --name ros ros-humble bash``

### How to run PlanSys2
Two terminals of the same Docker container are needed.

> The steps provided are an adapted version of the ones provided by the official page https://plansys2.github.io/tutorials/docs/simple_example.html

#### Terminal 1
1. Move to folder [problem5/plansys_problem5](problem5/plansys_problem5) ``cd problem5/plansys_problem5``
2. Compile and launch the repo by running ``bash launch_terminal1.sh``
3. Wait until the terminal stops to print verbose

#### Terminal 2 
1. Move to folder [problem5/plansys_problem5](problem5/plansys_problem5) ``cd problem5/plansys_problem5``
2. Launch PlanSys2 terminal ``bash launch_terminal2.sh``. If you see a server error, please type ``quit`` and repeat this step
3. When ROS started, run the command ``source /root/problem5/plansys2_problem5/launch/problem 1`` to add the problem - you may need to adjust the path and here you can change the problem... and don't forget the 1 at the end!
4. Now you can inspect the goal, predicates, types... use ``help``to see the available commands
5. Creates plan and shows it ``get plan`` to retrieve a valid plan to be executed. The default planner is POPF
6. Creates plan and runs ``run`` to launch the plan! :rocket:

# Results :memo:
## Problem 1

Both planners returned the same plan.

### Solutions found

#### lama

> Plan length: 19 steps.

```bash
1. take_box robot1 depot box1 (1)
2. fill_box robot1 depot box1 food1 (1)
3. move_robot robot1 depot location2 (1)
4. deliver robot1 location2 per2 food1 box1 (1)
5. move_robot robot1 location2 depot (1)
6. take_box robot1 depot box2 (1)
7. fill_box robot1 depot box2 medicine1 (1)
8. move_robot robot1 depot location1 (1)
9. deliver robot1 location1 per1 medicine1 box2 (1)
10. take_box robot1 location1 box2 (1)
11. move_robot robot1 location1 depot (1)
12. fill_box robot1 depot box2 tools1 (1)
13. move_robot robot1 depot location1 (1)
14. deliver robot1 location1 per1 tools1 box2 (1)
15. take_box robot1 location1 box2 (1)
16. move_robot robot1 location1 depot (1)
17. fill_box robot1 depot box2 food2 (1)
18. move_robot robot1 depot location3 (1)
19. deliver robot1 location3 per3 food2 box2 (1)
```

#### planning.domains

> Plan length: 19 steps.

```bash
1. (take_box robot1 depot box2)
2. (fill_box robot1 depot box2 food1)
3. (move_robot robot1 depot location2)
4. (deliver robot1 location2 per2 food1 box2)
5. (take_box robot1 location2 box2)
6. (move_robot robot1 location2 depot)
7. (fill_box robot1 depot box2 food2)
8. (move_robot robot1 depot location3)
9. (deliver robot1 location3 per3 food2 box2)
10. (take_box robot1 location3 box2)
11. (move_robot robot1 location3 depot)
12. (fill_box robot1 depot box2 medicine1)
13. (move_robot robot1 depot location1)
14. (deliver robot1 location1 per1 medicine1 box2)
15. (take_box robot1 location1 box2)
16. (move_robot robot1 location1 depot)
17. (fill_box robot1 depot box2 tools1)
18. (move_robot robot1 depot location1)
19. (deliver robot1 location1 per1 tools1 box2)
```

## Problem 2

### Solutions found

#### lama

> Plan length: 20 steps.

```bash
1. attach_carrier_to_robot robot1 depot carrier1 (1)
2. fill_box robot1 depot box1 med1 (1)
3. load_box_on_carrier carrier1 box1 depot robot1 (1)
4. fill_box robot1 depot box2 tools1 (1)
5. fill_box robot1 depot box4 food1 (1)
6. fill_box robot1 depot box3 food2 (1)
7. load_box_on_carrier carrier1 box2 depot robot1 (1)
8. load_box_on_carrier carrier1 box3 depot robot1 (1)
9. load_box_on_carrier carrier1 box4 depot robot1 (1)
10. move_robot_with_carrier robot1 carrier1 depot location1 (1)
11. unload_box_from_carrier carrier1 box1 location1 robot1 (1)
12. unload_box_from_carrier carrier1 box2 location1 robot1 (1)
13. deliver robot1 location1 per1 med1 box1 (1)
14. deliver robot1 location1 per1 tools1 box2 (1)
15. move_robot_with_carrier robot1 carrier1 location1 location2 (1)
16. unload_box_from_carrier carrier1 box3 location2 robot1 (1)
17. deliver robot1 location2 per2 food2 box3 (1)
18. move_robot_with_carrier robot1 carrier1 location2 location3 (1)
19. unload_box_from_carrier carrier1 box4 location3 robot1 (1)
20. deliver robot1 location3 per3 food1 box4 (1)
```

#### extra with 3 boxes using lama planner

> Plan length: 23 steps.

```bash
1. attach_carrier_to_robot robot1 depot carrier1 (1)
2. fill_box robot1 depot box1 med1 (1)
3. load_box_on_carrier carrier1 box1 depot robot1 (1)
4. fill_box robot1 depot box2 food1 (1)
5. fill_box robot1 depot box3 tools1 (1)
6. load_box_on_carrier carrier1 box2 depot robot1 (1)
7. load_box_on_carrier carrier1 box3 depot robot1 (1)
8. move_robot_with_carrier robot1 carrier1 depot location1 (1)
9. unload_box_from_carrier carrier1 box3 location1 robot1 (1)
10. deliver robot1 location1 per1 tools1 box3 (1)
11. unload_box_from_carrier carrier1 box1 location1 robot1 (1)
12. deliver robot1 location1 per1 med1 box1 (1)
13. move_robot_with_carrier robot1 carrier1 location1 location2 (1)
14. unload_box_from_carrier carrier1 box2 location2 robot1 (1)
15. deliver robot1 location2 per2 food1 box2 (1)
16. load_box_on_carrier carrier1 box2 location2 robot1 (1)
17. move_robot_with_carrier robot1 carrier1 location2 depot (1)
18. unload_box_from_carrier carrier1 box2 depot robot1 (1)
19. fill_box robot1 depot box2 food2 (1)
20. load_box_on_carrier carrier1 box2 depot robot1 (1)
21. move_robot_with_carrier robot1 carrier1 depot location3 (1)
22. unload_box_from_carrier carrier1 box2 location3 robot1 (1)
23. deliver robot1 location3 per3 food2 box2 (1)
```

#### numeric fluents

Using `planner.domains` planner.

> Plan length: 28 steps.

```bash
1. (move_robot robot1 location0 location1)
2. (attach_carrier_to_robot robot1 location1 carrier1)
3. (move_robot_with_carrier robot1 location1 location0 carrier1)
4. (fill_box robot1 location0 box1 med1)
5. (fill_box robot1 location0 box4 food1)
6. (fill_box robot1 location0 box3 food2)
7. (fill_box robot1 location0 box2 tools1)
8. (load_box_on_carrier carrier1 box3 location0 robot1)
9. (move_robot_with_carrier robot1 location0 location2 carrier1)
10. (unload_box_from_carrier carrier1 box3 location2 robot1)
11. (move_robot_with_carrier robot1 location2 location0 carrier1)
12. (load_box_on_carrier carrier1 box4 location0 robot1)
13. (move_robot_with_carrier robot1 location0 location3 carrier1)
14. (unload_box_from_carrier carrier1 box4 location3 robot1)
15. (move_robot_with_carrier robot1 location3 location0 carrier1)
16. (load_box_on_carrier carrier1 box1 location0 robot1)
17. (move_robot_with_carrier robot1 location0 location3 carrier1)
18. (deliver robot1 location3 per3 food1 box4)
19. (move_robot_with_carrier robot1 location3 location2 carrier1)
20. (deliver robot1 location2 per2 food2 box3)
21. (move_robot_with_carrier robot1 location2 location1 carrier1)
22. (unload_box_from_carrier carrier1 box1 location1 robot1)
23. (deliver robot1 location1 per1 med1 box1)
24. (move_robot_with_carrier robot1 location1 location0 carrier1)
25. (load_box_on_carrier carrier1 box2 location0 robot1)
26. (move_robot_with_carrier robot1 location0 location1 carrier1)
27. (unload_box_from_carrier carrier1 box2 location1 robot1)
28. (deliver robot1 location1 per1 tools1 box2)
```

# Problem 3

### Solution found

> Plan length: 26 actions.

#### Actions

```bash
SOLUTION SEQUENCE
0: attach_carrier_to_robot(robot1,depot,carrier1)
1: fill_box(robot1,depot,box1,med1)
2: load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT(carrier1,box1,depot,robot1)
3: move_robot_with_carrier(robot1,carrier1,depot,location1)
4: unload_box_from_carrier__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT(carrier1,box1,location1,robot1)
5: deliver(robot1,location1,per1,med1,box1)
6: load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT(carrier1,box1,location1,robot1)
7: move_robot_with_carrier(robot1,carrier1,location1,depot)
8: fill_box(robot1,depot,box3,tools1)
9: load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT(carrier1,box3,depot,robot1)
10: move_robot_with_carrier(robot1,carrier1,depot,location1)
11: unload_box_from_carrier__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT(carrier1,box3,location1,robot1)
12: deliver(robot1,location1,per1,tools1,box3)
13: load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT(carrier1,box3,location1,robot1)
14: move_robot_with_carrier(robot1,carrier1,location1,depot)
15: fill_box(robot1,depot,box4,food2)
16: load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT(carrier1,box4,depot,robot1)
17: move_robot_with_carrier(robot1,carrier1,depot,location2)
18: unload_box_from_carrier__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT(carrier1,box4,location2,robot1)
19: deliver(robot1,location2,per2,food2,box4)
20: load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT(carrier1,box4,location2,robot1)
21: move_robot_with_carrier(robot1,carrier1,location2,depot)
22: fill_box(robot1,depot,box2,food1)
23: load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT__ANTECEDENT__CONSEQUENT__(carrier1,box2,depot,robot1)
24: move_robot_with_carrier(robot1,carrier1,depot,location3)
25: unload_box_from_carrier__ANTECEDENT__ANTECEDENT__ANTECEDENT__CONSEQUENT__(carrier1,box2,location3,robot1)
26: deliver(robot1,location3,per3,food1,box2)
```

#### Methods

```bash
Found a solution:
__top_4 @ __artificialTopCompilation__top_4
M_deliver_supply_but_first_attach_carrier[?r=robot1,?p=per1,?l2=location3,?c=carrier1,?b=box2,?s=med1,?l1=depot] @ T_deliver_supply[per1,med1]
M_attach_carrier_to_robot[?r=robot1,?l=depot,?c=carrier1] @ T_attach_carrier_to_robot[robot1,carrier1]
attach_carrier_to_robot[robot1,depot,carrier1]
M_deliver_supply_with_already_carrier[?s=med1,?p=per1,?r=robot1,?c=carrier1] @ T_deliver_supply[per1,med1]
M_prepare_box[?b=box1,?c=carrier1,?s=med1,?l=depot,?r=robot1] @ T_prepare_box[robot1,med1]
fill_box[robot1,depot,box1,med1]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box1,?l=depot] @ T_load_box_on_carrier[robot1,box1]
M-load_box_on_carrier__DISJUNCT-0[?r=robot1,?c=carrier1,?l=depot,?b=box1] @ load_box_on_carrier[carrier1,box1,depot,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT__[?r=robot1,?c=carrier1,?l=depot,?b=box1] @ load_box_on_carrier__DISJUNCT-0[carrier1,box1,depot,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box1] @ load_box_on_carrier__DISJUNCT-0__CONSEQUENT__[carrier1,box1,depot,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box1] @ load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT[carrier1,box1,depot,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box1] @ load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT[carrier1,box1,depot,robot1]
load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT[carrier1,box1,depot,robot1]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_3[?l2=location1,?l1=depot,?r=robot1] @ T_move_robot_M_deliver_supply_with_already_carrier_3[robot1]
M_move_robot_with_carrier[?l2=location1,?c=carrier1,?r=robot1,?l1=depot] @ T_move_robot[robot1,depot,location1]
move_robot_with_carrier[robot1,carrier1,depot,location1]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_4[?b=box1,?r=robot1] @ T_unload_box_from_carrier_M_deliver_supply_with_already_carrier_4[robot1]
M_unload_box_from_carrier[?r=robot1,?l=location1,?c=carrier1,?b=box1] @ T_unload_box_from_carrier[robot1,box1]
M-unload_box_from_carrier__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ unload_box_from_carrier[carrier1,box1,location1,robot1]
M-unload_box_from_carrier__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ unload_box_from_carrier__CONSEQUENT__[carrier1,box1,location1,robot1]
M-unload_box_from_carrier__CONSEQUENT____ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ unload_box_from_carrier__CONSEQUENT____ANTECEDENT[carrier1,box1,location1,robot1]
M-unload_box_from_carrier__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ unload_box_from_carrier__CONSEQUENT____ANTECEDENT__ANTECEDENT[carrier1,box1,location1,robot1]
unload_box_from_carrier__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT[carrier1,box1,location1,robot1]
M_deliver[?b=box1,?p=per1,?r=robot1,?s=med1,?l=location1] @ T_deliver[per1,med1]
deliver[robot1,location1,per1,med1,box1]
M_deliver_supply_by_loading_supply_from_depot[?c=carrier1,?r=robot1,?l2=location3,?p=per1,?s=tools1,?l1=location2,?b=box3] @ T_deliver_supply[per1,tools1]
M_return_to_depot_M_return_to_depot_2[?r=robot1,?b=box1] @ T_load_box_on_carrier_M_return_to_depot_2[robot1]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box1,?l=location1] @ T_load_box_on_carrier[robot1,box1]
M-load_box_on_carrier__DISJUNCT-0[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ load_box_on_carrier[carrier1,box1,location1,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ load_box_on_carrier__DISJUNCT-0[carrier1,box1,location1,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ load_box_on_carrier__DISJUNCT-0__CONSEQUENT__[carrier1,box1,location1,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT[carrier1,box1,location1,robot1]
M-load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box1] @ load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT[carrier1,box1,location1,robot1]
load_box_on_carrier__DISJUNCT-0__CONSEQUENT____ANTECEDENT__ANTECEDENT__ANTECEDENT[carrier1,box1,location1,robot1]
M_return_to_depot_M_return_to_depot_3[?r=robot1,?l2=depot,?l1=location1] @ T_move_robot_M_return_to_depot_3[robot1]
M_move_robot_with_carrier[?l2=depot,?c=carrier1,?r=robot1,?l1=location1] @ T_move_robot[robot1,location1,depot]
move_robot_with_carrier[robot1,carrier1,location1,depot]
M_deliver_supply_with_already_carrier[?s=tools1,?p=per1,?r=robot1,?c=carrier1] @ T_deliver_supply[per1,tools1]
M_prepare_box[?b=box3,?c=carrier1,?s=tools1,?l=location1,?r=robot1] @ T_prepare_box[robot1,tools1]
fill_box[robot1,depot,box3,tools1]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box3,?l=depot] @ T_load_box_on_carrier[robot1,box3]
M-load_box_on_carrier__DISJUNCT-1[?r=robot1,?c=carrier1,?l=depot,?b=box3] @ load_box_on_carrier[carrier1,box3,depot,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box3] @ load_box_on_carrier__DISJUNCT-1[carrier1,box3,depot,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=depot,?b=box3] @ load_box_on_carrier__DISJUNCT-1__ANTECEDENT[carrier1,box3,depot,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box3] @ load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT__[carrier1,box3,depot,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box3] @ load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT[carrier1,box3,depot,robot1]
load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT[carrier1,box3,depot,robot1]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_3[?l2=location1,?l1=depot,?r=robot1] @ T_move_robot_M_deliver_supply_with_already_carrier_3[robot1]
M_move_robot_with_carrier[?l2=location1,?c=carrier1,?r=robot1,?l1=depot] @ T_move_robot[robot1,depot,location1]
move_robot_with_carrier[robot1,carrier1,depot,location1]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_4[?b=box3,?r=robot1] @ T_unload_box_from_carrier_M_deliver_supply_with_already_carrier_4[robot1]
M_unload_box_from_carrier[?r=robot1,?l=location1,?c=carrier1,?b=box3] @ T_unload_box_from_carrier[robot1,box3]
M-unload_box_from_carrier__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ unload_box_from_carrier[carrier1,box3,location1,robot1]
M-unload_box_from_carrier__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ unload_box_from_carrier__ANTECEDENT[carrier1,box3,location1,robot1]
M-unload_box_from_carrier__ANTECEDENT__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ unload_box_from_carrier__ANTECEDENT__CONSEQUENT__[carrier1,box3,location1,robot1]
M-unload_box_from_carrier__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ unload_box_from_carrier__ANTECEDENT__CONSEQUENT____ANTECEDENT[carrier1,box3,location1,robot1]
unload_box_from_carrier__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT[carrier1,box3,location1,robot1]
M_deliver[?b=box3,?p=per1,?r=robot1,?s=tools1,?l=location1] @ T_deliver[per1,tools1]
deliver[robot1,location1,per1,tools1,box3]
M_food_delivery_satisfied_method2[?s2=food2,?p1=per2,?s1=food1,?p2=per3] @ T_food_delivery_satisfied[per2,per3,food1,food2]
M_deliver_supply_by_loading_supply_from_depot[?c=carrier1,?r=robot1,?l2=depot,?p=per2,?s=food2,?l1=location1,?b=box4] @ T_deliver_supply[per2,food2]
M_return_to_depot_M_return_to_depot_2[?r=robot1,?b=box3] @ T_load_box_on_carrier_M_return_to_depot_2[robot1]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box3,?l=location1] @ T_load_box_on_carrier[robot1,box3]
M-load_box_on_carrier__DISJUNCT-1[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ load_box_on_carrier[carrier1,box3,location1,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ load_box_on_carrier__DISJUNCT-1[carrier1,box3,location1,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ load_box_on_carrier__DISJUNCT-1__ANTECEDENT[carrier1,box3,location1,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT__[carrier1,box3,location1,robot1]
M-load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location1,?b=box3] @ load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT[carrier1,box3,location1,robot1]
load_box_on_carrier__DISJUNCT-1__ANTECEDENT__CONSEQUENT____ANTECEDENT__ANTECEDENT[carrier1,box3,location1,robot1]
M_return_to_depot_M_return_to_depot_3[?r=robot1,?l2=depot,?l1=location1] @ T_move_robot_M_return_to_depot_3[robot1]
M_move_robot_with_carrier[?l2=depot,?c=carrier1,?r=robot1,?l1=location1] @ T_move_robot[robot1,location1,depot]
move_robot_with_carrier[robot1,carrier1,location1,depot]
M_deliver_supply_with_already_carrier[?s=food2,?p=per2,?r=robot1,?c=carrier1] @ T_deliver_supply[per2,food2]
M_prepare_box[?b=box4,?c=carrier1,?s=food2,?l=location2,?r=robot1] @ T_prepare_box[robot1,food2]
fill_box[robot1,depot,box4,food2]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box4,?l=depot] @ T_load_box_on_carrier[robot1,box4]
M-load_box_on_carrier__DISJUNCT-2[?r=robot1,?c=carrier1,?l=depot,?b=box4] @ load_box_on_carrier[carrier1,box4,depot,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box4] @ load_box_on_carrier__DISJUNCT-2[carrier1,box4,depot,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box4] @ load_box_on_carrier__DISJUNCT-2__ANTECEDENT[carrier1,box4,depot,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=depot,?b=box4] @ load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT[carrier1,box4,depot,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box4] @ load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT__[carrier1,box4,depot,robot1]
load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT[carrier1,box4,depot,robot1]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_3[?l2=location2,?l1=depot,?r=robot1] @ T_move_robot_M_deliver_supply_with_already_carrier_3[robot1]
M_move_robot_with_carrier[?l2=location2,?c=carrier1,?r=robot1,?l1=depot] @ T_move_robot[robot1,depot,location2]
move_robot_with_carrier[robot1,carrier1,depot,location2]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_4[?b=box4,?r=robot1] @ T_unload_box_from_carrier_M_deliver_supply_with_already_carrier_4[robot1]
M_unload_box_from_carrier[?r=robot1,?l=location2,?c=carrier1,?b=box4] @ T_unload_box_from_carrier[robot1,box4]
M-unload_box_from_carrier__ANTECEDENT[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ unload_box_from_carrier[carrier1,box4,location2,robot1]
M-unload_box_from_carrier__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ unload_box_from_carrier__ANTECEDENT[carrier1,box4,location2,robot1]
M-unload_box_from_carrier__ANTECEDENT__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ unload_box_from_carrier__ANTECEDENT__ANTECEDENT[carrier1,box4,location2,robot1]
M-unload_box_from_carrier__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ unload_box_from_carrier__ANTECEDENT__ANTECEDENT__CONSEQUENT__[carrier1,box4,location2,robot1]
unload_box_from_carrier__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT[carrier1,box4,location2,robot1]
M_deliver[?b=box4,?p=per2,?r=robot1,?s=food2,?l=location2] @ T_deliver[per2,food2]
deliver[robot1,location2,per2,food2,box4]
M_deliver_supply_by_loading_supply_from_depot[?c=carrier1,?r=robot1,?l2=location2,?p=per3,?s=food1,?l1=location1,?b=box3] @ T_deliver_supply[per3,food1]
M_return_to_depot_M_return_to_depot_2[?r=robot1,?b=box4] @ T_load_box_on_carrier_M_return_to_depot_2[robot1]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box4,?l=location2] @ T_load_box_on_carrier[robot1,box4]
M-load_box_on_carrier__DISJUNCT-2[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ load_box_on_carrier[carrier1,box4,location2,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ load_box_on_carrier__DISJUNCT-2[carrier1,box4,location2,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ load_box_on_carrier__DISJUNCT-2__ANTECEDENT[carrier1,box4,location2,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT[carrier1,box4,location2,robot1]
M-load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT[?r=robot1,?c=carrier1,?l=location2,?b=box4] @ load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT__[carrier1,box4,location2,robot1]
load_box_on_carrier__DISJUNCT-2__ANTECEDENT__ANTECEDENT__CONSEQUENT____ANTECEDENT[carrier1,box4,location2,robot1]
M_return_to_depot_M_return_to_depot_3[?r=robot1,?l2=depot,?l1=location2] @ T_move_robot_M_return_to_depot_3[robot1]
M_move_robot_with_carrier[?l2=depot,?c=carrier1,?r=robot1,?l1=location2] @ T_move_robot[robot1,location2,depot]
move_robot_with_carrier[robot1,carrier1,location2,depot]
M_deliver_supply_with_already_carrier[?s=food1,?p=per3,?r=robot1,?c=carrier1] @ T_deliver_supply[per3,food1]
M_prepare_box[?b=box2,?c=carrier1,?s=food1,?l=location1,?r=robot1] @ T_prepare_box[robot1,food1]
fill_box[robot1,depot,box2,food1]
M_load_box_on_carrier[?c=carrier1,?r=robot1,?b=box2,?l=depot] @ T_load_box_on_carrier[robot1,box2]
M-load_box_on_carrier__DISJUNCT-3[?r=robot1,?c=carrier1,?l=depot,?b=box2] @ load_box_on_carrier[carrier1,box2,depot,robot1]
M-load_box_on_carrier__DISJUNCT-3__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box2] @ load_box_on_carrier__DISJUNCT-3[carrier1,box2,depot,robot1]
M-load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box2] @ load_box_on_carrier__DISJUNCT-3__ANTECEDENT[carrier1,box2,depot,robot1]
M-load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=depot,?b=box2] @ load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT[carrier1,box2,depot,robot1]
M-load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=depot,?b=box2] @ load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT__ANTECEDENT[carrier1,box2,depot,robot1]
load_box_on_carrier__DISJUNCT-3__ANTECEDENT__ANTECEDENT__ANTECEDENT__CONSEQUENT__[carrier1,box2,depot,robot1]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_3[?l2=location3,?l1=depot,?r=robot1] @ T_move_robot_M_deliver_supply_with_already_carrier_3[robot1]
M_move_robot_with_carrier[?l2=location3,?c=carrier1,?r=robot1,?l1=depot] @ T_move_robot[robot1,depot,location3]
move_robot_with_carrier[robot1,carrier1,depot,location3]
M_deliver_supply_with_already_carrier_M_deliver_supply_with_already_carrier_4[?b=box2,?r=robot1] @ T_unload_box_from_carrier_M_deliver_supply_with_already_carrier_4[robot1]
M_unload_box_from_carrier[?r=robot1,?l=location3,?c=carrier1,?b=box2] @ T_unload_box_from_carrier[robot1,box2]
M-unload_box_from_carrier__ANTECEDENT[?r=robot1,?c=carrier1,?l=location3,?b=box2] @ unload_box_from_carrier[carrier1,box2,location3,robot1]
M-unload_box_from_carrier__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location3,?b=box2] @ unload_box_from_carrier__ANTECEDENT[carrier1,box2,location3,robot1]
M-unload_box_from_carrier__ANTECEDENT__ANTECEDENT__ANTECEDENT[?r=robot1,?c=carrier1,?l=location3,?b=box2] @ unload_box_from_carrier__ANTECEDENT__ANTECEDENT[carrier1,box2,location3,robot1]
M-unload_box_from_carrier__ANTECEDENT__ANTECEDENT__ANTECEDENT__CONSEQUENT__[?r=robot1,?c=carrier1,?l=location3,?b=box2] @ unload_box_from_carrier__ANTECEDENT__ANTECEDENT__ANTECEDENT[carrier1,box2,location3,robot1]
unload_box_from_carrier__ANTECEDENT__ANTECEDENT__ANTECEDENT__CONSEQUENT__[carrier1,box2,location3,robot1]
M_deliver[?b=box2,?p=per3,?r=robot1,?s=food1,?l=location3] @ T_deliver[per3,food1]
deliver[robot1,location3,per3,food1,box2]
```

# Problem 4

### Solution found

#### tfd

> Plan length: 26 steps.

> Sub-optimal solution because, if we apply the condition to not load a box that has been already used for delivery, the final plan is 25 steps long. But using the latter case broke the extra problem because we do not give the possibility to reuse the boxes.

```bash
- 0.00100000: (attach_carrier_to_robot robot1 depot carrier1) [2.00000000]
- 2.01100000: (fill_box robot1 depot box4 med1) [4.00000000]
- 6.02100000: (load_box_on_carrier carrier1 box4 depot robot1) [3.00000000]
- 9.03100000: (move_robot_with_carrier robot1 carrier1 depot location1) [7.00000000]
- 16.04100000: (unload_box_from_carrier carrier1 box4 location1 robot1) [3.00000000]
- 19.05100000: (deliver robot1 location1 per1 med1 box4) [3.00000000]
- 22.06100000: (load_box_on_carrier carrier1 box4 location1 robot1) [3.00000000]
- 25.07100000: (move_robot_with_carrier robot1 carrier1 location1 depot) [7.00000000]
- 32.08100000: (fill_box robot1 depot box3 tools1) [4.00000000]
- 36.09100000: (load_box_on_carrier carrier1 box3 depot robot1) [3.00000000]
- 39.10100000: (move_robot_with_carrier robot1 carrier1 depot location1) [7.00000000]
- 46.11100000: (unload_box_from_carrier carrier1 box3 location1 robot1) [3.00000000]
- 49.12100000: (deliver robot1 location1 per1 tools1 box3) [3.00000000]
- 52.13100000: (move_robot_with_carrier robot1 carrier1 location1 depot) [7.00000000]
- 59.14100000: (fill_box robot1 depot box2 food1) [4.00000000]
- 59.15100000: (fill_box robot1 depot box1 food2) [4.00000000]
- 63.16100000: (load_box_on_carrier carrier1 box1 depot robot1) [3.00000000]
- 63.17100000: (load_box_on_carrier carrier1 box2 depot robot1) [3.00000000]
- 66.18100000: (move_robot_with_carrier robot1 carrier1 depot location2) [7.00000000]
- 73.19100000: (unload_box_from_carrier carrier1 box1 location2 robot1) [3.00000000]
- 76.20100000: (move_robot_with_carrier robot1 carrier1 location2 location3) [7.00000000]
- 83.21100000: (unload_box_from_carrier carrier1 box2 location3 robot1) [3.00000000]
- 86.22100000: (deliver robot1 location3 per3 food1 box2) [3.00000000]
- 89.23100000: (move_robot_with_carrier robot1 carrier1 location3 location2) [7.00000000]
- 96.24100000: (deliver robot1 location2 per2 food2 box1) [3.00000000]
- 99.25100000: (delivery_or_refactored_possible_action2 per2 per3 food1 food2) [0.10000000]
```

#### extra with 3 boxes using tfd planner

> Plan length: 27 steps.

```bash
- 0.00100000: (attach_carrier_to_robot robot1 depot carrier1) [2.00000000]
- 2.01100000: (fill_box robot1 depot box2 food1) [4.00000000]
- 6.02100000: (load_box_on_carrier carrier1 box2 depot robot1) [3.00000000]
- 9.03100000: (move_robot_with_carrier robot1 carrier1 depot location2) [7.00000000]
- 16.04100000: (unload_box_from_carrier carrier1 box2 location2 robot1) [3.00000000]
- 19.05100000: (deliver robot1 location2 per2 food1 box2) [3.00000000]
- 22.06100000: (load_box_on_carrier carrier1 box2 location2 robot1) [3.00000000]
- 25.07100000: (move_robot_with_carrier robot1 carrier1 location2 depot) [7.00000000]
- 32.08100000: (unload_box_from_carrier carrier1 box2 depot robot1) [3.00000000]
- 35.09100000: (fill_box robot1 depot box2 food2) [4.00000000]
- 39.10100000: (load_box_on_carrier carrier1 box2 depot robot1) [3.00000000]
- 42.11100000: (move_robot_with_carrier robot1 carrier1 depot location3) [7.00000000]
- 49.12100000: (unload_box_from_carrier carrier1 box2 location3 robot1) [3.00000000]
- 52.13100000: (deliver robot1 location3 per3 food2 box2) [3.00000000]
- 55.14100000: (load_box_on_carrier carrier1 box2 location3 robot1) [3.00000000]
- 58.15100000: (move_robot_with_carrier robot1 carrier1 location3 depot) [7.00000000]
- 65.16100000: (fill_box robot1 depot box3 med1) [4.00000000]
- 69.17100000: (load_box_on_carrier carrier1 box3 depot robot1) [3.00000000]
- 72.18100000: (move_robot_with_carrier robot1 carrier1 depot location1) [7.00000000]
- 79.19100000: (unload_box_from_carrier carrier1 box3 location1 robot1) [3.00000000]
- 82.20100000: (deliver robot1 location1 per1 med1 box3) [3.00000000]
- 85.21100000: (move_robot_with_carrier robot1 carrier1 location1 depot) [7.00000000]
- 92.22100000: (fill_box robot1 depot box1 tools1) [4.00000000]
- 96.23100000: (load_box_on_carrier carrier1 box1 depot robot1) [3.00000000]
- 99.24100000: (move_robot_with_carrier robot1 carrier1 depot location1) [7.00000000]
- 106.25100000: (unload_box_from_carrier carrier1 box1 location1 robot1) [3.00000000]
- 109.26100000: (deliver robot1 location1 per1 tools1 box1) [3.00000000]
```

#### without negative preconditions using OPTIC

> Plan length: 23 steps.

```bash
- 0.000: (attach_carrier_to_robot robot1 depot carrier1)  [2.000]
- 2.001: (fill_box robot1 depot box1 food2 carrier1)  [4.000]
- 6.002: (load_box_on_carrier carrier1 box1 depot robot1)  [3.000]
- 9.002: (move_robot_with_carrier robot1 carrier1 depot location3)  [7.000]
- 16.002: (unload_box_from_carrier carrier1 box1 location3 robot1)  [3.000]
- 16.002: (deliver robot1 location3 per3 food2 box1)  [3.000]
- 19.002: (move_robot_with_carrier robot1 carrier1 location3 depot)  [7.000]
- 26.002: (fill_box robot1 depot box2 food1 carrier1)  [4.000]
- 30.002: (load_box_on_carrier carrier1 box2 depot robot1)  [3.000]
- 33.002: (move_robot_with_carrier robot1 carrier1 depot location2)  [7.000]
- 40.002: (unload_box_from_carrier carrier1 box2 location2 robot1)  [3.000]
- 40.002: (deliver robot1 location2 per2 food1 box2)  [3.000]
- 43.002: (move_robot_with_carrier robot1 carrier1 location2 depot)  [7.000]
- 50.002: (fill_box robot1 depot box3 tools1 carrier1)  [4.000]
- 50.002: (fill_box robot1 depot box4 med1 carrier1)  [4.000]
- 54.002: (load_box_on_carrier carrier1 box4 depot robot1)  [3.000]
- 54.003: (load_box_on_carrier carrier1 box3 depot robot1)  [3.000]
- 57.003: (move_robot_with_carrier robot1 carrier1 depot location1)  [7.000]
- 64.003: (unload_box_from_carrier carrier1 box4 location1 robot1)  [3.000]
- 64.004: (unload_box_from_carrier carrier1 box3 location1 robot1)  [3.000]
- 64.004: (deliver robot1 location1 per1 med1 box4)  [3.000]
- 64.004: (deliver robot1 location1 per1 tools1 box3)  [3.000]
- 66.904: (delivery_or_refactored_possible_action1 per2 per3 food1 food2)  [0.100]
```

# Problem 5

### Solution found

#### POPF planner output

> Plan length: 25 steps.

```bash
- 0:      (attach_carrier_to_robot robot1 depot carrier1) [2]
- 2.001:  (fill_box robot1 depot box1 food1 carrier1)     [4]
- 6.002:  (load_box_on_carrier carrier1 box1 depot robot1)        [3]
- 9.002:  (move_robot_with_carrier robot1 carrier1 depot location3)       [7]
- 16.002: (unload_box_from_carrier carrier1 box1 location3 robot1)        [3]
- 16.002: (deliver robot1 location3 per3 food1 box1)      [3]
- 19.002: (move_robot_with_carrier robot1 carrier1 location3 depot)       [7]
- 26.002: (fill_box robot1 depot box2 food2 carrier1)     [4]
- 30.002: (load_box_on_carrier carrier1 box2 depot robot1)        [3]
- 33.002: (move_robot_with_carrier robot1 carrier1 depot location2)       [7]
- 40.002: (unload_box_from_carrier carrier1 box2 location2 robot1)        [3]
- 40.002: (deliver robot1 location2 per2 food2 box2)      [3]
- 43.002: (move_robot_with_carrier robot1 carrier1 location2 depot)       [7]
- 43.003: (delivery_or_refactored_possible_action2 per2 per3 food1 food2) [0.1]
- 50.002: (fill_box robot1 depot box3 med1 carrier1)      [4]
- 54.002: (load_box_on_carrier carrier1 box3 depot robot1)        [3]
- 57.002: (move_robot_with_carrier robot1 carrier1 depot location1)       [7]
- 64.002: (unload_box_from_carrier carrier1 box3 location1 robot1)        [3]
- 64.002: (deliver robot1 location1 per1 med1 box3)       [3]
- 67.002: (move_robot_with_carrier robot1 carrier1 location1 depot)       [7]
- 74.002: (fill_box robot1 depot box4 tools1 carrier1)    [4]
- 78.002: (load_box_on_carrier carrier1 box4 depot robot1)        [3]
- 81.002: (move_robot_with_carrier robot1 carrier1 depot location1)       [7]
- 88.002: (unload_box_from_carrier carrier1 box4 location1 robot1)        [3]
- 88.002: (deliver robot1 location1 per1 tools1 box4)     [3]
```

#### PlanSys execution

```bash
- Attaching carrier to robot ... [100%]  
- Filling box ... [100%]  
- Loading box ... [100%]  
- Moving robot with carrier ... [100%]  
- Unloading box ... [100%]  
- Delivering to person ... [100%]  
- Moving robot with carrier ... [100%]  
- Filling box ... [100%]  
- Loading box ... [100%]  
- Moving robot with carrier ... [100%]  
- Unloading box ... [100%]  
- Delivering to person ... [100%]  
- Delivering supply ... [100%]  
- Moving robot with carrier ... [100%]  
- Filling box ... [100%]  
- Loading box ... [100%]  
- Moving robot with carrier ... [100%]  
- Unloading box ... [100%]  
- Delivering to person ... [100%]  
- Moving robot with carrier ... [100%]  
- Filling box ... [100%]  
- Loading box ... [100%]  
- Moving robot with carrier ... [100%]  
- Unloading box ... [100%]  
- Delivering to person ... [100%] 
```