# Problem 4

### Machine setup:
1. Planner: tfd
    - Command line ``tfd domain.pddl problem.pddl``
    - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
2. Planner: OPTIC
    - Command line: ``optic domain.pddl problem.pddl``
    - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
    - Move to _no_negative_preconditions_ folder to use it
    > OPTIC does not support :negative-preconditions


## Results :memo:

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

> Plan length: 27 steps. 

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