# Problem 2

### Machine setup:
1. Planner: LAMA
    - Command line ``lama domain.pddl problem.pddl``
    - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
2. Planner: online solver
    - Command line: ``python runnerSolverAPI.py``
    - Python is required. To run it move to _numeric_fluents_ folder

## Results :memo:

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