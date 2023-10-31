# Problem 1

### Machine setup:
1. Planner: LAMA 
    - Command line: ``lama domain.pddl problem.pddl``
    - Run on Docker image from https://hub.docker.com/r/aiplanning/planutils
2. Planner: online solver
    - Command line: ``python runnerSolverAPI.py``
    - Python is required

## Results :memo:

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