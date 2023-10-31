# Problem 5

### Machine setup:
_Please, refer to [setup](../setup.md) instructions._

## Results :memo:

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