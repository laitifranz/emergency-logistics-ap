# Problem 3

### Machine setup:
- HTN Planner: PANDA
    - Available at:
        - [Resource](https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.090/panda/PANDA.jar) from the official website;
        - [PANDA.jar](PANDA.jar) file inside this folder.
- Command line ``java -jar PANDA.jar -parser hddl domain.hddl problem.hddl``
- Run on Docker image ``docker pull openjdk:8u342-jre`` 
    >(why version 8? the authors recommend version 8 to build the planner. Reference https://github.com/galvusdamor/panda3core)

## Results :memo:

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