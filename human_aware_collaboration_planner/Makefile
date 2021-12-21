task_id = i
human_target = human_target_1
distance = 1.5
number = 2
waypoint = "0 10 2 0 15 2"
tool = hammer

agent_id = 1

.PHONY: launch

main:
	@catkin build --this

clean:
	@catkin clean --this
	
gesture_info:
	@sed -n 8,21p src/gesture_recognition_faker.cpp

launch:
	@roslaunch human_aware_collaboration_planner simulation.launch

monitor:
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_$(task_id) M $(human_target) $(distance) $(number)

inspect:
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_$(task_id) I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3

deliver:
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_$(task_id) D $(tool) $(human_target)

battery:
	@rostopic echo /uav_$(agent_id)/mavros/battery_fake

battery_off:
	@rostopic pub /uav_$(agent_id)/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "0.2" "0.01" "0.01"

battery_ok:
	@rostopic pub /uav_$(agent_id)/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "1" "0.01" "0.01"

battery_static:
	@rostopic pub /uav_$(agent_id)/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "1" "0.01" "0.01"

pose:
	@rostopic echo /uav_$(agent_id)/ual/pose

state:
	@rostopic echo /uav_$(agent_id)/ual/state

mission_over:
	@rostopic pub /mission_over human_aware_collaboration_planner/MissionOver "value: true"

groot:
	@rosrun groot Groot &

#Phase 1 are the tests executted to validate the Agent Behavior Manager in simulations with a single UAV
phase_1_tasks_1:
	@echo "Mission Start. 3 Tasks requested, 1 of each type"
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_1 D hammer human_target_1
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_2 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_3 M human_target_1 1.5 4

phase_1_tasks_2:
	@echo "Unforeseen event: New Task"
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_4 D hammer human_target_1

phase_1_tasks_3:
	@echo "Unforeseen event: New Task with a duplicated ID. Tool Delivery Task is overwrited."
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_4 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3

phase_1_unforeseen_event_1:
	@echo "Unforeseen event: uav_$(agent_id) has low battery"
	@rostopic pub /uav_$(agent_id)/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "0.2" "0.01" "0.01"

phase_1_unforeseen_event_2:
	@echo "Unforeseen event: uav_$(agent_id) has its battery charged"
	@rostopic pub /uav_$(agent_id)/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "1" "0.01" "0.01"

phase_1_unforeseen_event_3:
	@echo "Unforeseen event: High-Level Planner block disconnection"
	@rosnode kill /high_level_planner

phase_1_unforeseen_event_4:
	@echo "Unforseen event: mission over"
	@rostopic pub /mission_over human_aware_collaboration_planner/MissionOver "value: true"

#Phase 2 are the tests executted to validate the High-Level Planner in simulations with multiple UAVs
phase_2_tasks_1:
	@echo "Mission Start. 3 Tasks requested, 1 of each type"
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_1 D hammer human_target_2
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_2 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_3 M human_target_1 1.5 1

phase_2_tasks_2:
	@echo "Unforeseen event: New Tasks"
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_4 D hammer human_target_2
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_3 I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3

phase_2_tasks_3:
	@echo "Unforeseen event: task params update"
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_3 M human_target_1 3 4

phase_2_unforeseen_event_1:
	@echo "Unforeseen event: UAV_2 has low battery"
	@rostopic pub /uav_2/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "0.2" "0.01" "0.01"

phase_2_unforeseen_event_2:
	@echo "Unforeseen event: UAV_3 has low battery"
	@rostopic pub /uav_3/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "0.2" "0.01" "0.01"

phase_2_unforeseen_event_3:
	@echo "Unforeseen event: UAV_2 has its battery charged"
	@rostopic pub /uav_2/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "1" "0.01" "0.01"

phase_2_unforeseen_event_4:
	@echo "Unforeseen event: UAV_3 has its battery charged"
	@rostopic pub /uav_3/mavros/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "1" "0.01" "0.01"

phase_2_unforeseen_event_5:
	@echo "Unforeseen event: UAV_2 disconnection"
	@rosnode kill /uav_2/agent_behaviour_manager

phase_2_unforeseen_event_6:
	@echo "Unforeseen event: UAV_4 disconnection"
	@rosnode kill /uav_4/agent_behaviour_manager

phase_2_unforeseen_event_7:
	@echo "Unforeseen event: UAV_2 reconnection"
	@rosrun human_aware_collaboration_planner agent_behaviour_manager __ns:=uav_2

phase_2_unforeseen_event_8:
	@echo "Unforeseen event: UAV_4 reconnection"
	@rosrun human_aware_collaboration_planner agent_behaviour_manager __ns:=uav_4

phase_2_unforeseen_event_9:
	@echo "Unforseen event: mission over"
	@rostopic pub /mission_over human_aware_collaboration_planner/MissionOver "value: true"
