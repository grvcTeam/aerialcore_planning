task_id = i
human_target = human_target_1
distance = 1.5
number = 2
waypoint = "0 10 2 0 15 2"
tool = hammer

agent_prefix = uav
agent_id = 1
x = 3.5
y = 9
z = 0.3
h = 0

.PHONY: launch

main:
	@cd ~/grvc_planning_workspace; catkin build human_aware_collaboration_planner

clean:
	@cd ~/grvc_planning_workspace; catkin clean human_aware_collaboration_planner
	
gesture_info:
	@sed -n 8,21p src/gesture_recognition_faker.cpp

launch:
	@roslaunch human_aware_collaboration_planner simulation.launch

start:
	@./tmux_scripts/no_mapping_no_collision_avoidance/start.sh

monitor:
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_$(task_id) M $(human_target) $(distance) $(number)

inspect:
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_$(task_id) I 0 7 3 7 7 3 7 14 3 7 21 3 0 21 3 -7 21 3 -7 14 3 -7 7 3

deliver:
	@rosrun human_aware_collaboration_planner gesture_recognition_faker task_$(task_id) D $(tool) $(human_target)

battery:
	@rostopic echo /$(agent_prefix)$(agent_id)/battery_fake

battery_off:
	@rostopic pub /$(agent_prefx)$(agent_id)/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "0.2" "0.01" "0.01"

battery_ok:
	@rostopic pub /$(agent_prefx)$(agent_id)/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "1" "0.01" "0.01"

battery_static:
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "1" "0.01" "0.01"

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
	@echo "Unforeseen event: $(agent_prefix)$(agent_id) has low battery"
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "0.2" "0.01" "0.01"

phase_1_unforeseen_event_2:
	@echo "Unforeseen event: $(agent_prefix)$(agent_id) has its battery charged"
	@rostopic pub /$(agent_prefix)$(agent_id)/battery_fake/control human_aware_collaboration_planner/BatteryControl "2" "1" "0.01" "0.01"

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
	@echo "Unforeseen event: UAV2 has low battery"
	@rostopic pub /uav2/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "0.2" "0.01" "0.01"

phase_2_unforeseen_event_2:
	@echo "Unforeseen event: UAV3 has low battery"
	@rostopic pub /uav3/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "0.2" "0.01" "0.01"

phase_2_unforeseen_event_3:
	@echo "Unforeseen event: UAV2 has its battery charged"
	@rostopic pub /uav2/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "1" "0.01" "0.01"

phase_2_unforeseen_event_4:
	@echo "Unforeseen event: UAV3 has its battery charged"
	@rostopic pub /uav3/battery_fake/control human_aware_collaboration_planner/BatteryControl "0" "1" "0.01" "0.01"

phase_2_unforeseen_event_5:
	@echo "Unforeseen event: UAV2 disconnection"
	@rosnode kill /uav2/agent_behaviour_manager

phase_2_unforeseen_event_6:
	@echo "Unforeseen event: UAV4 disconnection"
	@rosnode kill /uav4/agent_behaviour_manager

phase_2_unforeseen_event_7:
	@echo "Unforeseen event: UAV2 reconnection"
	@rosrun human_aware_collaboration_planner agent_behaviour_manager __ns:=uav2

phase_2_unforeseen_event_8:
	@echo "Unforeseen event: UAV4 reconnection"
	@rosrun human_aware_collaboration_planner agent_behaviour_manager __ns:=uav4

phase_2_unforeseen_event_9:
	@echo "Unforseen event: mission over"
	@rostopic pub /mission_over human_aware_collaboration_planner/MissionOver "value: true"

mrs_drone_spawn:
	@rosservice call /mrs_drone_spawner/spawn "$(agent_id) --f550 --enable-rangefinder --pos $(x) $(y) $(z) $(h)"

mrs_takeoff:
	@rostopic pub /$(agent_prefix)$(agent_id)/mrs_actionlib_interface/goal mrs_actionlib_interface/commandActionGoal "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: ''}, goal: {command: 0, goto_x: 0.0, goto_y: 0.0, goto_z: 0.0, goto_hdg: 0.0}}"

mrs_land:
	@rostopic pub /$(agent_prefix)$(agent_id)/mrs_actionlib_interface/goal mrs_actionlib_interface/commandActionGoal "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: ''}, goal: {command: 1, goto_x: 0.0, goto_y: 0.0, goto_z: 0.0, goto_hdg: 0.0}}"

mrs_goto:
	@rostopic pub /$(agent_prefix)$(agent_id)/mrs_actionlib_interface/goal mrs_actionlib_interface/commandActionGoal "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}, goal_id: {stamp: {secs: 0, nsecs: 0}, id: ''}, goal: {command: 2, goto_x: $x, goto_y: $y, goto_z: $z, goto_hdg: $h}}"
