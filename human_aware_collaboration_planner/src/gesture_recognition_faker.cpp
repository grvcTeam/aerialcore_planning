#include "ros/ros.h"
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include "human_aware_collaboration_planner/NewTaskAction.h"
#include "human_aware_collaboration_planner/Waypoint.h"

/*
 * FUNCTION CALL PROTOTIPES:
 * rosrun human_aware_collaboration_planner gesture_recognition_faker id task_type params
 *
 * SAFETY MONITORING:
 * rosrun human_aware_collaboration_planner gesture_recognition_faker task_id m human_target_id distance number
 *
 * INSPECTION:
 * rosrun human_aware_collaboration_planner gesture_recognition_faker task_id i x y z
 * rosrun human_aware_collaboration_planner gesture_recognition_faker task_id i x y z x y z
 * rosrun human_aware_collaboration_planner gesture_recognition_faker task_id i x y z x y z x y z
 * rosrun human_aware_collaboration_planner gesture_recognition_faker task_id i x y z x y z x y z x y z...
 *
 * TOOL DELIVERY:
 * rosrun human_aware_collaboration_planner gesture_recognition_faker task_id d tool_id human_target_id
 *
 */


int main(int argc, char **argv)
{
  //Init ROS node
  std::string task_id = argv[1];
  ros::init(argc, argv, "gesture_recognition_faker_" + task_id);

  //Action client to send new task
  actionlib::SimpleActionClient<human_aware_collaboration_planner::NewTaskAction> nt_ac("incoming_task_action", true);
  nt_ac.waitForServer(ros::Duration(10.0));

  human_aware_collaboration_planner::NewTaskGoal goal;
  bool error = 1;
  int i;

  if (argc > 3)
  {
    goal.task.id = argv[1];
    goal.task.type = *(argv[2]);
    
    switch(goal.task.type)
    {
      case 'M':
      case 'm':
        if (argc == 6)
        {
          goal.task.monitor.human_target_id = argv[3];
          goal.task.monitor.distance = atof(argv[4]);
          goal.task.monitor.number = atoi(argv[5]);
          error = 0;
        }
        break;
      case 'F':
      case 'f':
        if (argc == 5)
        {
          goal.task.monitor_ugv.ugv_id = argv[3];
          goal.task.monitor_ugv.height = atof(argv[4]);
          error = 0;
        }
        break;
      case 'I':
      case 'i':
        i = 3;
        while(i + 2 < argc)
        {
          human_aware_collaboration_planner::Waypoint waypoint;
          waypoint.x = atof(argv[i]);
          waypoint.y = atof(argv[i+1]);
          waypoint.z = atof(argv[i+2]);

          goal.task.inspect.waypoints.push_back(waypoint);
          i += 3;
        }
        if(goal.task.inspect.waypoints.size())
          error = 0;
        break;
      case 'A':
      case 'a':
        i = 3;
        while(i + 2 < argc)
        {
          human_aware_collaboration_planner::Waypoint waypoint;
          waypoint.x = atof(argv[i]);
          waypoint.y = atof(argv[i+1]);
          waypoint.z = atof(argv[i+2]);

          goal.task.inspect.waypoints.push_back(waypoint);
          i += 3;
        }
        if(goal.task.inspect.waypoints.size())
          error = 0;
        break;
      case 'D':
      case 'd':
        if (argc == 5)
        {
          goal.task.deliver.tool_id = argv[3];
          goal.task.deliver.human_target_id = argv[4];
          error = 0;
        }
        break;
      default:
        break;
    }
  }
  if(error)
  {
    ROS_INFO("Usage: gesture_recognition_faker task_id task_type task_params");
    ROS_INFO("task_params defined in msg/{monitor, inspect, deliver_tool}_params");
    return 1;
  }

  nt_ac.sendGoal(goal);

  nt_ac.waitForResult(ros::Duration(60.0));
  if (nt_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("New Task send successfully");
  else
    ROS_INFO("New task sending timeout");
  ROS_INFO("Current State: %s", nt_ac.getState().toString().c_str());

  return 0;
}
