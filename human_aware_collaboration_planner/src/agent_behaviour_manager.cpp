#include "human_aware_collaboration_planner/agent_behaviour_manager.h"

//Behavior Tree structure xml definition: past here the xml code if createTreeFromText function is used
static const char* behaviour_tree_xml = R"(
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
    </BehaviorTree>
</root>
 )";

//Behavior Tree Nodes declaration ***********************************************************************************
//******************************* Actions {
//GoNearChargingStation {
GoNearChargingStation::GoNearChargingStation(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
GoNearChargingStation::~GoNearChargingStation(){halt();}
void GoNearChargingStation::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearChargingStation::providedPorts() {return{};}
BT::NodeStatus GoNearChargingStation::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  classes::Position assigned_charging_station;

  std::string nearest_station;
  float distance = -1;
  float tmp_distance;

  /***************************************** TODO: TO BE IMPROVED ***************************************************/
  //Find closest station (sustituido por estacion fija hasta que se una con los lower-level controllers)
  /*distance = -1;
  for(auto& charging_station : agent_->known_positions_["charging_stations"])
  {
    tmp_distance = classes::distance(agent_->position_, charging_station.second);
    if(distance == -1 || tmp_distance < distance)
    {
      distance = tmp_distance;
      nearest_station = charging_station.first;
    }
  }*/

  //TODO: While programig a shared resource with the charging stations, this will be where the Agent goes to recharge
  nearest_station = "charging_station_" + agent_->id_;

  //Emergency Recharging
  if(agent_->task_queue_.empty())
    assigned_charging_station = agent_->known_positions_["charging_stations"][nearest_station];
  //Recharge Task
  else
  {
    task = agent_->task_queue_.front();
    if(task->getType() != 'R')
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      ROS_WARN("[GoNearChargingStation] First task of the queue isn't type Recharge");
      return BT::NodeStatus::FAILURE;
    }
    assigned_charging_station = task->getChargingStation();

    //Assign and reserve this charging station for this Agent
    if(assigned_charging_station.getID().empty())
    {
      task->setChargingStation(&(agent_->known_positions_["charging_stations"][nearest_station]));
      assigned_charging_station = task->getChargingStation();
    }
  }
  /*************************************************************************************************************/

  /************************** IST Collaboration: Mobile Charging Station ***************************************/
  actionlib::SimpleActionClient<ist_use_collaboration_msgs::RequestMobileChargingStationAction> 
    request_charging_ac_("/jackal0/cooperation_use/request_mobile_charging_station", true);
  ist_use_collaboration_msgs::RequestMobileChargingStationGoal goal;
  request_charging_ac_.waitForServer(ros::Duration(1.0));
  goal.requester_id = agent_->id_;
  request_charging_ac_.sendGoal(goal);

  //Wait for result. Maximum 5 seconds. If timeout, recharge on a fixed platform.
  //If result, read result. If true, change assigned_charging_station_, if false, land on a fixed platform.
  if(request_charging_ac_.waitForResult(ros::Duration(5.0)))
  {
    ist_use_collaboration_msgs::RequestMobileChargingStationResultConstPtr result = request_charging_ac_.getResult();
    if(result->success)
      assigned_charging_station = agent_->atrvjr_pose_;
  }

  /*************************************************************************************************************/

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        ROS_INFO("[GoNearChargingStation] Calling take_off");
        //UAL take_off service call
        if(!agent_->take_off(3, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearChargingStation] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        //UAL take_off service result waiting loop
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        //Go to recharging station
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        ROS_INFO("[GoNearChargingStation] Moving to recharging station (%.1f,%.1f)[%s]",
            assigned_charging_station.getX(), assigned_charging_station.getY(), agent_->pose_frame_id_.c_str());
        if(agent_->go_to_waypoint(assigned_charging_station.getX(), assigned_charging_station.getY(),
              assigned_charging_station.getZ() + 1, false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(assigned_charging_station.getX(),
                assigned_charging_station.getY(), assigned_charging_station.getZ() + 1))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          ROS_INFO("[GoNearChargingStation] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearChargingStation] Failed to call service go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }
  return BT::NodeStatus::IDLE;
}
void GoNearChargingStation::halt(){
  ROS_INFO("[GoNearChargingStation] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//Recharge {
Recharge::Recharge(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
Recharge::~Recharge(){halt();}
void Recharge::init(AgentNode* agent){agent_ = agent;}
BT::PortsList Recharge::providedPorts() {return{};}
BT::NodeStatus Recharge::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  bool recharge_task = false;

  classes::Task* task;
  float final_percentage;

  actionlib::SimpleActionClient<human_aware_collaboration_planner::TaskResultAction> task_result_ac_("/" +
      agent_->beacon_.id + "/task_result", true);
  human_aware_collaboration_planner::TaskResultGoal goal;
  
  //Emergency Recharging
  if(agent_->task_queue_.empty())
    final_percentage = 0.99;
  //Recharge Task
  else
  {
    task = agent_->task_queue_.front();
    if(task->getType() != 'R')
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      ROS_WARN("[Recharge] First task of the queue isn't type Recharge");
      return BT::NodeStatus::FAILURE;
    }
    final_percentage = task->getFinalPercentage();
    recharge_task = true;
    goal.task.id = task->getID();
    goal.task.type = task->getType();
  }

  //TODO: Calling Recharge lower level controllers (faked) 
  ROS_INFO("[Recharge] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 1: //LANDED_DISARMED
      case 2: //LANDED_ARMED
        if(isHaltRequested())
        {
          if(recharge_task)
          {
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.result = 0; //TODO: Change with the result of Lower-level controllers
            task_result_ac_.sendGoal(goal);
          }
          return BT::NodeStatus::IDLE;
        }
        ROS_INFO("[Recharge] Recharging...");
        while(!isHaltRequested())
        {
          if(agent_->battery_ < final_percentage)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          else
          {
            if(recharge_task)
            {
              task_result_ac_.waitForServer(ros::Duration(1.0));
              goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
              task_result_ac_.sendGoal(goal);
            }
            return BT::NodeStatus::SUCCESS;
          }

        }
        if(recharge_task)
        {
          task_result_ac_.waitForServer(ros::Duration(1.0));
          goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
          task_result_ac_.sendGoal(goal);
        }
        return BT::NodeStatus::IDLE;
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
        {
          if(recharge_task)
          {
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.result = 0; //TODO: Change with the result of Lower-level controllers
            task_result_ac_.sendGoal(goal);
          }
          return BT::NodeStatus::IDLE;
        }
        if(!agent_->land(false))
        {
          if(isHaltRequested())
          {
            if(recharge_task)
            {
              task_result_ac_.waitForServer(ros::Duration(1.0));
              goal.result = 0; //TODO: Change with the result of Lower-level controllers
              task_result_ac_.sendGoal(goal);
            }
            return BT::NodeStatus::IDLE;
          }
          ROS_ERROR("[Recharge] Failed to call service land");
          if(recharge_task)
          {
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
            task_result_ac_.sendGoal(goal);
          }
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 1 && agent_->state_ != 2)
          {
            if(isHaltRequested())
            {
              if(recharge_task)
              {
                task_result_ac_.waitForServer(ros::Duration(1.0));
                goal.result = 0; //TODO: Change with the result of Lower-level controllers
                task_result_ac_.sendGoal(goal);
              }
              return BT::NodeStatus::IDLE;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 0: //UNINITIALIZED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }

  if(recharge_task)
  {
    task_result_ac_.waitForServer(ros::Duration(1.0));
    goal.result = 0; //TODO: Change with the result of Lower-level controllers
    task_result_ac_.sendGoal(goal);
  }
  return BT::NodeStatus::IDLE;
  //*****************************************************************************************************************
}
void Recharge::halt(){
  ROS_INFO("[Recharge] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//BackToStation {
BackToStation::BackToStation(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name,
    config) {}
BackToStation::~BackToStation(){halt();}
void BackToStation::init(AgentNode* agent){agent_ = agent;}
BT::PortsList BackToStation::providedPorts() {return{};}
BT::NodeStatus BackToStation::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");
    
  ROS_INFO("[BackToStation] Emptying the Queue...");
  agent_->emptyTheQueue();

  int flag = 0;
  std::string nearest_station;
  float distance = -1;
  float tmp_distance;

  //TODO: BackToStation. search instead for the closest free station
  nearest_station = "station_" + agent_->id_;

  while(!isHaltRequested())
  {
    //If Agent is already in station. Land if needed and back_to_station
    for(auto& station : agent_->known_positions_["stations"])
    {
      if(classes::distance2D(agent_->position_, station.second) < 0.5)
        flag = 1;
    }
    if(flag)
    {
      switch(agent_->state_)
      {
        case 1: //LANDED_DISARMED
        case 2: //LANDED_ARMED
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          return BT::NodeStatus::SUCCESS;
          break;
        case 4: //FLYING_AUTO
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(agent_->land(false))
          {
            while(agent_->state_ != 1 && agent_->state_ != 2)
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            ROS_INFO("[BackToStation] LANDED");
            return BT::NodeStatus::SUCCESS;
          }
          else
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            ROS_ERROR("[BackToStation] Failed to call service land");
            return BT::NodeStatus::FAILURE;
          }
          break;
        case 0: //UNINITIALIZED
        case 3: //TAKING_OFF
        case 5: //FLIYING_MANUAL
        case 6: //LANDING
        default:
          break;
      }
    }
    //If Agent is not in recharging station, call go_to_waypoint
    else
    {
      switch(agent_->state_)
      {
        case 2: //LANDED_ARMED
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(!agent_->take_off(3, false))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            ROS_ERROR("[BackToStation] Failed to call service take_off");
            return BT::NodeStatus::FAILURE;
          }
          else
          {
            while(agent_->state_ != 4)
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
          break;
        case 4: //FLYING_AUTO
          //Find closest station (temporaly comment out, need to implement a shared respurce to see the free stations)
          /*
             distance = -1;
             for(auto& station : agent_->known_positions_["stations"])
             {
             tmp_distance = classes::distance(agent_->position_, station.second);
            if(distance == -1 || tmp_distance < distance)
            {
              distance = tmp_distance;
              nearest_station = station.first;
            }
          }*/
          //Go to recharging station
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          //ROS_INFO("[BackToStation] Moving to station (%f,%f,%f)[%s]", 
              //agent_->known_positions_["stations"][nearest_station].getX(),
              //agent_->known_positions_["stations"][nearest_station].getY(),
              //agent_->known_positions_["stations"][nearest_station].getZ(),
              //agent_->pose_frame_id_.c_str());
          //UAL goto service call
          if(!agent_->go_to_waypoint(agent_->known_positions_["stations"][nearest_station].getX(),
                agent_->known_positions_["stations"][nearest_station].getY(),
                agent_->known_positions_["stations"][nearest_station].getZ() + 1, false))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            ROS_ERROR("[BackToStation] Failed to call service go_to_waypoint");
            return BT::NodeStatus::FAILURE;
          }
          //UAL goto service result waiting loop
          else
          {
            while(!agent_->checkIfGoToServiceSucceeded(agent_->known_positions_["stations"][nearest_station].getX(),
                  agent_->known_positions_["stations"][nearest_station].getY(),
                  agent_->known_positions_["stations"][nearest_station].getZ() + 1))
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
          //Land in station
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          if(!agent_->land(false))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            ROS_ERROR("[BackToStation] Failed to call service land");
            return BT::NodeStatus::FAILURE;
          }
          else
          {
            while(agent_->state_ != 1 && agent_->state_ != 2)
            {
              if(isHaltRequested())
                return BT::NodeStatus::IDLE;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return BT::NodeStatus::SUCCESS;
          }
          break;
        case 0: //UNINITIALIZED
        case 1: //LANDED_DISARMED
        case 3: //TAKING_OFF
        case 5: //FLIYING_MANUAL
        case 6: //LANDING
        default:
          break;
      }
    }
  }
  return BT::NodeStatus::IDLE;
}
void BackToStation::halt(){
  ROS_INFO("[BackToStation] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//GoNearHumanTarget {
GoNearHumanTarget::GoNearHumanTarget(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
GoNearHumanTarget::~GoNearHumanTarget(){halt();}
void GoNearHumanTarget::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearHumanTarget::providedPorts() {return{};}
BT::NodeStatus GoNearHumanTarget::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[GoNearHumanTarget] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  float distance;
  switch(task->getType())
  {
    case 'D':
    case 'd':
      distance = 1.5;
      break;
    case 'M':
    case 'm':
      distance = task->getDistance();
      break;
    default:
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      ROS_WARN("[GoNearHumanTarget] First task of the queue isn't type Monitor");
      return BT::NodeStatus::FAILURE;
      break;
  }

  classes::Position human_position = task->getHumanPosition();
  classes::Position near_human_pose = classes::close_pose_2D(agent_->position_, human_position, distance);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(3, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearHumanTarget] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        ROS_INFO("[GoNearHumanTarget] Moving near HT...");
        if(agent_->go_to_waypoint(near_human_pose.getX(), near_human_pose.getY(), near_human_pose.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_human_pose.getX(), near_human_pose.getY(),
                near_human_pose.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          ROS_INFO("[GoNearHumanTarget] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        ROS_ERROR("[GoNearHumanTarget] Failed to call service go_to_waypoint");
        return BT::NodeStatus::FAILURE;
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
    //MAYBE PUT SOME SLEEP TIME HERE ********************************************************************************
  }
  return BT::NodeStatus::IDLE;
}
void GoNearHumanTarget::halt(){
  ROS_INFO("[GoNearHumanTarget] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//MonitorHumanTarget {
MonitorHumanTarget::MonitorHumanTarget(const std::string& name, const BT::NodeConfiguration& config) :
  BT::AsyncActionNode(name, config) {}
MonitorHumanTarget::~MonitorHumanTarget(){halt();}
void MonitorHumanTarget::init(AgentNode* agent){agent_ = agent;}
BT::PortsList MonitorHumanTarget::providedPorts() {return{};}
BT::NodeStatus MonitorHumanTarget::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[MonitorHumanTarget] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'M')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[MonitorHumanTarget] First task of the queue isn't type Monitor");
    return BT::NodeStatus::FAILURE;
  }

  actionlib::SimpleActionClient<human_aware_collaboration_planner::TaskResultAction> 
    task_result_ac_("/" + agent_->beacon_.id + "/task_result", true);
  human_aware_collaboration_planner::TaskResultGoal goal;

  //TODO: Calling Safety Monitoring lower level controllers (faked) 
  ROS_INFO("[MonitorHumanTarget] Calling Lower-level controllers..."); 
  //********************************************* FAKED *************************************************************
  while(!isHaltRequested())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  //TODO: As this task ends when an operator requested, its result should be either true or false according to this
  task_result_ac_.waitForServer(ros::Duration(1.0));
  goal.task.id = task_id;
  goal.task.type = 'M';
  goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
  task_result_ac_.sendGoal(goal);
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'M');
  ROS_INFO("[MonitorHumanTarget] MONITOR TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();
  
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void MonitorHumanTarget::halt(){
  ROS_INFO("[MonitorHumanTarget] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//GoNearWP {
GoNearWP::GoNearWP(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
GoNearWP::~GoNearWP(){halt();}
void GoNearWP::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearWP::providedPorts() {return{};}
BT::NodeStatus GoNearWP::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[GoNearWP] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  human_aware_collaboration_planner::Waypoint nearest_wp;

  if(task->getType() != 'I' && task->getType() != 'A')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[GoNearWP] First task of the queue isn't type Inspect or InspectPVArray");
    return BT::NodeStatus::FAILURE;
  }

  if(task->getType() == 'I')
  {
    //Find the closest WP from the list
    float distance = -1;
    float tmp_distance;
    for(auto& waypoint : task->getInspectWaypoints())
    {
      tmp_distance = classes::distance(agent_->position_, waypoint);
      if(distance == -1 || tmp_distance < distance)
      {
        distance = tmp_distance;
        nearest_wp = waypoint;
      }
    }
  }

  if(task->getType() == 'A')
  {
    auto waypoint = task->getInspectWaypoints();
    nearest_wp = waypoint[0];
  }

  classes::Position near_waypoint = classes::close_pose_2D(agent_->position_, nearest_wp, 1.5);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(3, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearWP] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        ROS_INFO("[GoNearWP] Moving near WP...");
        if(agent_->go_to_waypoint(near_waypoint.getX(), near_waypoint.getY(), near_waypoint.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_waypoint.getX(), near_waypoint.getY(),
                near_waypoint.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          ROS_INFO("[GoNearWP] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearWP] Failed to call service go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
    //MAYBE PUT SOME SLEEP TIME HERE ********************************************************************************
  }
  return BT::NodeStatus::IDLE;
}
void GoNearWP::halt(){
  ROS_INFO("[GoNearWP] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//TakeImage {
TakeImage::TakeImage(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
TakeImage::~TakeImage(){halt();}
void TakeImage::init(AgentNode* agent){agent_ = agent;}
BT::PortsList TakeImage::providedPorts() {return{};}
BT::NodeStatus TakeImage::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[TakeImage] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'I')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[TakeImage] First task of the queue isn't type Inspect");
    return BT::NodeStatus::FAILURE;
  }

  actionlib::SimpleActionClient<human_aware_collaboration_planner::TaskResultAction> 
    task_result_ac_("/" + agent_->beacon_.id + "/task_result", true);
  human_aware_collaboration_planner::TaskResultGoal goal;

  //TODO: Calling Inspection lower level controllers (faked) 
  ROS_INFO("[TakeImage] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  task_result_ac_.waitForServer(ros::Duration(1.0));
  goal.task.id = task_id;
  goal.task.type = 'I';
  goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
  task_result_ac_.sendGoal(goal);
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'I');
  ROS_INFO("[TakeImage] INSPECT TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();

  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void TakeImage::halt(){
  ROS_INFO("[TakeImage] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//InspectPVArray {
InspectPVArray::InspectPVArray(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
InspectPVArray::~InspectPVArray(){halt();}
void InspectPVArray::init(AgentNode* agent){agent_ = agent;}
BT::PortsList InspectPVArray::providedPorts() {return{};}
BT::NodeStatus InspectPVArray::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[InspectPVArray] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'A')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[InspectPVArray] First task of the queue isn't type Inspect PV Array");
    return BT::NodeStatus::FAILURE;
  }

  actionlib::SimpleActionClient<human_aware_collaboration_planner::TaskResultAction> 
    task_result_ac_("/" + agent_->beacon_.id + "/task_result", true);
  human_aware_collaboration_planner::TaskResultGoal goal;

  //TODO: Calling Inspection lower level controllers (faked) 
  ROS_INFO("[InspectPVArray] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  auto wp = task->getInspectWaypoints();

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
        {
          task_result_ac_.waitForServer(ros::Duration(1.0));
          goal.task.id = task_id;
          goal.task.type = 'A';
          goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
          task_result_ac_.sendGoal(goal);
          if(goal.result)
            agent_->removeTaskFromQueue(task_id, 'A');
          ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
          agent_->infoQueue();
          return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
        }
        if(!agent_->take_off(3, false))
        {
          if(isHaltRequested())
          {
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.task.id = task_id;
            goal.task.type = 'A';
            goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
            task_result_ac_.sendGoal(goal);
            if(goal.result)
              agent_->removeTaskFromQueue(task_id, 'A');
            ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
            agent_->infoQueue();
            return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
          }
          ROS_ERROR("[InspectPVArray] Failed to call service take_off");
          task_result_ac_.waitForServer(ros::Duration(1.0));
          goal.task.id = task_id;
          goal.task.type = 'A';
          goal.result = 0; //TODO: Change with the result of Lower-level controllers
          task_result_ac_.sendGoal(goal);
          if(goal.result)
            agent_->removeTaskFromQueue(task_id, 'A');
          ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
          agent_->infoQueue();
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
            {
              task_result_ac_.waitForServer(ros::Duration(1.0));
              goal.task.id = task_id;
              goal.task.type = 'A';
              goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
              task_result_ac_.sendGoal(goal);
              if(goal.result)
                agent_->removeTaskFromQueue(task_id, 'A');
              ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
              agent_->infoQueue();
              return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
        {
          task_result_ac_.waitForServer(ros::Duration(1.0));
          goal.task.id = task_id;
          goal.task.type = 'A';
          goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
          task_result_ac_.sendGoal(goal);
          if(goal.result)
            agent_->removeTaskFromQueue(task_id, 'A');
          ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
          agent_->infoQueue();
          return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
        }
        ROS_INFO("[InspectPVArray] Moving to the beggining...");
        if(agent_->go_to_waypoint(wp[0].x, wp[0].y, wp[0].z, false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(wp[0].x, wp[0].y, wp[0].z))
          {
            if(isHaltRequested())
            {
              task_result_ac_.waitForServer(ros::Duration(1.0));
              goal.task.id = task_id;
              goal.task.type = 'A';
              goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
              task_result_ac_.sendGoal(goal);
              if(goal.result)
                agent_->removeTaskFromQueue(task_id, 'A');
              ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
              agent_->infoQueue();
              return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          ROS_INFO("[InspectPVArray] Moving to the end...");
          if(agent_->go_to_waypoint(wp[1].x, wp[1].y, wp[1].z, false))
          {
            while(!agent_->checkIfGoToServiceSucceeded(wp[1].x, wp[1].y, wp[1].z))
            {
              if(isHaltRequested())
              {
                task_result_ac_.waitForServer(ros::Duration(1.0));
                goal.task.id = task_id;
                goal.task.type = 'A';
                goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
                task_result_ac_.sendGoal(goal);
                if(goal.result)
                  agent_->removeTaskFromQueue(task_id, 'A');
                ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
                agent_->infoQueue();
                return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            ROS_INFO("[InspectPVArray] Returning SUCCESS...");
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.task.id = task_id;
            goal.task.type = 'A';
            goal.result = 1; //TODO: Change with the result of Lower-level controllers
            //Solar Panel 3_4
            geometry_msgs::Point target_xyz;
            target_xyz.x = -36.6343;
            target_xyz.y =  62.293;
            geographic_msgs::GeoPose target_gps;
            target_gps.position.latitude  = -7.96213167462045;
            target_gps.position.longitude = 38.54156780106911;
            goal.do_closer_inspection.xyz_coordinates.push_back(target_xyz);
            goal.do_closer_inspection.gps_coordinates.push_back(target_gps);
            task_result_ac_.sendGoal(goal);
            if(goal.result)
              agent_->removeTaskFromQueue(task_id, 'A');
            ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
            agent_->infoQueue();
            return BT::NodeStatus::SUCCESS;
          }
          else
          {
            if(isHaltRequested())
            {
              task_result_ac_.waitForServer(ros::Duration(1.0));
              goal.task.id = task_id;
              goal.task.type = 'A';
              goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
              task_result_ac_.sendGoal(goal);
              if(goal.result)
                agent_->removeTaskFromQueue(task_id, 'A');
              ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
              agent_->infoQueue();
              return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
            }
            ROS_ERROR("[InspectPVArray] Failed to call service go_to_waypoint");
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.task.id = task_id;
            goal.task.type = 'A';
            goal.result = 0; //TODO: Change with the result of Lower-level controllers
            task_result_ac_.sendGoal(goal);
            if(goal.result)
              agent_->removeTaskFromQueue(task_id, 'A');
            ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
            agent_->infoQueue();
            return BT::NodeStatus::FAILURE;
          }
          ROS_INFO("[InspectPVArray] Returning SUCCESS...");
          task_result_ac_.waitForServer(ros::Duration(1.0));
          goal.task.id = task_id;
          goal.task.type = 'A';
          goal.result = 1; //TODO: Change with the result of Lower-level controllers
          //Solar Panel 3_4
          geometry_msgs::Point target_xyz;
          target_xyz.x = -36.6343;
          target_xyz.y =  62.293;
          geographic_msgs::GeoPose target_gps;
          target_gps.position.latitude  = -7.96213167462045;
          target_gps.position.longitude = 38.54156780106911;
          goal.do_closer_inspection.xyz_coordinates.push_back(target_xyz);
          goal.do_closer_inspection.gps_coordinates.push_back(target_gps);
          task_result_ac_.sendGoal(goal);
          if(goal.result)
            agent_->removeTaskFromQueue(task_id, 'A');
          ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
          agent_->infoQueue();
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
          {
            task_result_ac_.waitForServer(ros::Duration(1.0));
            goal.task.id = task_id;
            goal.task.type = 'A';
            goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
            task_result_ac_.sendGoal(goal);
            if(goal.result)
              agent_->removeTaskFromQueue(task_id, 'A');
            ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
            agent_->infoQueue();
            return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
          }
          ROS_ERROR("[InspectPVArray] Failed to call service go_to_waypoint");
          task_result_ac_.waitForServer(ros::Duration(1.0));
          goal.task.id = task_id;
          goal.task.type = 'A';
          goal.result = 0; //TODO: Change with the result of Lower-level controllers
          task_result_ac_.sendGoal(goal);
          if(goal.result)
            agent_->removeTaskFromQueue(task_id, 'A');
          ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
          agent_->infoQueue();
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }

  task_result_ac_.waitForServer(ros::Duration(1.0));
  goal.task.id = task_id;
  goal.task.type = 'A';
  goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
  task_result_ac_.sendGoal(goal);
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'A');
  ROS_INFO("[InspectPVArray] INSPECT PV ARRAY TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();

  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void InspectPVArray::halt(){
  ROS_INFO("[InspectPVArray] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//GoNearStation {
GoNearStation::GoNearStation(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name,
    config) {}
GoNearStation::~GoNearStation(){halt();}
void GoNearStation::init(AgentNode* agent){agent_ = agent;}
BT::PortsList GoNearStation::providedPorts() {return{};}
BT::NodeStatus GoNearStation::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  classes::Position tool_position;
  classes::Position near_tool_pose;

  if(agent_->tool_flag_ != "none")
    tool_position = agent_->tools_[agent_->tool_flag_].getPosition();
  else
  {
    if(agent_->task_queue_.empty())
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      ROS_WARN("[GoNearStation] Task queue is empty");
      return BT::NodeStatus::FAILURE;
    }
    task = agent_->task_queue_.front();

    if(task->getType() != 'D')
    {
      if(isHaltRequested())
        return BT::NodeStatus::IDLE;
      ROS_WARN("[GoNearStation] First task of the queue isn't type Deliver");
      return BT::NodeStatus::FAILURE;
    }

    tool_position = task->getToolPosition();
  }
  near_tool_pose = classes::close_pose_2D(agent_->position_, tool_position, 1.5);

  while(!isHaltRequested())
  {
    switch(agent_->state_)
    {
      case 2: //LANDED_ARMED
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        if(!agent_->take_off(3, false))
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearStation] Failed to call service take_off");
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          while(agent_->state_ != 4)
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        break;
      case 4: //FLYING_AUTO
        if(isHaltRequested())
          return BT::NodeStatus::IDLE;
        ROS_INFO("[GoNearStation] Moving near Tool...");
        if(agent_->go_to_waypoint(near_tool_pose.getX(), near_tool_pose.getY(), near_tool_pose.getZ(), false))
        {
          while(!agent_->checkIfGoToServiceSucceeded(near_tool_pose.getX(), near_tool_pose.getY(),
                near_tool_pose.getZ()))
          {
            if(isHaltRequested())
              return BT::NodeStatus::IDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          ROS_INFO("[GoNearStation] Returning SUCCESS...");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          if(isHaltRequested())
            return BT::NodeStatus::IDLE;
          ROS_ERROR("[GoNearStation] Failed to call service go_to_waypoint");
          return BT::NodeStatus::FAILURE;
        }
        break;
      case 0: //UNINITIALIZED
      case 1: //LANDED_DISARMED
      case 3: //TAKING_OFF
      case 5: //FLIYING_MANUAL
      case 6: //LANDING
      default:
        break;
    }
  }
  return BT::NodeStatus::IDLE;
}
void GoNearStation::halt(){
  ROS_INFO("[GoNearStation] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//PickTool {
PickTool::PickTool(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
PickTool::~PickTool(){halt();}
void PickTool::init(AgentNode* agent){agent_ = agent;}
BT::PortsList PickTool::providedPorts() {return{};}
BT::NodeStatus PickTool::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  std::string tool_id;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[PickTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  if(task->getType() != 'D')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[PickTool] First task of the queue isn't type Deliver");
    return BT::NodeStatus::FAILURE;
  }
  tool_id = task->getToolID();

  //TODO: Calling Picking Tool lower level controllers (faked) 
  ROS_INFO("[PickTool] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  agent_->tool_flag_ = isHaltRequested() ? "none" : tool_id;
  ROS_INFO("[PickTool] PICK TOOL FINISHED");
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void PickTool::halt(){
  ROS_INFO("[PickTool] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//DropTool {
DropTool::DropTool(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config) {}
DropTool::~DropTool(){halt();}
void DropTool::init(AgentNode* agent){agent_ = agent;}
BT::PortsList DropTool::providedPorts() {return{};}
BT::NodeStatus DropTool::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");
 
  //TODO: Calling Dropping Tool lower level controllers (faked) 
  ROS_INFO("[DropTool] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  agent_->tool_flag_ = "none";
  ROS_INFO("[DropTool] DROP TOOL FINISHED");
  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void DropTool::halt(){
  ROS_INFO("[DropTool] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }

//DeliverTool {
DeliverTool::DeliverTool(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name,
    config) {}
DeliverTool::~DeliverTool(){halt();}
void DeliverTool::init(AgentNode* agent){agent_ = agent;}
BT::PortsList DeliverTool::providedPorts() {return{};}
BT::NodeStatus DeliverTool::tick(){
  if(!agent_->stop(false))
    ROS_ERROR("Failed to call stop");

  classes::Task* task;
  std::string tool_id;
  if(agent_->task_queue_.empty())
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[DeliverTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  std::string task_id = task->getID();

  if(task->getType() != 'D')
  {
    if(isHaltRequested())
      return BT::NodeStatus::IDLE;
    ROS_WARN("[DeliverTool] First task of the queue isn't type Deliver");
    return BT::NodeStatus::FAILURE;
  }
  tool_id = task->getToolID();

  actionlib::SimpleActionClient<human_aware_collaboration_planner::TaskResultAction> 
    task_result_ac_("/" + agent_->beacon_.id + "/task_result", true);
  human_aware_collaboration_planner::TaskResultGoal goal;

  //TODO: Calling Tool Delivery lower level controllers (faked) 
  ROS_INFO("[DeliverTool] Calling Lower-level controllers...");
  //********************************************* FAKED *************************************************************
  for(int i = 0; i <= 1000; i++)
  {
    if(isHaltRequested())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  agent_->tool_flag_ = isHaltRequested() ? tool_id : "none";

  task_result_ac_.waitForServer(ros::Duration(1.0));
  goal.task.id = task_id;
  goal.task.type = 'D';
  goal.result = isHaltRequested() ? 0 : 1; //TODO: Change with the result of Lower-level controllers
  task_result_ac_.sendGoal(goal);
  if(goal.result)
      agent_->removeTaskFromQueue(task_id, 'D');
  ROS_INFO("[DeliverTool] DELIVER TOOL TASK FINISHED (%s)", goal.result ? "SUCCESS" : "FAILURE");
  agent_->infoQueue();

  return isHaltRequested() ? BT::NodeStatus::IDLE : BT::NodeStatus::SUCCESS;
  //*****************************************************************************************************************
}
void DeliverTool::halt(){
  ROS_INFO("[DeliverTool] halt requested");
  //Do some cleanup if necessary
  
  BT::AsyncActionNode::halt();
}
// }
// }

//******************************* Conditions {
//MissionOver {
MissionOver::MissionOver(const std::string& name) : BT::ConditionNode(name, {}) {}
void MissionOver::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus MissionOver::tick(){
  if(agent_->mission_over_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }

//Idle {
Idle::Idle(const std::string& name) : BT::ConditionNode(name, {}) {}
void Idle::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus Idle::tick(){
  classes::Task* task;

  if(agent_->task_queue_.empty())
    return BT::NodeStatus::SUCCESS;

  task = agent_->task_queue_.front();
  switch(task->getType())
  {
    case 'W':
    case 'w':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsBatteryEnough {
IsBatteryEnough::IsBatteryEnough(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsBatteryEnough::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsBatteryEnough::tick(){
  if(agent_->battery_enough_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }

//IsBatteryFull {
IsBatteryFull::IsBatteryFull(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsBatteryFull::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsBatteryFull::tick(){
  if(agent_->battery_ > 0.95)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }

//IsTaskRecharge {
IsTaskRecharge::IsTaskRecharge(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskRecharge::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskRecharge::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsTaskRecharge] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'R':
    case 'r':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskMonitor {
IsTaskMonitor::IsTaskMonitor(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskMonitor::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskMonitor::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsTaskMonitor] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'M':
    case 'm':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskInspect {
IsTaskInspect::IsTaskInspect(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskInspect::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskInspect::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsTaskInspect] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'I':
    case 'i':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskInspectPVArray {
IsTaskInspectPVArray::IsTaskInspectPVArray(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskInspectPVArray::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskInspectPVArray::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsTaskInspectPVArray] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'A':
    case 'a':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsTaskDeliverTool {
IsTaskDeliverTool::IsTaskDeliverTool(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsTaskDeliverTool::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsTaskDeliverTool::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsTaskDeliverTool] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  switch(task->getType())
  {
    case 'D':
    case 'd':
      return BT::NodeStatus::SUCCESS;
      break;
    default:
      return BT::NodeStatus::FAILURE;
      break;
  }
}
// }

//IsAgentNearChargingStation {
IsAgentNearChargingStation::IsAgentNearChargingStation(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearChargingStation::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearChargingStation::tick(){
  classes::Task* task;
  classes::Position assigned_charging_station;

  //Emergency Recharging
  if(agent_->task_queue_.empty())
  {
    for(auto& charging_station : agent_->known_positions_["charging_stations"])
      if(classes::distance2D(agent_->position_, charging_station.second) < 0.5)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

  //Recharge Task
  task = agent_->task_queue_.front();
  if(task->getType() != 'R')
  {
    ROS_WARN("[IsAgentNearChargingStation] First task of the queue isn't type Recharge");
    return BT::NodeStatus::FAILURE;
  }
  assigned_charging_station = task->getChargingStation();

  if(assigned_charging_station.getID().empty())
  {
    for(auto& charging_station : agent_->known_positions_["charging_stations"])
    {
      if(classes::distance2D(agent_->position_, charging_station.second) < 0.5)
      {
        //Assign and reserve this charging station for this Agent (to be improved)
        task->setChargingStation(&(charging_station.second));
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    if(classes::distance2D(agent_->position_, assigned_charging_station) < 0.5)
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}
// }

//IsAgentNearHumanTarget {
IsAgentNearHumanTarget::IsAgentNearHumanTarget(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearHumanTarget::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearHumanTarget::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsAgentNearHumanTarget] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  classes::Position human_position;
  switch(task->getType())
  {
    case 'D':
    case 'd':
      human_position = task->getHumanPosition();
      if(classes::distance(human_position, agent_->position_) < 2.5)
        return BT::NodeStatus::SUCCESS;
      break;
    case 'M':
    case 'm':
      human_position = task->getHumanPosition();
      if(classes::distance(human_position, agent_->position_) < task->getDistance() + 1)
        return BT::NodeStatus::SUCCESS;
      break;
    default:
      ROS_WARN("[IsAgentNearHumanTarget] First task of the queue isn't type Monitor or Deliver");
      break;
  }
  return BT::NodeStatus::FAILURE;
}
// }

//IsAgentNearWP {
IsAgentNearWP::IsAgentNearWP(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearWP::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearWP::tick(){
  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_WARN("[IsAgentNearWP] Task queue is empty");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  if(task->getType() != 'I' && task->getType() != 'A')
  {
    ROS_WARN("[IsAgentNearWP] First task of the queue isn't type Inspect or InspectPVArray");
    return BT::NodeStatus::FAILURE;
  }

  if(task->getType() == 'I')
  {
    for(auto& waypoint : task->getInspectWaypoints())
    {
      if(classes::distance(agent_->position_, waypoint) < 2)
        return BT::NodeStatus::SUCCESS;
    }
  }
  if(task->getType() == 'A')
  {
    auto waypoint = task->getInspectWaypoints();
    if(classes::distance(agent_->position_, waypoint[0]) < 2)
      return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
// }

//NeedToDropTheTool {
NeedToDropTheTool::NeedToDropTheTool(const std::string& name) : BT::ConditionNode(name, {}) {}
void NeedToDropTheTool::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus NeedToDropTheTool::tick(){
  if(agent_->tool_flag_ == "none")
    return BT::NodeStatus::FAILURE;

  classes::Task* task;
  if(agent_->task_queue_.empty())
    return BT::NodeStatus::SUCCESS;
  task = agent_->task_queue_.front();

  if(task->getType() != 'D')
    return BT::NodeStatus::SUCCESS;
  std::string tool_id = task->getToolID();

  //If Agent first task is Deliver and has a tool, check if it is the correct one
  if(agent_->tool_flag_ == tool_id)
    return BT::NodeStatus::FAILURE;
  else
    return BT::NodeStatus::SUCCESS;
}
// }

//HasAgentTheTool {
HasAgentTheTool::HasAgentTheTool(const std::string& name) : BT::ConditionNode(name, {}) {}
void HasAgentTheTool::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus HasAgentTheTool::tick(){
  if(agent_->tool_flag_ == "none")
    return BT::NodeStatus::FAILURE;

  classes::Task* task;
  if(agent_->task_queue_.empty())
  {
    ROS_INFO("[HasAgentTheTool] Task queue is empty. Need to drop the tool.");
    return BT::NodeStatus::FAILURE;
  }
  task = agent_->task_queue_.front();

  if(task->getType() != 'D')
  {
    ROS_INFO("[HasAgentTheTool] First task of the queue isn't type Deliver. Need to drop the tool.");
    return BT::NodeStatus::FAILURE;
  }
  std::string tool_id = task->getToolID();

  if(agent_->tool_flag_ == tool_id)
    return BT::NodeStatus::SUCCESS;
  else
  {
    ROS_INFO("[HasAgentTheTool] Agent has the incorrect tool for Deliver task. Need to drop the tool.");
    return BT::NodeStatus::FAILURE;
  }
}
// }

//IsAgentNearStation {
IsAgentNearStation::IsAgentNearStation(const std::string& name) : BT::ConditionNode(name, {}) {}
void IsAgentNearStation::init(AgentNode* agent){agent_ = agent;}
BT::NodeStatus IsAgentNearStation::tick(){
  classes::Task* task;
  classes::Position tool_position;

  if(agent_->tool_flag_ != "none")
    tool_position = agent_->tools_[agent_->tool_flag_].getPosition();
  else
  {
    if(agent_->task_queue_.empty())
    {
      ROS_WARN("[IsAgentNearStation] Task queue is empty");
      return BT::NodeStatus::FAILURE;
    }
    task = agent_->task_queue_.front();

    if(task->getType() != 'D')
    {
      ROS_WARN("[IsAgentNearStation] First task of the queue isn't type Deliver");
      return BT::NodeStatus::FAILURE;
    }

    tool_position = task->getToolPosition();
  }

  if(classes::distance(tool_position, agent_->position_) < 2.5)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
// }
// }

//******************************* Decorators {
//ForceRunnigNode {
ForceRunningNode::ForceRunningNode(const std::string& name) : BT::DecoratorNode(name, {} ){
  setRegistrationID("ForceRunning");
}
BT::NodeStatus ForceRunningNode::tick(){
  setStatus(BT::NodeStatus::RUNNING);

  const BT::NodeStatus child_state = child_node_->executeTick();

  switch (child_state)
  {
    case BT::NodeStatus::FAILURE:
    case BT::NodeStatus::SUCCESS:
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    default:
      break;
  }
  return status();
}
// }
// }

//Behavior Tree Nodes registration function *************************************************************************
inline void RegisterNodes(BT::BehaviorTreeFactory& factory){
  //Actions
  factory.registerNodeType<GoNearChargingStation>("GoNearChargingStation");
  factory.registerNodeType<Recharge>("Recharge");
  factory.registerNodeType<BackToStation>("BackToStation");
  factory.registerNodeType<GoNearHumanTarget>("GoNearHumanTarget");
  factory.registerNodeType<MonitorHumanTarget>("MonitorHumanTarget");
  factory.registerNodeType<GoNearWP>("GoNearWP");
  factory.registerNodeType<TakeImage>("TakeImage");
  factory.registerNodeType<InspectPVArray>("InspectPVArray");
  factory.registerNodeType<GoNearStation>("GoNearStation");
  factory.registerNodeType<PickTool>("PickTool");
  factory.registerNodeType<DropTool>("DropTool");
  factory.registerNodeType<DeliverTool>("DeliverTool");

  //Conditions
  factory.registerNodeType<MissionOver>("MissionOver");
  factory.registerNodeType<Idle>("Idle");
  factory.registerNodeType<IsBatteryEnough>("IsBatteryEnough");
  factory.registerNodeType<IsBatteryFull>("IsBatteryFull");
  factory.registerNodeType<IsTaskRecharge>("IsTaskRecharge");
  factory.registerNodeType<IsTaskMonitor>("IsTaskMonitor");
  factory.registerNodeType<IsTaskInspect>("IsTaskInspect");
  factory.registerNodeType<IsTaskInspectPVArray>("IsTaskInspectPVArray");
  factory.registerNodeType<IsTaskDeliverTool>("IsTaskDeliverTool");
  factory.registerNodeType<IsAgentNearChargingStation>("IsAgentNearChargingStation");
  factory.registerNodeType<IsAgentNearHumanTarget>("IsAgentNearHumanTarget");
  factory.registerNodeType<IsAgentNearWP>("IsAgentNearWP");
  factory.registerNodeType<NeedToDropTheTool>("NeedToDropTheTool");
  factory.registerNodeType<HasAgentTheTool>("HasAgentTheTool");
  factory.registerNodeType<IsAgentNearStation>("IsAgentNearStation");

  //Decorators
  factory.registerNodeType<ForceRunningNode>("ForceRunning");
}

//AgentNode definition **********************************************************************************************
AgentNode::AgentNode(human_aware_collaboration_planner::AgentBeacon beacon) : battery_enough_(true), loop_rate_(1),
  tool_flag_("none"), timeout_(false), beacon_(beacon), mission_over_(false), state_(0), battery_(0),
  ntl_as_(nh_, "task_list", boost::bind(&AgentNode::newTaskList, this, _1), false),
  battery_ac_("/" + beacon.id + "/battery_enough", true), 
  mrs_ac_("/" + beacon.id + "/mrs_actionlib_interface", true)
{
  //Start the server to receive tasks list through actions
  ntl_as_.start();

  ros::param::param<std::string>("~id", id_, "0");
  ros::param::param<std::string>("~ns_prefix", ns_prefix_, "uav");
  ros::param::param<std::string>("~pose_frame_id", pose_frame_id_, "map");
  ros::param::param<std::string>("~low_level_interface", low_level_interface_, "UAL");
  ros::param::param<std::string>("~pose_topic", pose_topic_, "/" + beacon_.id + "/ual/pose");
  ros::param::param<std::string>("~state_topic", state_topic_, "/" + beacon_.id + "/ual/state");
  ros::param::param<std::string>("~battery_topic", battery_topic_, "/" + beacon_.id + "/battery_fake");

  std::vector<double> origin_geo_aux(3, 0.0);
  ros::param::get("~origin_geo", origin_geo_aux);
  origin_geo_.latitude  = origin_geo_aux[0];
  origin_geo_.longitude = origin_geo_aux[1];
  origin_geo_.altitude  = origin_geo_aux[2];

  //Load of config file
  std::string path = ros::package::getPath("human_aware_collaboration_planner");
  //std::string path_evora = ros::package::getPath("ist_use_collaboration_msgs");
  ros::param::param<std::string>("~config_file", config_file_, path + "/config/conf.yaml");
  //ros::param::param<std::string>("~config_file_evora", config_file_evora_, path_evora + "/config/placemarks.yaml");
  readConfigFile(config_file_);
  //readEvoraConfigFile(config_file_evora_);

  beacon_pub_ = nh_.advertise<human_aware_collaboration_planner::AgentBeacon>("/agent_beacon", 1);
  battery_sub_ = nh_.subscribe(battery_topic_, 1, &AgentNode::batteryCallback, this);
  mission_over_sub_ = nh_.subscribe("/mission_over", 1, &AgentNode::missionOverCallback, this);
  planner_beacon_sub_ = nh_.subscribe("/planner_beacon", 1, &AgentNode::beaconCallback, this);

  if(low_level_interface_ == "UAL")
  {
    position_sub_ = nh_.subscribe(pose_topic_, 1, &AgentNode::positionCallbackUAL, this);
    state_sub_ = nh_.subscribe(state_topic_, 1, &AgentNode::stateCallbackUAL, this);
  }
  else if(low_level_interface_ == "MRS")
  {
    position_sub_ = nh_.subscribe(pose_topic_, 1, &AgentNode::positionCallbackMRS, this);
    state_sub_ = nh_.subscribe(state_topic_, 1, &AgentNode::stateCallbackMRS, this);
  }
  //UGVs position callbacks
  atrvjr_geopose_sub_ = nh_.subscribe("/atrvjr/geopose", 1, &AgentNode::atrvjrPositionCallback, this);
  jackal_geopose_sub_ = nh_.subscribe("/jackal0/geopose", 1, &AgentNode::jackalPositionCallback, this);

  //Behavior Tree declaration
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  BT::BehaviorTreeFactory factory;
  RegisterNodes(factory);

  auto tree = factory.createTreeFromFile(path + "/src/behaviour_tree.xml");
  //auto tree = factory.createTreeFromText(behavioiur_tree_xml);
  
  for(auto& node: tree.nodes)
  {
    if( auto go_near_charging_station = dynamic_cast<GoNearChargingStation*>(node.get()))
    {go_near_charging_station->init(this);}
    else if( auto recharge = dynamic_cast<Recharge*>(node.get())) {recharge->init(this);}
    else if( auto back_to_station = dynamic_cast<BackToStation*>(node.get())) {back_to_station->init(this);}
    else if( auto go_near_human_target = dynamic_cast<GoNearHumanTarget*>(node.get()))
    {go_near_human_target->init(this);}
    else if( auto monitor_human_target = dynamic_cast<MonitorHumanTarget*>(node.get()))
    {monitor_human_target->init(this);}
    else if( auto go_near_wp = dynamic_cast<GoNearWP*>(node.get())) {go_near_wp->init(this);}
    else if( auto take_image = dynamic_cast<TakeImage*>(node.get())) {take_image->init(this);}
    else if( auto inspect_pv_array = dynamic_cast<InspectPVArray*>(node.get())) {inspect_pv_array->init(this);}
    else if( auto go_near_station = dynamic_cast<GoNearStation*>(node.get())) {go_near_station->init(this);}
    else if( auto pick_tool = dynamic_cast<PickTool*>(node.get())) {pick_tool->init(this);}
    else if( auto drop_tool = dynamic_cast<DropTool*>(node.get())) {drop_tool->init(this);}
    else if( auto deliver_tool = dynamic_cast<DeliverTool*>(node.get())) {deliver_tool->init(this);}
    else if( auto mission_over = dynamic_cast<MissionOver*>(node.get())) {mission_over->init(this);}
    else if( auto idle = dynamic_cast<Idle*>(node.get())) {idle->init(this);}
    else if( auto is_battery_enough = dynamic_cast<IsBatteryEnough*>(node.get())) {is_battery_enough->init(this);}
    else if( auto is_battery_full = dynamic_cast<IsBatteryFull*>(node.get())) {is_battery_full->init(this);}
    else if( auto is_task_recharge = dynamic_cast<IsTaskRecharge*>(node.get())) {is_task_recharge->init(this);}
    else if( auto is_task_monitor = dynamic_cast<IsTaskMonitor*>(node.get())) {is_task_monitor->init(this);}
    else if( auto is_task_inspect = dynamic_cast<IsTaskInspect*>(node.get())) {is_task_inspect->init(this);}
    else if( auto is_task_inspect_pv_array = dynamic_cast<IsTaskInspectPVArray*>(node.get())) {is_task_inspect_pv_array->init(this);}
    else if( auto is_task_deliver_tool = dynamic_cast<IsTaskDeliverTool*>(node.get()))
    {is_task_deliver_tool->init(this);}
    else if( auto is_agent_near_charging_station = dynamic_cast<IsAgentNearChargingStation*>(node.get()))
    {is_agent_near_charging_station->init(this);}
    else if( auto is_agent_near_human_target = dynamic_cast<IsAgentNearHumanTarget*>(node.get()))
    {is_agent_near_human_target->init(this);}
    else if( auto is_agent_near_wp = dynamic_cast<IsAgentNearWP*>(node.get())) {is_agent_near_wp->init(this);}
    else if( auto need_to_drop_the_tool = dynamic_cast<NeedToDropTheTool*>(node.get()))
    {need_to_drop_the_tool->init(this);}
    else if( auto has_agent_the_tool = dynamic_cast<HasAgentTheTool*>(node.get())) {has_agent_the_tool->init(this);}
    else if( auto is_agent_near_station = dynamic_cast<IsAgentNearStation*>(node.get()))
    {is_agent_near_station->init(this);}
  }

  std::stringstream ss;
  ss << "bt_trace_" << beacon.id << ".fbl";
  BT::FileLogger logger_file(tree, ss.str().c_str());
  //BT::printTreeRecursively(tree.rootNode());
  BT::PublisherZMQ publisher_zmq(tree, 25, 1666 + (std::stoi(id_) - 1) * 2, 1667 + (std::stoi(id_) - 1) * 2);

  //Waiting for the Agent to initialize
  ROS_INFO("[AgentNode] Waiting for %s to initialize. State: %i", beacon.id.c_str(), state_);
  loop_rate_.reset();
  while(!state_ && ros::ok())
  {
    ros::spinOnce();
    loop_rate_.sleep();
  }
  ROS_INFO("[AgentNode] %s initialized. State: %i", beacon.id.c_str(), state_);

  //Behavior tree execution
  loop_rate_.reset();
  while(status == BT::NodeStatus::RUNNING && ros::ok())
  {
    //Agent only send beacons when it receibes beacons from Planner to avoid problems with the communication working 
    //only in one dirction. If Planner cant see Agent but this can see Planner, Planner sends an empty task list to the
    //"disconnected" agent to avoid collisions and repeated task in different UAVs
    status = tree.tickRoot();
    if(!checkBeaconTimeout(ros::Time::now()))
      beacon_pub_.publish(beacon_);
    isBatteryEnough();
    taskQueueManager();
    ros::spinOnce();
    loop_rate_.sleep();

  }
}
AgentNode::~AgentNode(){}
void AgentNode::readConfigFile(std::string config_file){
  YAML::Node yaml_config = YAML::LoadFile(config_file);
  if(yaml_config["positions"])
  {
    for(auto const& group : yaml_config["positions"])
    {
      for(auto const& position : group.second)
      {
        known_positions_[group.first.as<std::string>()][position.first.as<std::string>()] = 
          classes::Position(position.first.as<std::string>(), position.second['x'].as<float>(), 
              position.second['y'].as<float>(), position.second['z'].as<float>());
      }
    }
  }
  if(yaml_config["human_targets"])
  {
    for(auto const& human_target : yaml_config["human_targets"])
    {
      human_targets_[human_target.first.as<std::string>()] = classes::HumanTarget(human_target.first.as<std::string>(), 
          human_target.second['x'].as<float>(), human_target.second['y'].as<float>(),
          human_target.second['z'].as<float>());
    }
  }
  if(yaml_config["tools"])
  {
    for(auto const& tool : yaml_config["tools"])
    {
      tools_[tool.first.as<std::string>()] = classes::Tool(tool.first.as<std::string>(),
          tool.second["weight"].as<float>(), tool.second['x'].as<float>(), tool.second['y'].as<float>(),
          tool.second['z'].as<float>());
    }
  }
}
void AgentNode::readEvoraConfigFile(std::string config_file){
}
void AgentNode::isBatteryEnough(){
  bool previous_state = battery_enough_;
  bool empty_queue = task_queue_.empty();
  human_aware_collaboration_planner::BatteryEnoughGoal goal;
  //TODO: isBatteryEnough according to the task, its params, UAV type, UAV pose, current battery, etc
  //********************************************* FAKED *************************************************************
  /*
   * Esta condición dependerá de la tarea, del tipo de UAV, de la distancia a recorrer (posición del UAV, posición
   * de destino, paradas intermedias) y de la velocidad o inercia a frenar o aprovechar para llegar al destino
   */

  //Planner node has decided that this Agent should stop recharging to go on
  if(!battery_enough_ && !empty_queue)
    battery_enough_ = true;

  //Check if this Agent has battery enough to fulfill its next task
  //if(!empty_queue)
  if(battery_ < 0.3)
    battery_enough_ = false;

  //Charged battery
  if(battery_ > 0.95)
    battery_enough_ = true;

  if(previous_state != battery_enough_)
  {
    //Planner node has decided that this Agent should stop recharging to go on
    if(battery_enough_ && !empty_queue)
    {
      ROS_WARN("[isBatteryEnough] Planner has set me up");
      return;
    }
    //Other case: 
    ROS_WARN("[isBatteryEnough] Advertising that battery_enough_ = %s", battery_enough_ ? "true" : "false");
    emptyTheQueue();
    battery_ac_.waitForServer(ros::Duration(1.0));
    goal.value = battery_enough_;
    battery_ac_.sendGoal(goal);
    return;
  }
}
//Task queue methods
void AgentNode::addTaskToQueue(classes::Task* task){task_queue_.push(task);}
void AgentNode::removeTaskFromQueue(std::string id, char type){
  if(task_queue_.empty())
  {
    ROS_WARN("[removeTaskFromQueue] Task queue already empty");
    return;
  }
  classes::Task* task = task_queue_.front();

  if((id == task->getID()) && (type == task->getType()))
  {
    delete task;
    task_queue_.pop();
    return;
  }

  ROS_WARN("[removeTaskFromQueue] Task isn't in the first place of the queue. Not deleting.");
  return;
}
void AgentNode::emptyTheQueue(){
  classes::Task* task;
  while(!task_queue_.empty())
  {
    task = task_queue_.front();
    delete task;
    task_queue_.pop();
  }
}
int AgentNode::getQueueSize(){return task_queue_.size();}
void AgentNode::infoQueue(){
  classes::Task* tmp;
  char task_type;
  for(int i = 0; i < getQueueSize(); ++i)
  {
    tmp = task_queue_.front();
    task_type = tmp->getType();
    ROS_INFO_STREAM("" << tmp->getID() << ": " << (
          task_type == 'M' ? "Monitor" : 
          task_type == 'I' ? "Inspect" : 
          task_type == 'D' ? "DeliverTool" :
          task_type == 'R' ? "Recharge" :
          task_type == 'W' ? "Wait" :
          "Task"));
    task_queue_.push(task_queue_.front());
    task_queue_.pop();
  }
}
void AgentNode::taskQueueManager(){
  std::queue<classes::Task*> task_queue_aux(task_queue_);
  classes::Task* current_task;
  classes::Task* next_task;
  char next_task_type;
  float initial_percentage;
  actionlib::SimpleActionClient<human_aware_collaboration_planner::TaskResultAction> task_result_ac_("/" +
      beacon_.id + "/task_result", true);
  human_aware_collaboration_planner::TaskResultGoal goal;

  //Check if there are some waiting tasks in the queue
  if(task_queue_.size() < 2)
    return;
  task_queue_aux.pop();
  current_task = task_queue_.front();
  next_task = task_queue_aux.front();
  next_task_type = next_task->getType();
  //Check if the second task is a Recharge task
  if(next_task_type != 'R')
    return;
  initial_percentage = next_task->getInitialPercentage();
  //Check if the Agent's battery is equal or lower than the recharge task initial percentage
  if(battery_ > initial_percentage)
    return;
  //Halt the first task, delete it from queue and start the Recharge task
  goal.task.id = current_task->getID();
  goal.task.type = current_task->getType();
  ROS_INFO_STREAM("[taskQueueManager] " << goal.task.id << ": " << (
          goal.task.type == 'M' ? "Monitor" : 
          goal.task.type == 'I' ? "Inspect" : 
          goal.task.type == 'D' ? "DeliverTool" :
          goal.task.type == 'R' ? "Recharge" :
          goal.task.type == 'W' ? "Wait" :
          "Halted to Recharge"));
  task_result_ac_.waitForServer(ros::Duration(1.0));
  goal.result = 2; //TODO: Change with the result of Lower-level controllers
  task_result_ac_.sendGoal(goal);
  return;
}
//New Task List Action callback
void AgentNode::newTaskList(const human_aware_collaboration_planner::NewTaskListGoalConstPtr& goal){
  ROS_INFO("[newTaskList] Received a NewTaskList Action");
  ntl_feedback_.status = "Received a NewTaskList Action";
  ntl_as_.publishFeedback(ntl_feedback_);

  if(beacon_.id != goal->agent_id)
  {
    ROS_INFO("[newTaskList] Received wrong list. Incorrect agent ID");
    ntl_feedback_.status = "Received wrong list. Incorrect agent ID";
    ntl_as_.publishFeedback(ntl_feedback_);

    ntl_result_.ack = false;
    ntl_as_.setAborted(ntl_result_);
  }
  else
  {
    ntl_feedback_.status = "Reading the New Task List";
    ntl_as_.publishFeedback(ntl_feedback_);
    emptyTheQueue();

    for(int i = 0; i < goal->task_list.size(); ++i)
    {
      const human_aware_collaboration_planner::Task &task_msg = goal->task_list[i];
      classes::Task* task;

      switch(task_msg.type)
      {
        case 'M':
        case 'm':
          task = new classes::Monitor(task_msg.id, &(human_targets_[task_msg.monitor.human_target_id]), 
              task_msg.monitor.distance, task_msg.monitor.number, task_msg.monitor.agent_list);
          break;
        case 'I':
        case 'i':
          task = new classes::Inspect(task_msg.id, task_msg.inspect.waypoints, task_msg.inspect.agent_list);
          break;
        case 'D':
        case 'd':
          task = new classes::DeliverTool(task_msg.id, &(tools_[task_msg.deliver.tool_id]), 
              &(human_targets_[task_msg.deliver.human_target_id]));
          break;
        case 'R':
        case 'r':
          if(task_msg.recharge.charging_station_id.empty())
          {
            task = new classes::Recharge(task_msg.id, task_msg.recharge.initial_percentage,
                task_msg.recharge.final_percentage);
          }
          else
          {
            task = new classes::Recharge(task_msg.id,
                &(known_positions_["charging_stations"][task_msg.recharge.charging_station_id]),
                task_msg.recharge.initial_percentage, task_msg.recharge.final_percentage);
          }
          break;
        default:
          break;
      }
      addTaskToQueue(task);
    }

    ntl_feedback_.status = "Task queue saved";
    ntl_as_.publishFeedback(ntl_feedback_);

    ntl_result_.ack = true;
    ntl_as_.setSucceeded(ntl_result_);

    infoQueue();
  }
}
void AgentNode::positionCallbackUAL(const geometry_msgs::PoseStamped& pose){
  //TODO: Comment the following when lower level controller for travelling are integrated
  //This correction is needed becouse of the UAL/Land service
  if(pose.pose.position.z - std::stoi(id_) < 0.5)
    position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  else
    position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z - std::stoi(id_));
  //position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}
void AgentNode::positionCallbackMRS(const mrs_msgs::UavStatus& pose){
  //TODO: Comment the following when lower level controller for travelling are integrated
  //This correction is needed becouse of the UAL/Land service
  if(pose.odom_z - std::stoi(id_) < 0.5)
    position_.update(pose.odom_x, pose.odom_y, pose.odom_z);
  else
    position_.update(pose.odom_x, pose.odom_y, pose.odom_z - std::stoi(id_));
  //position_.update(pose.odom_x, pose.odom_y, pose.odom_z);
}
void AgentNode::batteryCallback(const sensor_msgs::BatteryState& battery){battery_ = battery.percentage;}
void AgentNode::stateCallbackUAL(const uav_abstraction_layer::State& state){state_ = state.state;}
void AgentNode::stateCallbackMRS(const mrs_actionlib_interface::State& state){
  /* UAL States                 MRS States
   *********************        *********************
   * UNINITIALIZED   = 0        * LANDED          = 0
   * LANDED_DISARMED = 1        * TAKEOFF_LANDING = 1
   * LANDED_ARMED    = 2        * IDLE_FLYING     = 2
   * TAKING_OFF      = 3        * GOTO_PATHFINDER = 3
   * FLYING_AUTO     = 4        * GOTO_DIRECT     = 4
   * FLYING_MANUAL   = 5        * UNKNOWN         = 5 
   * LANDING         = 6        * 
   *********************        *********************/

  switch(state.state){
    case 0:
      state_ = 2; //state_ = 1;
      break;
    case 1:
      state_ = 6; //state_ = 3;
      break;
    case 2:
    case 3:
    case 4:
      state_ = 4;
      break;
    case 5:
    default:
      state_ = 0; //state_ = 5;
      break;
  }
  return;
}
void AgentNode::missionOverCallback(const human_aware_collaboration_planner::MissionOver& value){
  mission_over_ = value.value;
}
void AgentNode::beaconCallback(const human_aware_collaboration_planner::PlannerBeacon& beacon){
  last_beacon_ = beacon.time;
  timeout_ = false;
  beacon_.timeout = false;
}
bool AgentNode::checkBeaconTimeout(ros::Time now){
  ros::Duration timeout(5);
  if(((now - last_beacon_) > timeout) && !timeout_)
  {
    ROS_WARN("[checkBeaconTimeout] Beacon timeout. Disconnection detected. Emptying the task queue.");
    emptyTheQueue();
    timeout_ = true;
    beacon_.timeout = true;
  }
  return timeout_;
}
void AgentNode::atrvjrPositionCallback(const geographic_msgs::GeoPoseStamped& geo_pose){ 
  geometry_msgs::Point32 pose = geographic_to_cartesian(geo_pose.pose.position, origin_geo_);
  atrvjr_pose_ = classes::Position(pose.x, pose.y, 3); //As UGV are at ground level, z id fiex at 3m for goto purposes
  return;
}
void AgentNode::jackalPositionCallback(const geographic_msgs::GeoPoseStamped& geo_pose){ 
  geometry_msgs::Point32 pose = geographic_to_cartesian(geo_pose.pose.position, origin_geo_);
  jackal_pose_ = classes::Position(pose.x, pose.y, 3); //As UGV are at ground level, z id fiex at 3m for goto purposes
  return;
}
// UAL/MRS Service calls
bool AgentNode::land(bool blocking){
  if(low_level_interface_ == "UAL")
  {
    ros::ServiceClient land_client = nh_.serviceClient<uav_abstraction_layer::Land>("/" + beacon_.id + "/ual/land");
    uav_abstraction_layer::Land land_srv;
    land_srv.request.blocking = blocking;
    if(land_client.call(land_srv))
      return true;
    return false;
  }
  if(low_level_interface_ == "MRS")
  {
    mrs_ac_.waitForServer(ros::Duration(1.0));
    mrs_actionlib_interface::commandGoal goal;
    goal.command = 1;
    ROS_INFO("[land] Calling mrs_actionlib_interface action command LAND");
    mrs_ac_.sendGoal(goal);
    return true;
  }
  return false;
}
bool AgentNode::take_off(float height, bool blocking){
  if(low_level_interface_ == "UAL")
  {
    ros::ServiceClient take_off_client = nh_.serviceClient<uav_abstraction_layer::TakeOff>("/" + beacon_.id +
        "/ual/take_off");
    uav_abstraction_layer::TakeOff take_off_srv;
    //TODO: Change the following two lines when lower level controller for travelling are integrated
    take_off_srv.request.height = height + std::stoi(id_);
    //take_off_srv.request.height = height;
    take_off_srv.request.blocking = blocking;
    if(take_off_client.call(take_off_srv))
      return true;
    return false;
  }
  if(low_level_interface_ == "MRS")
  {
    mrs_ac_.waitForServer(ros::Duration(1.0));
    mrs_actionlib_interface::commandGoal goal;
    goal.command = 0;
    ROS_INFO("[take_off] Calling mrs_actionlib_interface action command TAKEOFF");
    mrs_ac_.sendGoal(goal);
    /*
    //TODO: Delete the following ten lines when lower level controller for travelling are integrated
    //Goto command to change the height because it's not an option in takeoff command with the MRS System
    if(mrs_ac_.waitForResult(ros::Duration(10.0)))
    {
      goal.command = 3;
      goal.goto_x = position_.getX();
      goal.goto_y = position_.getY();
      goal.goto_z = height + std::stoi(id_);
      goal.goto_hdg = 0;
      mrs_ac_.sendGoal(goal);
      return true;
    }
    */
    return true;
  }
  return false;
}
bool AgentNode::go_to_waypoint(float x, float y, float z, bool blocking){
  if(low_level_interface_ == "UAL")
  {
    ros::ServiceClient go_to_wp_client = nh_.serviceClient<uav_abstraction_layer::GoToWaypoint>("/" + beacon_.id +
        "/ual/go_to_waypoint");
    uav_abstraction_layer::GoToWaypoint go_to_wp_srv;
    go_to_wp_srv.request.waypoint.header.frame_id = pose_frame_id_;
    go_to_wp_srv.request.waypoint.pose.position.x = x;
    go_to_wp_srv.request.waypoint.pose.position.y = y;
    //TODO: Change the following four lines when lower level controller for travelling are integrated
    if(z < 1)
      go_to_wp_srv.request.waypoint.pose.position.z = z;
    else
      go_to_wp_srv.request.waypoint.pose.position.z = z + std::stoi(id_);
    //go_to_wp_srv.request.waypoint.pose.position.z = z;
    go_to_wp_srv.request.blocking = blocking;
    if(go_to_wp_client.call(go_to_wp_srv))
      return true;
    return false;
  }
  if(low_level_interface_ == "MRS")
  {
    mrs_ac_.waitForServer(ros::Duration(1.0));
    mrs_actionlib_interface::commandGoal goal;
    goal.command = 3;
    goal.goto_x = x;
    goal.goto_y = y;
    //TODO: Change the following four lines when lower level controller for travelling are integrated
    if(z < 1)
      goal.goto_z = z;
    else
      goal.goto_z = z + std::stoi(id_);
    //goal.goto_z = z;
    goal.goto_hdg = 0;
    ROS_INFO("[go_to_waypoint] Calling mrs_actionlib_interface action command GOTO");
    mrs_ac_.sendGoal(goal);
    return true;
  }
  return false;
}
bool AgentNode::stop(bool blocking){
  if(state_ == 4)
  {
    if(low_level_interface_ == "UAL")
    {
      ros::ServiceClient stop_client = nh_.serviceClient<uav_abstraction_layer::GoToWaypoint>("/" + beacon_.id +
          "/ual/go_to_waypoint");
      uav_abstraction_layer::GoToWaypoint stop_srv;
      stop_srv.request.waypoint.header.frame_id = pose_frame_id_;
      stop_srv.request.waypoint.pose.position.x = position_.getX();
      stop_srv.request.waypoint.pose.position.y = position_.getY();
      //TODO: Change the following four lines when lower level controller for travelling are integrated
      if(position_.getZ() < 1)
        stop_srv.request.waypoint.pose.position.z = position_.getZ();
      else
        stop_srv.request.waypoint.pose.position.z = position_.getZ() + std::stoi(id_);
      //stop_srv.request.waypoint.pose.position.z = position_.getZ();
      stop_srv.request.blocking = blocking;
      if(!stop_client.call(stop_srv))
        return false;
    }
    if(low_level_interface_ == "MRS")
    {
      mrs_ac_.waitForServer(ros::Duration(1.0));
      mrs_actionlib_interface::commandGoal goal;
      goal.command = 3;
      goal.goto_x = position_.getX();
      goal.goto_y = position_.getY();
      //TODO: Change the following four lines when lower level controller for travelling are integrated
      if(position_.getZ() < 1)
        goal.goto_z = position_.getZ();
      else
        goal.goto_z = position_.getZ() + std::stoi(id_);
      //goal.goto_z = position_.getZ();
      goal.goto_hdg = 0;
      mrs_ac_.sendGoal(goal);
      return true;
    }
  }
  return true;
}
bool AgentNode::checkIfGoToServiceSucceeded(float x, float y, float z){
  classes::Position position(x, y, z);
  if(classes::distance(position, position_) < 0.1)
    return true;
  return false;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "agent_behaviour_manager");

  std::string id;
  std::string ns_prefix;
  human_aware_collaboration_planner::AgentBeacon beacon;

  //Read AgentBeacon parameters and send AgentBeacon to planner
  ros::param::param<std::string>("~id", id, "0");
  ros::param::param<std::string>("~ns_prefix", ns_prefix, "uav");
  ros::param::param<std::string>("~type", beacon.type, "ACW");
  beacon.id = ns_prefix + id;

  AgentNode agent(beacon);

  return 0;
}
