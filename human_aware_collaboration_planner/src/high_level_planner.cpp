#include "human_aware_collaboration_planner/high_level_planner.h"

//class Agent
//class Agent Constructors
Agent::Agent() : id_(), type_(), position_(), battery_(), task_queue_(), position_sub_(), battery_sub_(),
  battery_as_(nh_, "/uav0/battery_enough", boost::bind(&Agent::batteryEnoughCB, this, _1), false),
  task_result_as_(nh_, "/uav0/task_result", boost::bind(&Agent::taskResultCB, this, _1), false),
  ntl_ac_("/uav0/task_list", true), battery_enough_(true), planner_(nullptr), last_beacon_time_(), last_beacon_(),
  low_level_interface_(), pose_topic_(), battery_topic_() {}

Agent::Agent(Planner* planner, std::string id, std::string type, ros::Time first_beacon_time,
    human_aware_collaboration_planner::AgentBeacon first_beacon) :
  planner_(planner), id_(id), type_(type), position_(), battery_(), task_queue_(), battery_enough_(true),
  battery_as_(nh_, "/" + id + "/battery_enough", boost::bind(&Agent::batteryEnoughCB, this, _1), false), 
  task_result_as_(nh_, "/" + id + "/task_result", boost::bind(&Agent::taskResultCB, this, _1), false),
  ntl_ac_("/" + id + "/task_list", true), last_beacon_time_(first_beacon_time), last_beacon_(first_beacon)
{
  ros::param::param<std::string>("~low_level_interface", low_level_interface_, "UAL");
  ros::param::param<std::string>("~pose_topic", pose_topic_, "/ual/pose");
  ros::param::param<std::string>("~battery_topic", battery_topic_, "/battery_fake");

  if(low_level_interface_ == "UAL")
    position_sub_ = nh_.subscribe("/" + id + pose_topic_, 1, &Agent::positionCallbackUAL, this);
  else if(low_level_interface_ == "MRS")
    position_sub_ = nh_.subscribe("/" + id + pose_topic_, 1, &Agent::positionCallbackMRS, this);
  battery_sub_ = nh_.subscribe("/" + id + battery_topic_, 1, &Agent::batteryCallback, this);
  battery_as_.start();
  task_result_as_.start();
}

Agent::Agent(const Agent& a) : id_(a.id_), type_(a.type_), position_(a.position_), battery_(a.battery_),
  task_queue_(a.task_queue_), battery_enough_(a.battery_enough_), planner_(a.planner_), last_beacon_(a.last_beacon_),
  battery_as_(nh_, "/" + a.id_ + "/battery_enough", boost::bind(&Agent::batteryEnoughCB, this, _1), false), 
  task_result_as_(nh_, "/" + a.id_ + "/task_result", boost::bind(&Agent::taskResultCB, this, _1), false),
  ntl_ac_("/" + a.id_ + "/task_list", true), last_beacon_time_(a.last_beacon_time_),
  low_level_interface_(a.low_level_interface_), pose_topic_(a.pose_topic_), battery_topic_(a.battery_topic_)
{
  if(low_level_interface_ == "UAL")
    position_sub_ = nh_.subscribe("/" + a.id_ + a.pose_topic_, 1, &Agent::positionCallbackUAL, this);
  else if(low_level_interface_ == "MRS")
    position_sub_ = nh_.subscribe("/" + a.id_ + a.pose_topic_, 1, &Agent::positionCallbackMRS, this);
  battery_sub_ = nh_.subscribe("/" + a.id_ + a.battery_topic_, 1, &Agent::batteryCallback, this);

  battery_as_.start();
  task_result_as_.start();
}

Agent::~Agent()
{
  position_sub_.shutdown();
  battery_sub_.shutdown();
  battery_as_.shutdown();
  task_result_as_.shutdown();
}

//class Agent Topic methods
void Agent::updateSensorsInformation(){}
bool Agent::isBatteryForQueue(){
  //TODO: isBatteryForQueue according to the task list, its params, UAV type, UAV pose, current battery, etc
  /*
   * Esta condición dependerá de la tarea, del tipo de UAV, de la distancia a recorrer (posición del UAV, posición
   * de destino, paradas intermedias) y de la velocidad o inercia a frenar o aprovechar para llegar al destino
   * */
  /*
   * Esta función se ejecuta desde el nodo Planner. Si el Agente tiene batería suficiente, no se hace nada. Si el
   * agente no tiene batería suficiente para cumplir la lista completa de taréas, el planner decide si se reasignan
   * tareas o si se dejan tal y como están.
   */
  return true;
}
bool Agent::isBatteryEnough(classes::Task* task){
  //TODO: isBatteryEnough. Think what to do here when all is improved and if this function still neccesary.
  //Maybe make this function in charge of compute the expected battery level along the plan to introduce Recharges

  //battery_enough is set by the agent behavior manager. Its computed there for safety reasons.
  //However, High-Level Planner can make the UAV to stop recharging by changing the flag here. (current version idea)
  if(battery_enough_)
    return true;
  if(task->getType() == 'I' && battery_ > 0.5)
    battery_enough_ = true;
  return battery_enough_;
}

bool Agent::checkBeaconTimeout(ros::Time now){
  ros::Duration timeout(5);
  if((now - last_beacon_time_) > timeout)
    return true;
  else
    return false;
}

//class Agent Task queue methods
bool Agent::isQueueEmpty(){return task_queue_.empty();}

void Agent::emptyTheQueue(){
  while(!task_queue_.empty())
    task_queue_.pop();
}

void Agent::addTaskToQueue(classes::Task* task){task_queue_.push(task);}

void Agent::replaceTaskFromQueue(std::string task_id){
  classes::Task* aux = new classes::Task();//Memory Leak
  for(int i = 0; i < task_queue_.size(); ++i)
  {
    if(task_queue_.front()->getID() == task_id)
      task_queue_.push(aux);
    else
      task_queue_.push(task_queue_.front());
    task_queue_.pop();
  }
}

classes::Task* Agent::getFirstTask(){return task_queue_.front();}

classes::Task* Agent::getOldFirstTask(){return old_task_queue_.front();}

classes::Task* Agent::getLastTask(){return task_queue_.back();}

bool Agent::isTaskInQueue(std::string task_id){
  for(int i = 0; i < task_queue_.size(); ++i)
  {
    if(task_queue_.front()->getID() == task_id)
      return true;
  }
  return false;
}

void Agent::setOldTaskQueue(){old_task_queue_ = task_queue_;}

void Agent::deleteOldTaskQueue(){
  //Delete memory from invalid tasks only. Valid tasks' data still needed.
  for(int i = 0; i < old_task_queue_.size(); ++i)
  {
    if(old_task_queue_.front()->getType() == 'T')
      delete old_task_queue_.front();
    old_task_queue_.pop();
  }
}

int Agent::getQueueSize(){return task_queue_.size();}
void Agent::sendQueueToAgent(){
  ntl_ac_.waitForServer(ros::Duration(1.0));
  human_aware_collaboration_planner::NewTaskListGoal goal;

  goal.agent_id = id_;

  auto queue_size = task_queue_.size(); 
  for(int i = 0; i < queue_size; ++i)
  {
    human_aware_collaboration_planner::Task task_msg;
    classes::Task* task = task_queue_.front();

    task_msg.id = task->getID();
    task_msg.type = task->getType();

    switch(task_msg.type)
    {
      case 'M':
      case 'm':
        task_msg.monitor.human_target_id = task->getHumanID();
        task_msg.monitor.distance = task->getDistance();
        task_msg.monitor.number = task->getNumber();
        task_msg.monitor.agent_list = task->getAgentList();
        break;
      case 'I':
      case 'i':
        task_msg.inspect.waypoints = task->getInspectWaypoints();
        task_msg.inspect.agent_list = task->getAgentList();
        break;
      case 'A':
      case 'a':
        task_msg.inspect.waypoints = task->getInspectWaypoints();
        task_msg.inspect.agent_list = task->getAgentList();
        break;
      case 'D':
      case 'd':
        task_msg.deliver.tool_id = task->getToolID();
        task_msg.deliver.human_target_id = task->getHumanID();
        break;
      case 'R':
      case 'r':
        task_msg.recharge.charging_station_id = task->getChargingStationID();
        task_msg.recharge.initial_percentage = task->getInitialPercentage();
        task_msg.recharge.final_percentage = task->getFinalPercentage();
        break;
      default:
        break;
    }

    goal.task_list.push_back(task_msg);

    task_queue_.push(task_queue_.front());
    task_queue_.pop();
  }

  ntl_ac_.sendGoal(goal);
}

float Agent::computeTaskCost(classes::Task* task){
  float a, t, i;
  //float a, t, b, i;
  float agent_type_cost;
  float traveling_cost;
  //float battery_cost;
  float interruption_cost;

  //Costs weights. TODO: calibrate the costs
  a = 50;
  t = 1;
  //b = 5;
  i = 3;

  //Agent type cost
  if(type_ == "PhysicalACW")
  {
    switch(task->getType())
    {
      case 'M':
      case 'm':
        agent_type_cost = 1;
        break;
      case 'I':
      case 'i':
        agent_type_cost = 1;
        break;
      case 'A':
      case 'a':
        agent_type_cost = 1;
        break;
      case 'D':
      case 'd':
        agent_type_cost = 0;
        break;
      default:
        agent_type_cost = 0;
        break;
    }
  }
  else if(type_ == "InspectionACW")
  {
    switch(task->getType())
    {
      case 'M':
      case 'm':
        agent_type_cost = 0.5;
        break;
      case 'I':
      case 'i':
        agent_type_cost = 0;
        break;
      case 'A':
      case 'a':
        agent_type_cost = 0;
        break;
      case 'D':
      case 'd':
        agent_type_cost = 1;
        break;
      default:
        agent_type_cost = 0;
        break;
    }
  }
  else if(type_ == "SafetyACW")
  {
    switch(task->getType())
    {
      case 'M':
      case 'm':
        agent_type_cost = 0;
        break;
      case 'I':
      case 'i':
        agent_type_cost = 1;
        break;
      case 'A':
      case 'a':
        agent_type_cost = 1;
        break;
      case 'D':
      case 'd':
        agent_type_cost = 1;
        break;
      default:
        agent_type_cost = 0;
        break;
    }
  }

  //Traveling cost
  if(task_queue_.empty())
  {
    classes::Position human_position;
    classes::Position tool_position;
    human_aware_collaboration_planner::Waypoint central_position;
    std::vector<human_aware_collaboration_planner::Waypoint> inspect_waypoints;
    switch(task->getType())
    {
      case 'M':
      case 'm':
        human_position = task->getHumanPosition();
        traveling_cost = classes::distance(position_, human_position);
        break;
      case 'I':
      case 'i':
        inspect_waypoints = task->getInspectWaypoints();
        central_position = classes::central_position(inspect_waypoints);
        traveling_cost = classes::distance(position_, central_position);
        break;
      case 'A':
      case 'a':
        inspect_waypoints = task->getInspectWaypoints();
        central_position = classes::central_position(inspect_waypoints);
        traveling_cost = classes::distance(position_, central_position);
        break;
      case 'D':
      case 'd':
        tool_position = task->getToolPosition();
        human_position = task->getHumanPosition();
        traveling_cost = classes::distance(position_, tool_position) + classes::distance(tool_position, human_position); 
        break;
      default:
        traveling_cost = 0;
        break;
    }
  }
  else
  {
    classes::Position human_position;
    classes::Position previous_human_position;
    classes::Position previous_charging_station;
    classes::Position tool_position;
    human_aware_collaboration_planner::Waypoint central_position;
    human_aware_collaboration_planner::Waypoint previous_central_position;
    std::vector<human_aware_collaboration_planner::Waypoint> inspect_waypoints;
    std::vector<human_aware_collaboration_planner::Waypoint> previous_inspect_waypoints;
    classes::Task* previous_task = getLastTask();
    switch(previous_task->getType())
    {
      case 'M':
      case 'm':
        switch(task->getType())
        {
          case 'M':
          case 'm':
            previous_human_position = previous_task->getHumanPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, human_position);
            break;
          case 'I':
          case 'i':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;
          case 'A':
          case 'a':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;
          case 'D':
          case 'd':
            previous_human_position = previous_task->getHumanPosition();
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position , tool_position) + 
              classes::distance(tool_position, human_position); 
            break;
          default:
            traveling_cost = 0;
            break;
        }
        break;
      case 'I':
      case 'i':
        switch(task->getType())
        {
          case 'M':
          case 'm':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_central_position, human_position);
            break;
          case 'I':
          case 'i':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = previous_task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;
          case 'A':
          case 'a':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = previous_task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;
          case 'D':
          case 'd':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_central_position, tool_position) + 
              classes::distance(tool_position, human_position); 
            break;
          default:
            traveling_cost = 0;
            break;
        }
        break;
      case 'A':
      case 'a':
        switch(task->getType())
        {
          case 'M':
          case 'm':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_central_position, human_position);
            break;
          case 'I':
          case 'i':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = previous_task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;
          case 'A':
          case 'a':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            inspect_waypoints = previous_task->getInspectWaypoints();
            central_position = classes::central_position(previous_inspect_waypoints);
            traveling_cost = classes::distance(previous_central_position, central_position);
            break;
          case 'D':
          case 'd':
            previous_inspect_waypoints = previous_task->getInspectWaypoints();
            previous_central_position = classes::central_position(previous_inspect_waypoints);
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_central_position, tool_position) + 
              classes::distance(tool_position, human_position); 
            break;
          default:
            traveling_cost = 0;
            break;
        }
        break;
      case 'D':
      case 'd':
        switch(task->getType())
        {
          case 'M':
          case 'm':
            previous_human_position = previous_task->getHumanPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position, human_position);
            break;
          case 'I':
          case 'i':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;
          case 'A':
          case 'a':
            previous_human_position = previous_task->getHumanPosition();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_human_position, central_position);
            break;
          case 'D':
          case 'd':
            previous_human_position = previous_task->getHumanPosition();
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_human_position , tool_position) + 
              classes::distance(tool_position, human_position); 
            break;
          default:
            traveling_cost = 0;
            break;
        }
        break;
      case 'R':
      case 'r':
        switch(task->getType())
        {
          case 'M':
          case 'm':
            previous_charging_station = previous_task->getChargingStation();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_charging_station, human_position);
            break;
          case 'I':
          case 'i':
            previous_charging_station = previous_task->getChargingStation();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_charging_station, central_position);
            break;
          case 'A':
          case 'a':
            previous_charging_station = previous_task->getChargingStation();
            inspect_waypoints = task->getInspectWaypoints();
            central_position = classes::central_position(inspect_waypoints);
            traveling_cost = classes::distance(previous_charging_station, central_position);
            break;
          case 'D':
          case 'd':
            previous_charging_station = previous_task->getChargingStation();
            tool_position = task->getToolPosition();
            human_position = task->getHumanPosition();
            traveling_cost = classes::distance(previous_charging_station, tool_position) + 
              classes::distance(tool_position, human_position); 
            break;
          default:
            traveling_cost = 0;
            break;
        }
        break;
      default:
        traveling_cost = 0;
        break;
    }
  }

  //Battery cost: almost the same that travelling cost. The diference is the battery spent in not traveling parts of the
  //task, but that part's cost is the same regardless the agent who perform the task considering the agent type cost.
  //So, for the moment we are just checking if the Agent has battery enough for its first task.

  //Interruption cost
  if(old_task_queue_.empty())
    interruption_cost = 0;
  else if(task_queue_.empty())
  {
    classes::Task* old_first_task = getOldFirstTask();
    if(old_first_task == task)
      interruption_cost = -1;
    else
    {
      switch(old_first_task->getType())
      {
        case 'M':
        case 'm':
          switch(task->getType())
          {
            case 'M':
            case 'm':
              interruption_cost = 1;
              break;
            case 'I':
            case 'i':
              interruption_cost = 0;
              break;
            case 'A':
            case 'a':
              interruption_cost = 0;
              break;
            case 'D':
            case 'd':
              interruption_cost = 0;
              break;
            default:
              interruption_cost = 0;
              break;
          }
          break;
        case 'I':
        case 'i':
          switch(task->getType())
          {
            case 'M':
            case 'm':
              interruption_cost = 2;
              break;
            case 'I':
            case 'i':
              interruption_cost = 1;
              break;
            case 'A':
            case 'a':
              interruption_cost = 1;
              break;
            case 'D':
            case 'd':
              interruption_cost = 0;
              break;
            default:
              interruption_cost = 0;
              break;
          }
          break;
        case 'A':
        case 'a':
          switch(task->getType())
          {
            case 'M':
            case 'm':
              interruption_cost = 2;
              break;
            case 'I':
            case 'i':
              interruption_cost = 1;
              break;
            case 'A':
            case 'a':
              interruption_cost = 1;
              break;
            case 'D':
            case 'd':
              interruption_cost = 0;
              break;
            default:
              interruption_cost = 0;
              break;
          }
          break;
        case 'D':
        case 'd':
          switch(task->getType())
          {
            case 'M':
            case 'm':
              interruption_cost = 3;
              break;
            case 'I':
            case 'i':
              interruption_cost = 2;
              break;
            case 'A':
            case 'a':
              interruption_cost = 2;
              break;
            case 'D':
            case 'd':
              interruption_cost = 1;
              break;
            default:
              interruption_cost = 0;
              break;
          }
          break;
        default:
          interruption_cost = 0;
          break;
      }
    }
  }
  else
    interruption_cost = 0;

  return a*agent_type_cost + t*traveling_cost + i*interruption_cost;
  //return a*agent_type_cost + t*traveling_cost + b*battery_cost + i*interruption_cost;
}

//class Agent Getters
std::string Agent::getID(){return id_;}
std::string Agent::getType(){return type_;}
bool Agent::getLastBeaconTimeout(){return last_beacon_.timeout;}

//class Agent Setters
void Agent::setLastBeaconTime(ros::Time last_beacon_time){last_beacon_time_ = last_beacon_time;}
void Agent::setLastBeacon(human_aware_collaboration_planner::AgentBeacon last_beacon){last_beacon_ = last_beacon;}

//class Agent Callbacks
void Agent::positionCallbackUAL(const geometry_msgs::PoseStamped& pose){
  position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}
void Agent::positionCallbackMRS(const mrs_msgs::UavStatus& pose){
  position_.update(pose.odom_x, pose.odom_y, pose.odom_z);
}
void Agent::batteryCallback(const sensor_msgs::BatteryState& battery){battery_ = battery.percentage;}
void Agent::batteryEnoughCB(const human_aware_collaboration_planner::BatteryEnoughGoalConstPtr& goal){
  //TODO: planned battery recharges initial percentage should always be higher than the emergency !battery_enough_
  battery_enough_ = goal->value;
  ROS_WARN_STREAM("[batteryEnoughCB] (" << id_ << ") Noticed that battery_enough_ = " << (battery_enough_ ? "true" :
        "false"));
  battery_feedback_.status = "battery_enough_ updated";
  battery_as_.publishFeedback(battery_feedback_);

  if (battery_as_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO_STREAM("[batteryEnoughCB] (" << id_ << ") BatteryEnough: Preempted");
    battery_as_.setPreempted();
  }

  battery_result_.ack = true;
  battery_as_.setSucceeded(battery_result_);

  planner_->performTaskAllocation(); 
}

void Agent::taskResultCB(const human_aware_collaboration_planner::TaskResultGoalConstPtr& goal){
  human_aware_collaboration_planner::TaskResultResult task_result_result_;
  classes::Task* task = planner_->getPendingTask(goal->task.id);
  char task_type;

  //Check if task ended becouse mission is over
  if(planner_->getMissionOver())
  {
    task_type = task->getType();
    ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << "(" << (
            task_type == 'M' ? "Monitor" : 
            task_type == 'I' ? "Inspect" : 
            task_type == 'A' ? "InspectPVArray" : 
            task_type == 'D' ? "DeliverTool" :
            task_type == 'R' ? "Recharge" :
            task_type == 'W' ? "Wait" : 
            "Task") << ") Halted becouse mission is over.");
    task_result_result_.ack = true;
    task_result_as_.setSucceeded(task_result_result_);
    return;
  }

  //Check if the task still exists
  if(task == nullptr)
  {
    ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") An already deleted Multi-UAV task ended");
    task_result_result_.ack = true;
    task_result_as_.setSucceeded(task_result_result_);
    return;
  }
  task_type = task->getType();

  //Check if this task's action node ended becouse a task with the same ID arrived
  if(goal->task.type != task_type)
  {
    ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << "(" << goal->task.type << ")"
      << ") Halted becouse of a task (" << (
          task_type == 'M' ? "Monitor" : 
          task_type == 'I' ? "Inspect" : 
          task_type == 'A' ? "InspectPVArray" : 
          task_type == 'D' ? "DeliverTool" :
          task_type == 'R' ? "Recharge" :
          task_type == 'W' ? "Wait" : 
          "Task") << ") with a duplicated ID.");
    task_result_result_.ack = true;
    task_result_as_.setSucceeded(task_result_result_);
    return;
  }

  //If task has been HALTED by the task queue manager as scheduled. Not deleted in this case (relieved or postponed)
  if(goal->result == 2)
  {
    ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << "(" << (
            task_type == 'M' ? "Monitor" : 
            task_type == 'I' ? "Inspect" : 
            task_type == 'A' ? "InspectPVArray" : 
            task_type == 'D' ? "DeliverTool" :
            task_type == 'R' ? "Recharge" :
            task_type == 'W' ? "Wait" : 
            "Task") << ") HALTED by the task queue manager to Recharge. Reallocating...");
    if(task_queue_.front() == task)
      task_queue_.pop();
    planner_->performTaskAllocation();
  }
  //If task ended with SUCCESS, delete task from Planner's pending tasks list
  else if(goal->result)
  {
    ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << "(" << (
            task_type == 'M' ? "Monitor" : 
            task_type == 'I' ? "Inspect" : 
            task_type == 'A' ? "InspectPVArray" : 
            task_type == 'D' ? "DeliverTool" :
            task_type == 'R' ? "Recharge" :
            task_type == 'W' ? "Wait" : 
            "Task") << ") SUCCEEDED. Reallocating just in case...");
    if(task_queue_.front() == task)
      task_queue_.pop();
    planner_->deletePendingTask(goal->task.id);
    planner_->performTaskAllocation();
  }
  //If task ended with FAILURE 
  else
  {
    //Check if task has been halted becouse of not having battery enough
    if(!battery_enough_)
    {
      ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << " (" << (
              task_type == 'M' ? "Monitor" : 
              task_type == 'I' ? "Inspect" : 
              task_type == 'A' ? "InspectPVArray" : 
              task_type == 'D' ? "DeliverTool" :
              task_type == 'R' ? "Recharge" :
              task_type == 'W' ? "Wait" : 
              "Task") << ") in " << id_ << " FAILED due to low battery.");
      //TODO: planned battery recharges initial percentage should always be higher than the emergency !battery_enough_
      //planner_->performTaskAllocation(); //Not needed becouse has already been executed in batteryEnoughCB
    }
    //Check if task has been halted becouse of not having battery enough but has't been communicated yet
    else if(battery_ < 0.3)
    {
      ROS_WARN_STREAM("[taskResultCB] (" << id_ << 
          ") Halt() is executed before the new node's tick(), so taskResultCB() runs before batteryEnoughCB()");
      ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << " (" << (
              task_type == 'M' ? "Monitor" : 
              task_type == 'I' ? "Inspect" : 
              task_type == 'A' ? "InspectPVArray" : 
              task_type == 'D' ? "DeliverTool" :
              task_type == 'R' ? "Recharge" :
              task_type == 'W' ? "Wait" : 
              "Task") << ") in " << id_ << " FAILED due to low battery.");
      //TODO: planned battery recharges initial percentage should always be higher than the emergency !battery_enough_
      //planner_->performTaskAllocation(); //Not needed becouse has already been executed in batteryEnoughCB
    }
    //First task failed. Unplanned event. Reallocation needed. Task deleted
    else if(task == task_queue_.front())
    {
      ROS_ERROR_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << " (" << (
              task_type == 'M' ? "Monitor" : 
              task_type == 'I' ? "Inspect" : 
              task_type == 'A' ? "InspectPVArray" : 
              task_type == 'D' ? "DeliverTool" :
              task_type == 'R' ? "Recharge" :
              task_type == 'W' ? "Wait" : 
              "Task") << ") FAILED. Deleting and Reallocating...");
      task_queue_.pop();
      planner_->deletePendingTask(goal->task.id);
      planner_->performTaskAllocation();
    }
    else
      ROS_INFO_STREAM("[taskResultCB] (" << id_ << ") " << goal->task.id << " (" << (
              task_type == 'M' ? "Monitor" : 
              task_type == 'I' ? "Inspect" : 
              task_type == 'A' ? "InspectPVArray" : 
              task_type == 'D' ? "DeliverTool" :
              task_type == 'R' ? "Recharge" :
              task_type == 'W' ? "Wait" : 
              "Task") << ") in " << id_ << " FAILED but seems to be planned (reallocation).");
  }

  task_result_result_.ack = true;
  task_result_as_.setSucceeded(task_result_result_);
  return;
}

//class Agent Visualization method
void Agent::print(std::ostream& os){
  os << "Agent id: " << id_; //<< std::endl;
  //os << "Agent type: " << type_ << std::endl;
  //os << "Position: " << position_.getX() << ", " << position_.getY() << ", " << position_.getZ() << std::endl;
  //os << "Battery percentage: " << battery_ << std::endl;
  //os << "Task list: (" << task_queue_.size() << " tasks)";
  classes::Task* tmp;
  char task_type;
  auto queue_size = task_queue_.size(); 
  for(int i = 0; i < queue_size; ++i)
  {
    tmp = task_queue_.front();
    task_type = tmp->getType();
    os << "\n\t" << tmp->getID() << ": " << (
        task_type == 'M' ? "Monitor" : 
        task_type == 'I' ? "Inspect" : 
        task_type == 'A' ? "InspectPVArray" : 
        task_type == 'D' ? "DeliverTool" :
        task_type == 'R' ? "Recharge" :
        task_type == 'W' ? "Wait" : 
        "Task");
    task_queue_.push(task_queue_.front());
    task_queue_.pop();
  }
}

//Planner definitions
Planner::Planner(human_aware_collaboration_planner::PlannerBeacon beacon) : 
  nt_as_(nh_, "incoming_task_action", boost::bind(&Planner::incomingTask, this, _1), false), 
  beacon_rate_(1), beacon_(beacon), mission_over_(false)
{
  nt_as_.start();

  //Load of known position and human targets known positions
  std::string path = ros::package::getPath("human_aware_collaboration_planner");
  ros::param::param<std::string>("~config_file", config_file, path + "/config/conf.yaml");

  readConfigFile(config_file);

  beacon_pub_ = nh_.advertise<human_aware_collaboration_planner::PlannerBeacon>("/planner_beacon", 1);
  beacon_sub_ = nh_.subscribe("/agent_beacon", 100, &Planner::beaconCallback, this);
  mission_over_sub_ = nh_.subscribe("/mission_over", 1, &Planner::missionOverCallback, this);
  ROS_INFO("[Planner] Initialization complete");

  beacon_rate_.reset();
  while(ros::ok() && !mission_over_)
  {
    checkBeaconsTimeout(ros::Time::now());
    beacon_.time = ros::Time::now();
    beacon_pub_.publish(beacon_);
    ros::spinOnce();
    beacon_rate_.sleep();
  }

  ROS_INFO("[Planner] Mission Over. Waiting for the agents to finish");

  while(!agent_map_.empty())
  {
    checkBeaconsTimeout(ros::Time::now());
    ros::spinOnce();
    beacon_rate_.sleep();
  }

  //Free memory
  for(auto &task: pending_tasks_)
    delete task.second;
  pending_tasks_.clear();
}
Planner::~Planner(void){}

void Planner::readConfigFile(std::string config_file){
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
          human_target.second['x'].as<float>(), human_target.second['y'].as<float>(), human_target.second['z'].as<float>());
    }
  }
  if(yaml_config["tools"])
  {
    for(auto const& tool : yaml_config["tools"])
    {
      tools_[tool.first.as<std::string>()] = classes::Tool(tool.first.as<std::string>(), tool.second["weight"].as<float>(),
          tool.second['x'].as<float>(), tool.second['y'].as<float>(), tool.second['z'].as<float>());
    }
  }
}

bool Planner::checkTaskParams(const human_aware_collaboration_planner::NewTaskGoalConstPtr& goal){
  std::map <std::string, classes::HumanTarget>::iterator human_itr;
  std::map <std::string, classes::Position>::iterator position_itr;
  std::map <std::string, classes::Tool>::iterator tool_itr;

  switch(goal->task.type)
  {
    case 'M':
    case 'm':
      human_itr = human_targets_.find(goal->task.monitor.human_target_id);
      if(human_itr == human_targets_.end())
      {
        ROS_INFO("[checkTaskParams] Invalid task: unknown human target ID");
        return false;
      }
      if(goal->task.monitor.distance == 0)
      {
        ROS_INFO("[checkTaskParams] Invalid task: Monitor Distance couldn't be 0");
        return false;
      }
      if(goal->task.monitor.number == 0)
      {
        ROS_INFO("[checkTaskParams] Invalid Task: Monitor Number couldn't be 0.");
        return false;
      }
      break;
    case 'I':
    case 'i':
      if(goal->task.inspect.waypoints.size() == 0)
      {
        ROS_INFO("[checkTaskParams] Invalid task: There has to be at least one waypoint");
        return false;
      }
      break;
    case 'A':
    case 'a':
      if(goal->task.inspect.waypoints.size() != 2)
      {
        ROS_INFO("[checkTaskParams] Invalid task: There has to be two waypoints");
        return false;
      }
      break;
    case 'D':
    case 'd':
      human_itr = human_targets_.find(goal->task.deliver.human_target_id);
      if(human_itr == human_targets_.end())
      {
        ROS_INFO("[checkTaskParams] Invalid task: unknown human target ID");
        return false;
      }
      tool_itr = tools_.find(goal->task.deliver.tool_id);
      if(tool_itr == tools_.end())
      {
        ROS_INFO("[checkTaskParams] Invalid task: unknown tool ID");
        return false;
      }
      break;
    default:
      return false;
      break;
  }
  return true;
}

//New Tasks callback
void Planner::incomingTask(const human_aware_collaboration_planner::NewTaskGoalConstPtr& goal){
  //Recharge tasks are not supposed to come from this way. Recharge tasks are created by the High-Level Planner
  //Wait tasks are not supposed to come from this way. Wait tasks are created by the High-Level Planner
  nt_feedback_.status = "Reading the New Task";
  nt_as_.publishFeedback(nt_feedback_);

  std::string id = goal->task.id;
  char type = goal->task.type;
  char old_type;

  std::string m_human_target_id;
  float distance;
  int number;
  std::string d_tool_id;
  std::string d_human_target_id;

  //Check if the task params are correct
  if(!checkTaskParams(goal))
  {
    ROS_WARN("[incomingTask] Incorrect task");
    //If the task already exists, delete it from memory and reallocate tasks
    std::map<std::string, classes::Task*>::iterator task_itr = pending_tasks_.find(id);
    if(task_itr != pending_tasks_.end())
    {
      old_type = task_itr->second->getType();
      //Warn Operators that a task has been deleted due to repeated IDs.
      ROS_INFO_STREAM("[incomingTask] " << id << "(" << (
              old_type == 'M' ? "Monitor" : 
              old_type == 'I' ? "Inspect" :
              old_type == 'A' ? "InspectiPVArray" :
              old_type == 'D' ? "DeliverTool" : 
              old_type == 'R' ? "Recharge" : 
              old_type == 'W' ? "Wait" : 
              "Task") << ") is going to be deleted");
      deletePendingTask(id);

      ROS_INFO("[incomingTask] Allocating tasks...");
      performTaskAllocation();

      nt_result_.ack = false;
      nt_as_.setSucceeded(nt_result_);
      return;
    }
    nt_result_.ack = false;
    nt_as_.setSucceeded(nt_result_);
    return;
  }

  nt_feedback_.status = "Checking if the New Task already exists";
  nt_as_.publishFeedback(nt_feedback_);

  //Check if the new task already exist
  std::map<std::string, classes::Task*>::iterator task_itr = pending_tasks_.find(id);
  if(task_itr != pending_tasks_.end())
  {
    old_type = task_itr->second->getType();
    //Check if new task's type is the same as old one's type.
    if(type == old_type)
    {
      //Update params
      if(!updateTaskParams(goal))
      {
        nt_result_.ack = false;
        nt_as_.setAborted(nt_result_);
        return;
      }
      else
      {
        ROS_INFO("[incomingTask] Task Params Updated. Allocating tasks...");

        nt_feedback_.status = "Allocating pending tasks";
        nt_as_.publishFeedback(nt_feedback_);

        performTaskAllocation();

        nt_result_.ack = true;
        nt_as_.setSucceeded(nt_result_);
        return;
      }
    }
    else
    {
      //Warn Operators that a task has been deleted due to repeated IDs.
      ROS_WARN_STREAM("[incomingTask] Duplicated ID. An unfinished task is going to be deleted: " << id << "(" << (
              old_type == 'M' ? "Monitor" : 
              old_type == 'I' ? "Inspect" : 
              old_type == 'A' ? "InspectPVArray" : 
              old_type == 'D' ? "DeliverTool" : 
              old_type == 'R' ? "Recharge" : 
              old_type == 'W' ? "Wait" : 
              "Task") << ")");
      //Delete old pending task and change the task in Agents tasks queue by a classes::Task (for cost calculation)
      deletePendingTask(id);
    }
  }

  nt_feedback_.status = "Adding the New Task to pending_tasks_ map";
  nt_as_.publishFeedback(nt_feedback_);

  switch(goal->task.type)
  {
    case 'M':
    case 'm':
      pending_tasks_[id] = new classes::Monitor(id, &(human_targets_[goal->task.monitor.human_target_id]),
          goal->task.monitor.distance, goal->task.monitor.number);
      monitor_tasks_.push_back(id);
      break;
    case 'I':
    case 'i':
      pending_tasks_[id] = new classes::Inspect(id, goal->task.inspect.waypoints);//Memory Leak
      inspect_tasks_.push_back(id);
      break;
    case 'A':
    case 'a':
      pending_tasks_[id] = new classes::InspectPVArray(id, goal->task.inspect.waypoints);//Memory Leak
      inspect_tasks_.push_back(id);
      break;
    case 'D':
    case 'd':
      pending_tasks_[id] = new classes::DeliverTool(id, &(tools_[goal->task.deliver.tool_id]),
          &(human_targets_[goal->task.deliver.human_target_id]));
      deliver_tasks_.push_back(id);
      break;
    default:
      break;
  }
  ROS_INFO_STREAM("[incomingTask] Received a New Task:\n" << *pending_tasks_[id]);
  ROS_INFO("[incomingTask] Allocating tasks...");

  nt_feedback_.status = "Allocating pending tasks";
  nt_as_.publishFeedback(nt_feedback_);

  performTaskAllocation();

  if (nt_as_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("[incomingTask] Incommig Task: Preempted");
    nt_as_.setPreempted();
  }

  nt_result_.ack = true;
  nt_as_.setSucceeded(nt_result_);

  return;
}

//Agent Beacon Handler
void Planner::beaconCallback(const human_aware_collaboration_planner::AgentBeacon::ConstPtr& beacon){
  std::map <std::string, Agent>::iterator agent_itr = agent_map_.find(beacon->id);
  if(agent_itr == agent_map_.end())
  {
    ROS_INFO("[beaconCallback] New Agent connected: %s", beacon->id.c_str());
    Agent agent(this, beacon->id, beacon->type, ros::Time::now(), *beacon);
    agent_map_.emplace(beacon->id, agent);
    if(beacon->type == "PhysicalACW")
      deliver_agents_.push_back(beacon->id);
    else if(beacon->type == "InspectionACW")
      inspect_agents_.push_back(beacon->id);
    else if(beacon->type == "SafetyACW")
      monitor_agents_.push_back(beacon->id);
    performTaskAllocation();
  }
  else
  {
    //Reiniciar timeout del heartbeat
    agent_map_[beacon->id].setLastBeaconTime(ros::Time::now());

    if(!beacon->timeout && agent_map_[beacon->id].getLastBeaconTimeout())
    {
      ROS_WARN_STREAM("[beaconCallback] (" << beacon->id << ") Disconnected briefly, activated the emergency " 
          << "protocol by emptying its task queue and reconnected without Planner noticing");
      agent_map_[beacon->id].sendQueueToAgent();
    }

    agent_map_[beacon->id].setLastBeacon(*beacon);
  }
}

void Planner::missionOverCallback(const human_aware_collaboration_planner::MissionOver& value){mission_over_ = value.value;}

//Method to reasign all not finished tasks
void Planner::performTaskAllocation(){
  //TODO: This is a simplified version. Random heuristic methods should be included.
  //TODO: Recharge tasks should be included too in the plans. Need something to sinchonise all plans.
  /* Task priority order will be inherent to task type and therefore, it will be hardcoded.
   * As I haven't read anywhere the priority order specifications:
   *    Deliver tasks will have the highest priority.
   *    Inspection tasks will be in second place.
   *    Monitor tasks will be the least priority.
   * 
   * Also, Agent type will be considered in order to asign tasks. Priority will be more important
   * than Agent type but if an Agent can't perform a task due to it's type, that task will wait.
   *
   * Monitor tasks will never end, once they are send, only the end of the mission, another
   * task with the same task ID, the retrear of the Human Target or a specific order to stop
   * monitoring will stop this task. Also, a Recharge task could postpone this task.
   */

  std::multiset<Cost, std::less<Cost>> cost;
  std::vector <std::string> idle_deliver_agents;
  std::vector <std::string> idle_inspect_agents;
  std::vector <std::string> idle_monitor_agents;
  std::vector <human_aware_collaboration_planner::Waypoint> inspect_waypoints;
  std::vector <std::string> agent_list;
  std::vector <std::string>::iterator agent_list_itr;
  std::vector <human_aware_collaboration_planner::Waypoint> aux_waypoints;
  std::map <std::string, std::vector <human_aware_collaboration_planner::Waypoint>> divided_waypoints;
  std::vector <std::string> deliver_tasks_copy;
  std::vector <std::string> inspect_tasks_copy;
  std::vector <std::string> monitor_tasks_copy;
  int number_of_waypoints;
  int number_of_idle_agents;
  int agents_to_select;

  //Task allocation algorithm. 
  /*
   * While allocating, check the level of the battery and create a Recharge Task when needed depending on the predicted
   * consume of battery. A "new" will be needed to allocate temporally this task. As this task only matter inside the
   * task allocation function, remember to do "delete" before "return" after sendind the tasks queues.
   */
  if(!agent_map_.empty())
  {
    //Loop throught map, set auxiliary queue and empty agents' queue
    for(auto &agent: agent_map_)
    {
      agent.second.setOldTaskQueue();
      agent.second.emptyTheQueue();
    }

    //First. Allocate deliver task in order of arrival
    if(!deliver_tasks_.empty())
    {
      //Only DeliveryACW can perform this task
      if(!deliver_agents_.empty())
      {
        deliver_tasks_copy = deliver_tasks_;
        for(auto &task: deliver_tasks_copy)
        {
          cost.clear();
          //Take the first deliver task and compute the cost from each Agent
          for(auto &agent: deliver_agents_)
          {
            if(agent_map_[agent].isBatteryEnough(pending_tasks_[task]))
              cost.emplace(agent_map_[agent].computeTaskCost(pending_tasks_[task]), agent);
          }
          //Assign the task to the Agent that costs the least
          if(cost.empty())
            continue;
          else
            agent_map_[cost.begin()->id_].addTaskToQueue(pending_tasks_[task]);
        }
      }
    }

    //Second. Allocate inspect task in order of arrival
    if(!inspect_tasks_.empty())
    {
      inspect_tasks_copy = inspect_tasks_;
      for(auto &task: inspect_tasks_copy)
      {
        inspect_waypoints = pending_tasks_[task]->getInspectWaypoints();
        agent_list.clear();
        idle_inspect_agents.clear();
        divided_waypoints.clear();
        cost.clear();
        //Take the first inspect task and compute the cost from each Agent
        for(auto &agent: agent_map_)
        {
          if(agent.second.isBatteryEnough(pending_tasks_[task]))
          {
            if(agent.second.isQueueEmpty())
              idle_inspect_agents.push_back(agent.first);
            cost.emplace(agent_map_[agent.first].computeTaskCost(pending_tasks_[task]), agent.first);
          }
        }
        /* The task should be assigned to N agents decided by task allocator
         *    Check how many waypoints are in the inspection list:
         *      <= 3 waypoints: 1 agent
         *      <= 6 waypoints: 2 agents
         *      >  6 waypoints: 3 agents
         *    Check how many inspect agents has an empty queue (means that will run this task first, at the same time)
         *
         * The máximun number of selected agents will be min between:
         *    Idle agents from those who have battery enough
         *    Agents in function of number of waypoints
         * Need to select the least cost agents from idle ones.
         */

        number_of_waypoints = inspect_waypoints.size();
        if(number_of_waypoints <= 3)
          agents_to_select = 1;
        else if(number_of_waypoints <= 6)
          agents_to_select = 2;
        else
          agents_to_select = 3;
        if(pending_tasks_[task]->getType() == 'A')
          agents_to_select = 1;

        number_of_idle_agents = idle_inspect_agents.size();
        if(number_of_idle_agents < agents_to_select)
          agents_to_select = number_of_idle_agents;
 
        //Fill the "agent_list" task param
        if(cost.empty())
          continue;
        else if(agents_to_select == 0)
        {
          agent_list.push_back(cost.begin()->id_);
          pending_tasks_[task]->setAgentList(agent_list);//Memory Leak
          agent_map_[cost.begin()->id_].addTaskToQueue(pending_tasks_[task]);
          continue;
        }
        else
        {
          for(const auto& c: cost)
          {
            if(std::find(idle_inspect_agents.begin(), idle_inspect_agents.end(), c.id_) != idle_inspect_agents.end())
            {
              agent_list.push_back(c.id_);
              --agents_to_select;
              if(agents_to_select == 0)
                break;
            }
          }
        }
        pending_tasks_[task]->setAgentList(agent_list);//Memory Leak
        agents_to_select = agent_list.size();
        //Divide waypoints into "agents_to_select" groups and send tasks to selected agents' queue
        /*
         * At the moment, the waypoints will be divided acording to the vector order, in the next version, this
         * distribution will be done by imlementing a k-mean algorith to divide de waypoints list into minimun distance
         * groups of wayopoints. Will be "agents_to_select"-means.
         */
        agent_list_itr = agent_list.begin();
        for(auto& wp: inspect_waypoints)
        {
          if(divided_waypoints.find(*agent_list_itr) != divided_waypoints.end())
            divided_waypoints[*agent_list_itr].push_back(wp);
          else
          {
            aux_waypoints.clear();
            aux_waypoints.push_back(wp);
            divided_waypoints[*agent_list_itr] = aux_waypoints;
          }

          if(++agent_list_itr == agent_list.end())
            agent_list_itr = agent_list.begin();
        }


        for(const auto& agent: agent_list)
        {
          pending_tasks_[task]->setWaypoints(divided_waypoints[agent]);
          agent_map_[agent].addTaskToQueue(pending_tasks_[task]);
        }
      }
    }

    //Third. Allocate monitor task in order of arrival
    if(!monitor_tasks_.empty())
    {
      monitor_tasks_copy = monitor_tasks_;
      for(auto &task: monitor_tasks_copy)
      {
        agent_list.clear();
        idle_monitor_agents.clear();
        cost.clear();
        //Take the first monitor task and compute the cost from each Agent
        for(auto &agent: agent_map_)
        {
          if(agent.second.isBatteryEnough(pending_tasks_[task]))
            cost.emplace(agent_map_[agent.first].computeTaskCost(pending_tasks_[task]), agent.first);
        }
        //Assign the task to the Agent that costs the least
        agents_to_select = pending_tasks_[task]->getNumber();
        //If agent_to_select > connected agents or agents that have battery: select only disponible agents
        for(const auto& c: cost)
        {
          if(agents_to_select == 0)
            break;
          agent_list.push_back(c.id_);
          --agents_to_select;
        }
        pending_tasks_[task]->setAgentList(agent_list);//Memory Leak
        for(const auto& agent: agent_list)
        {
          agent_map_[agent].addTaskToQueue(pending_tasks_[task]);
        }
      }
    }

    ROS_INFO("[performTaskAllocation] Tasks Allocated:");
    for(auto &agent: agent_map_)
      ROS_INFO_STREAM("[performTaskAllocation] " << agent.second);
    
    //Loop throught map and send new queues to agents
    for(auto &agent: agent_map_)
      agent.second.sendQueueToAgent();

    //Delete no Typed Tasks from agents old queues for good use of memory
    for(auto &agent: agent_map_)
      agent.second.deleteOldTaskQueue();

    //Delete Recharge Tasks from memory (delete)
    //TODO: free recharge tasks memory, no need to keep a copy in memory outside this function.

  }
  else
    ROS_WARN("[performTaskAllocation] No Agents connected yet. %lu pending tasks", pending_tasks_.size());

  return;
}

classes::Task* Planner::getPendingTask(std::string task_id){
  std::map<std::string, classes::Task*>::iterator task_itr = pending_tasks_.find(task_id);
  if(task_itr != pending_tasks_.end())
    return task_itr->second;
  else
    return nullptr;
}

void Planner::deletePendingTask(std::string task_id){
  //Check if the task exists
  std::map<std::string, classes::Task*>::iterator task_itr = pending_tasks_.find(task_id);
  if(task_itr != pending_tasks_.end())
  {
    char type = task_itr->second->getType();
    //Delete task from type_task_queue
    switch(type)
    {
      case 'M':
      case 'm':
        monitor_tasks_.erase(std::find(monitor_tasks_.begin(), monitor_tasks_.end(), task_itr->second->getID()));
        break;
      case 'I':
      case 'i':
        inspect_tasks_.erase(std::find(inspect_tasks_.begin(), inspect_tasks_.end(), task_itr->second->getID()));
        break;
      case 'A':
      case 'a':
        inspect_tasks_.erase(std::find(inspect_tasks_.begin(), inspect_tasks_.end(), task_itr->second->getID()));
        break;
      case 'D':
      case 'd':
        deliver_tasks_.erase(std::find(deliver_tasks_.begin(), deliver_tasks_.end(), task_itr->second->getID()));
        break;
      default:
        break;
    }
    //Delete task from agents_queue (becouse old_task_queue will use the pointer)
    if(!agent_map_.empty())
      for(auto &agent: agent_map_)
        agent.second.replaceTaskFromQueue(task_id);
    //Delete task pointer
    delete task_itr->second;
    pending_tasks_.erase(task_itr);
  }
  return;
}

bool Planner::updateTaskParams(const human_aware_collaboration_planner::NewTaskGoalConstPtr& goal){
  classes::Task* aux;

  std::string id = goal->task.id;
  char type = goal->task.type;

  std::string m_human_target_id;
  float distance;
  int number;
  std::string d_tool_id;
  std::string d_human_target_id;

  std::map <std::string, classes::HumanTarget>::iterator human_itr;
  std::map <std::string, classes::Position>::iterator position_itr;
  std::map <std::string, classes::Tool>::iterator tool_itr;

  //Check if the task exists
  std::map<std::string, classes::Task*>::iterator task_itr = pending_tasks_.find(id);
  if(task_itr != pending_tasks_.end())
  {
    switch(type)
    {
      case 'M':
      case 'm':
        m_human_target_id = goal->task.monitor.human_target_id;
        distance = goal->task.monitor.distance;
        number = goal->task.monitor.number;

        human_itr = human_targets_.find(m_human_target_id);
        if(human_itr == human_targets_.end())
        {
          ROS_INFO("[updateTaskParams] Invalid task: unknown human target ID");
          return false;
        }
        aux = new classes::Monitor(id, &(human_targets_[m_human_target_id]), distance, number);
        task_itr->second->updateParams(aux);
        break;
      case 'I':
      case 'i':
        aux = new classes::Inspect(id, goal->task.inspect.waypoints);//Memory Leak
        task_itr->second->updateParams(aux);//Memory Leak
        break;
      case 'A':
      case 'a':
        aux = new classes::InspectPVArray(id, goal->task.inspect.waypoints);//Memory Leak
        task_itr->second->updateParams(aux);//Memory Leak
        break;
      case 'D':
      case 'd':
        d_tool_id = goal->task.deliver.tool_id;
        d_human_target_id = goal->task.deliver.human_target_id;

        human_itr = human_targets_.find(d_human_target_id);
        if(human_itr == human_targets_.end())
        {
          ROS_INFO("[updateTaskParams] Invalid task: unknown human target ID");
          return false;
        }
        tool_itr = tools_.find(d_tool_id);
        if(tool_itr == tools_.end())
        {
          ROS_INFO("[updateTaskParams] Invalid task: unknown tool ID");
          return false;
        }
        aux = new classes::DeliverTool(id, &(tools_[d_tool_id]), &(human_targets_[d_human_target_id]));
        task_itr->second->updateParams(aux);
        break;
      default:
        break;
    }
  }
  delete aux;

  return true;
}

void Planner::checkBeaconsTimeout(ros::Time now){
  //Delete disconnected Agent from agent_map_ and from the corresponding type_agents_ vector, and send them an empty
  //task just in case they still listen to this block
  std::queue <std::string> disconnected_agents;
  auto prev_size = agent_map_.size();
  std::string type;
  std::string id;

  if(!agent_map_.empty())
  {
    for(auto &agent: agent_map_)
      if(agent.second.checkBeaconTimeout(now))
        disconnected_agents.push(agent.first);

    while(!disconnected_agents.empty())
    {
      id = disconnected_agents.front();
      ROS_WARN_STREAM("[checkBeaconsTimeout] Beacon Timeout. " << id << " disconnected.");

      //Erase disconnected UAV from agent type corresponding list
      type = agent_map_[id].getType();
      if(type == "PhysicalACW")
        deliver_agents_.erase(std::find(deliver_agents_.begin(), deliver_agents_.end(), id));
      else if(type == "InspectionACW")
        inspect_agents_.erase(std::find(inspect_agents_.begin(), inspect_agents_.end(), id));
      else if(type == "SafetyACW")
        monitor_agents_.erase(std::find(monitor_agents_.begin(), monitor_agents_.end(), id));

      //Send to the disconnected UAV an empty queue just in case
      ROS_WARN_STREAM("[checkBeaconsTimeout] Sending " << id << " an empty queue just in case");
      agent_map_[id].emptyTheQueue();
      agent_map_[id].sendQueueToAgent();

      //Erase it from memory
      agent_map_.erase(id);
      disconnected_agents.pop();
    }

    if(agent_map_.size() != prev_size)
    {
      ROS_INFO_STREAM("[checkBeaconsTimeout] Connected Agents: " << agent_map_.size() <<". Perform Task Allocation:");
      performTaskAllocation();
    }
  }

  return;
}

//Getters
bool Planner::getMissionOver(){return mission_over_;}

int main(int argc, char **argv){
  ros::init(argc, argv, "high_level_planner");

  human_aware_collaboration_planner::PlannerBeacon beacon;
  Planner planner(beacon);
  ROS_INFO("[main] Ending...");

  return 0;
}

