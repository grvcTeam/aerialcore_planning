#include "human_aware_collaboration_planner/classes.h"

using namespace classes;

//class Position
Position::Position() : id_(""), x_(0), y_(0), z_(0) {}
Position::Position(float x, float y, float z) : id_(""), x_(x), y_(y), z_(z) {}
Position::Position(std::string id, float x, float y, float z) : id_(id), x_(x), y_(y), z_(z) {}
Position::Position(const Position& p) : id_(p.id_), x_(p.x_), y_(p.y_), z_(p.z_) {}
Position::~Position(){}
void Position::update(float x, float y, float z){
  x_ = x;
  y_ = y;
  z_ = z;
}
//class Position Getters
std::string Position::getID(){return id_;}
float Position::getX(){return x_;}
float Position::getY(){return y_;}
float Position::getZ(){return z_;}
void Position::print(std::ostream& os) const{
  os << "(" << x_ << ", " << y_ << ", " << z_ << ")";
  return;
}

float classes::distance(Position& p1, Position& p2){
  return sqrt(pow(p1.x_ - p2.x_, 2) + pow(p1.y_ - p2.y_, 2) + pow(p1.z_ - p2.z_, 2));
}

float classes::distance(Position& p1, human_aware_collaboration_planner::Waypoint& p2){
  return sqrt(pow(p1.x_ - p2.x, 2) + pow(p1.y_ - p2.y, 2) + pow(p1.z_ - p2.z, 2));
}

float classes::distance(human_aware_collaboration_planner::Waypoint& p1, Position& p2){
  return sqrt(pow(p1.x - p2.x_, 2) + pow(p1.y - p2.y_, 2) + pow(p1.z - p2.z_, 2));
}

float classes::distance(human_aware_collaboration_planner::Waypoint& p1, human_aware_collaboration_planner::Waypoint& p2){
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

float classes::distance2D(Position& p1, Position& p2){
  return sqrt(pow(p1.x_ - p2.x_, 2) + pow(p1.y_ - p2.y_, 2));
}

float classes::distance2D(Position& p1, human_aware_collaboration_planner::Waypoint& p2){
  return sqrt(pow(p1.x_ - p2.x, 2) + pow(p1.y_ - p2.y, 2));
}

float classes::distance2D(human_aware_collaboration_planner::Waypoint& p1, Position& p2){
  return sqrt(pow(p1.x - p2.x_, 2) + pow(p1.y - p2.y_, 2));
}

float classes::distance2D(human_aware_collaboration_planner::Waypoint& p1, human_aware_collaboration_planner::Waypoint& p2){
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

Position classes::close_pose(Position& orig, Position& dest, float dist){
  float mod = sqrt(pow(orig.x_ - dest.x_, 2) + pow(orig.y_ - dest.y_, 2) + pow(orig.z_ - dest.z_, 2));
  float x = dest.x_ + dist*(orig.x_ - dest.x_)/mod;
  float y = dest.y_ + dist*(orig.y_ - dest.y_)/mod;
  float z = dest.z_ + dist*(orig.z_ - dest.z_)/mod;

  return Position(x, y, z);
}

Position classes::close_pose(Position& orig, human_aware_collaboration_planner::Waypoint& dest, float dist){
  float mod = sqrt(pow(orig.x_ - dest.x, 2) + pow(orig.y_ - dest.y, 2) + pow(orig.z_ - dest.z, 2));
  float x = dest.x + dist*(orig.x_ - dest.x)/mod;
  float y = dest.y + dist*(orig.y_ - dest.y)/mod;
  float z = dest.z + dist*(orig.z_ - dest.z)/mod;

  return Position(x, y, z);
}

Position classes::close_pose_2D(Position& orig, Position& dest, float dist){
  float mod = sqrt(pow(orig.x_ - dest.x_, 2) + pow(orig.y_ - dest.y_, 2));
  float x = dest.x_ + dist*(orig.x_ - dest.x_)/mod;
  float y = dest.y_ + dist*(orig.y_ - dest.y_)/mod;
  float z = dest.z_;

  return Position(x, y, z);
}

Position classes::close_pose_2D(Position& orig, human_aware_collaboration_planner::Waypoint& dest, float dist){
  float mod = sqrt(pow(orig.x_ - dest.x, 2) + pow(orig.y_ - dest.y, 2));
  float x = dest.x + dist*(orig.x_ - dest.x)/mod;
  float y = dest.y + dist*(orig.y_ - dest.y)/mod;
  float z = dest.z;

  return Position(x, y, z);
}

human_aware_collaboration_planner::Waypoint classes::central_position(std::vector<human_aware_collaboration_planner::Waypoint>& waypoints){
  human_aware_collaboration_planner::Waypoint central_position;

  central_position.x = 0;
  central_position.y = 0;
  central_position.z = 0;
  
  for(auto &waypoint: waypoints)
  {
    central_position.x += waypoint.x;
    central_position.y += waypoint.y;
    central_position.z += waypoint.z;
  }
  central_position.x /= waypoints.size();
  central_position.y /= waypoints.size();
  central_position.z /= waypoints.size();

  return central_position;
}

//class HumanTarget
HumanTarget::HumanTarget() : position_(), id_("") {}
HumanTarget::HumanTarget(std::string id, std::string position_id, float x, float y, float z) : 
  position_(position_id, x, y, z), id_(id){}
HumanTarget::HumanTarget(std::string id, float x, float y, float z) : position_(x, y, z), id_(id){}
HumanTarget::HumanTarget(std::string id, const Position& position) : position_(position), id_(id){}
HumanTarget::HumanTarget(const HumanTarget& h) : 
  position_(h.position_.id_, h.position_.x_, h.position_.y_, h.position_.z_), id_(h.id_) {}
HumanTarget::~HumanTarget(){}
//class HumanTarget Getters
std::string HumanTarget::getID(){return id_;}
Position HumanTarget::getPosition(){return position_;}
//void HumanTarget::updatePosition(){}

//class Tool
Tool::Tool() : position_(), id_(""), weight_(0.0) {}
Tool::Tool(std::string id, float weight, std::string position_id, float x, float y, float z) : 
  position_(position_id, x, y, z), id_(id), weight_(weight) {}
Tool::Tool(std::string id, float weight, float x, float y, float z) : 
  position_(x, y, z), id_(id), weight_(weight) {}
Tool::Tool(std::string id, float weight, const Position& position) : 
  position_(position), id_(id), weight_(weight) {}
Tool::Tool(const Tool& t) : 
  position_(t.position_.id_, t.position_.x_, t.position_.y_, t.position_.z_), id_(t.id_), weight_(t.weight_) {}
Tool::~Tool(){}
//class Tool Getters
std::string Tool::getID(){return id_;}
float Tool::getWeight(){return weight_;}
Position Tool::getPosition(){return position_;}
//void Tool::updatePosition(){}

//class Task
Task::Task() : id_("") {}
Task::Task(std::string id) : id_(id) {}
Task::Task(const Task& t) : id_(t.id_) {}
Task::~Task(){}
//class Task Getters
std::string Task::getID(){return id_;}
char Task::getType(){return 'T';}
const HumanTarget* Task::getHumanPtr(){return nullptr;}
std::string Task::getHumanID(){return "";}
Position Task::getHumanPosition(){return Position();}
float Task::getDistance(){return 0;}
int Task::getNumber(){return 0;}
std::vector<std::string> Task::getAgentList(){return std::vector<std::string>();}
std::vector<human_aware_collaboration_planner::Waypoint> Task::getInspectWaypoints(){
  return std::vector<human_aware_collaboration_planner::Waypoint>();
}
Tool Task::getTool(){return Tool();}
std::string Task::getToolID(){return "";}
const Tool* Task::getToolPtr(){return nullptr;}
Position Task::getToolPosition(){return Position();}
Position* Task::getChargingStationPtr(){return nullptr;}
Position Task::getChargingStation(){return Position();}
std::string Task::getChargingStationID(){return "";}
float Task::getInitialPercentage(){return 0;}
float Task::getFinalPercentage(){return 0;}
void Task::print(std::ostream& os) const{
  os << "\t" << id_ << ": Task";
  return;
}
//Class Task Setters
void Task::setWaypoints(std::vector<human_aware_collaboration_planner::Waypoint> waypoints){return;}
void Task::setAgentList(std::vector<std::string> agent_list){return;}
void Task::setChargingStation(Position* charging_station){return;}
void Task::setInitialPercentage(float initial_percentage){return;}
void Task::setFinalPercentage(float final_percentage){return;}
void Task::updateParams(classes::Task* task){return;}

//class Monitor : public Task
Monitor::Monitor() : Task(), human_target_(nullptr), distance_(0), number_(0) {}
Monitor::Monitor(std::string task_id, const HumanTarget* human_target, float distance, int number) : Task(task_id), 
  human_target_(human_target), distance_(distance), number_(number) {}
Monitor::Monitor(std::string task_id, const HumanTarget* human_target, float distance, int number, 
  std::vector<std::string> agent_list) : Task(task_id), human_target_(human_target), distance_(distance), 
  number_(number), agent_list_(agent_list) {}
Monitor::Monitor(const Monitor& m) : Task(m.id_), human_target_(m.human_target_), distance_(m.distance_), 
  number_(m.number_), agent_list_(m.agent_list_) {}
Monitor::~Monitor(){}
//class monitor Getters
std::string Monitor::getID(){return id_;}
char Monitor::getType(){return 'M';}
const HumanTarget* Monitor::getHumanPtr(){return human_target_;}
std::string Monitor::getHumanID(){return const_cast <HumanTarget*> (human_target_)->getID();}
Position Monitor::getHumanPosition(){return const_cast <HumanTarget*> (human_target_)->getPosition();}
float Monitor::getDistance(){return distance_;}
int Monitor::getNumber(){return number_;}
std::vector<std::string> Monitor::getAgentList(){return agent_list_;}
void Monitor::print(std::ostream& os) const{
  os << "\t" << id_ << ": Monitor (" << distance_ << ", " << number_ << ")\n\t\tHuman Target: " << 
    human_target_->id_ << ": " << human_target_->position_.x_ << ", " << human_target_->position_.y_ << ", " << 
    human_target_->position_.z_ << "\n\t\tAgent List:";
  for(auto a = agent_list_.begin(); a != agent_list_.end(); ++a)
    os << " (" << *a << ")";
  return;
}
//Class Monitor Setters
void Monitor::setAgentList(std::vector<std::string> agent_list){
  agent_list_ = agent_list;
  return;
}
void Monitor::updateParams(classes::Task* task){
  if(task->getType() == 'M'){
    human_target_ = task->getHumanPtr();
    distance_ = task->getDistance();
    number_ = task->getNumber();
  }
  return;
}

//class Inspect : public Task
Inspect::Inspect() : Task() {}
Inspect::Inspect(std::string task_id, std::vector<human_aware_collaboration_planner::Waypoint> waypoints) : 
  Task(task_id), waypoints_(waypoints) {}
Inspect::Inspect(std::string task_id, std::vector<human_aware_collaboration_planner::Waypoint> waypoints, 
    std::vector<std::string> agent_list) : Task(task_id), waypoints_(waypoints), agent_list_(agent_list) {}
Inspect::Inspect(const Inspect& i) : Task(i.id_), waypoints_(i.waypoints_), agent_list_(i.agent_list_) {}
Inspect::~Inspect(){}
//class Inspect Getters
std::string Inspect::getID(){return id_;}
char Inspect::getType(){return 'I';}
std::vector<human_aware_collaboration_planner::Waypoint> Inspect::getInspectWaypoints(){return waypoints_;}
std::vector<std::string> Inspect::getAgentList(){return agent_list_;}
void Inspect::print(std::ostream& os) const{
  os << "\t" << id_ << ": Inspect\n\t\tPositions:";
  for(auto wp = waypoints_.begin(); wp != waypoints_.end(); ++wp)
    os << " (" << wp->x << ", " << wp->y << ", " << wp->z << ")";
  os << "\n\t\tAgent List:";
  for(auto a = agent_list_.begin(); a != agent_list_.end(); ++a)
    os << " (" << *a << ")";
  return;
}
//Class Inspect Setters
void Inspect::setWaypoints(std::vector<human_aware_collaboration_planner::Waypoint> waypoints){
  waypoints_ = waypoints;
  return;
}
void Inspect::setAgentList(std::vector<std::string> agent_list){
  agent_list_ = agent_list;
  return;
}
void Inspect::updateParams(classes::Task* task){
  if(task->getType() == 'I')
    waypoints_ = task->getInspectWaypoints();
  return;
}

//class DeliverTool : public Task
DeliverTool::DeliverTool() : Task(), tool_(nullptr), human_target_(nullptr) {}
DeliverTool::DeliverTool(std::string task_id, const Tool* tool, const HumanTarget* human_target) : 
  Task(task_id), tool_(tool), human_target_(human_target) {}
DeliverTool::DeliverTool(const DeliverTool& d) : Task(d.id_), tool_(d.tool_), human_target_(d.human_target_) {}
DeliverTool::~DeliverTool(){}
//class DeliverTool Getters
std::string DeliverTool::getID(){return id_;}
char DeliverTool::getType(){return 'D';}
Tool DeliverTool::getTool(){return *tool_;}
std::string DeliverTool::getToolID(){return const_cast <Tool*> (tool_)->getID();}
const Tool* DeliverTool::getToolPtr(){return tool_;}
Position DeliverTool::getToolPosition(){return const_cast <Tool*> (tool_)->getPosition();}
const HumanTarget* DeliverTool::getHumanPtr(){return human_target_;}
std::string DeliverTool::getHumanID(){return const_cast <HumanTarget*> (human_target_)->getID();}
Position DeliverTool::getHumanPosition(){return const_cast <HumanTarget*> (human_target_)->getPosition();}
void DeliverTool::print(std::ostream& os) const{
  os << "\t" << id_ << ": Deliver\n\t\tTool: " << tool_->id_ << " (" << tool_->weight_ << "kg): " <<
    tool_->position_.x_ << ", " << tool_->position_.y_ << ", " << tool_->position_.z_ << "\n\t\tHuman Target: " << 
    human_target_->id_ << ": " << human_target_->position_.x_ << ", " << human_target_->position_.y_ << ", " << 
    human_target_->position_.z_;
  return;
}
//class DeliverTool Setters
void DeliverTool::updateParams(classes::Task* task){
  if(task->getType() == 'D'){
    tool_ = task->getToolPtr();
    human_target_ = task->getHumanPtr();
  }
  return;
}

//class Recharge : public Task
Recharge::Recharge() : Task(), charging_station_(), initial_percentage_(), final_percentage_() {}
Recharge::Recharge(std::string task_id, float initial_percentage, float final_percentage) : Task(task_id), 
  charging_station_(), initial_percentage_(initial_percentage), final_percentage_(final_percentage) {}
Recharge::Recharge(std::string task_id, Position* charging_station, float initial_percentage, float
    final_percentage) : Task(task_id), charging_station_(charging_station), initial_percentage_(initial_percentage),
  final_percentage_(final_percentage) {}
Recharge::Recharge(const Recharge& r) : Task(r.id_), charging_station_(r.charging_station_), 
  initial_percentage_(r.initial_percentage_), final_percentage_(r.final_percentage_) {}
Recharge::~Recharge(){}
//Recharge Getters
std::string Recharge::getID(){return id_;}
char Recharge::getType(){return 'R';}
Position* Recharge::getChargingStationPtr(){return charging_station_;}
Position Recharge::getChargingStation(){return (*charging_station_);}
std::string Recharge::getChargingStationID(){return const_cast <Position*> (charging_station_)->getID();}
float Recharge::getInitialPercentage(){return initial_percentage_;}
float Recharge::getFinalPercentage(){return final_percentage_;}
void Recharge::print(std::ostream& os) const{
  os << "\t" << id_ << ": Recharge\n\t\tCharging Station: " << charging_station_ << "\n\t\tInitial percentage: " << 
    initial_percentage_ << "\n\t\tFinal percentage: " << final_percentage_;
  return;
}
//Recharge Setters
void Recharge::setChargingStation(Position* charging_station){
  charging_station_ = charging_station;
  return;
}
void Recharge::setInitialPercentage(float initial_percentage){
  initial_percentage_ = initial_percentage;
  return;
}
void Recharge::setFinalPercentage(float final_percentage){
  final_percentage_ = final_percentage;
  return;
}
void Recharge::updateParams(classes::Task* task){
  if(task->getType() == 'R'){
    charging_station_ = task->getChargingStationPtr();
    initial_percentage_ = task->getInitialPercentage();
    final_percentage_ = task->getFinalPercentage();
  }
  return;
}

//class Wait : public Task
Wait::Wait(){}
Wait::Wait(const Wait& w) : Task(w.id_){}
Wait::~Wait(){}
//Wait Getters
std::string Wait::getID(){return id_;}
char Wait::getType(){return 'W';}
void Wait::print(std::ostream& os) const{
  os << "\t" << id_ << ": Wait";
  return;
}
//Wait Setters
