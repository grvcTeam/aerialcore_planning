#ifndef CLASSES_H
#define CLASSES_H

#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include <cmath>
#include <vector>
#include <queue>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "human_aware_collaboration_planner/Waypoint.h"

namespace classes {
//Forward declarations
class BT;
class Position;
class HumanTarget;
class Tool;
class Task;
class Monitor;
class Inspect;
class DeliverTool;
class Recharge;
class Agent;

class Position {
  friend class HumanTarget;
  friend class Tool;
  friend class Monitor;
  friend class Inspect;
  friend class DeliverTool;
	friend class Recharge;
  
  protected:
    std::string id_;
    float x_, y_, z_;
  public:
    Position();
    Position(float x, float y, float z);
    Position(std::string id, float x, float y, float z);
    Position(const Position& p);
		~Position();
    void update(float x, float y, float z);
		friend float distance(Position&, Position&);
		friend float distance(Position&, human_aware_collaboration_planner::Waypoint&);
		friend float distance(human_aware_collaboration_planner::Waypoint&, Position&);
		friend Position close_pose(Position&, Position&, float);
		friend Position close_pose(Position&, human_aware_collaboration_planner::Waypoint&, float);
		friend Position close_pose_2D(Position&, Position&, float);
		friend Position close_pose_2D(Position&, human_aware_collaboration_planner::Waypoint&, float);
		//Getters
		std::string getID();
		float getX();
		float getY();
		float getZ();
    void print(std::ostream& os) const;
};

std::ostream& operator << (std::ostream& os, const Position& p) {
	p.print(os);
	return os;
}
		
float distance(Position&, Position&);
float distance(Position&, human_aware_collaboration_planner::Waypoint&);
float distance(human_aware_collaboration_planner::Waypoint&, Position&);
float distance(human_aware_collaboration_planner::Waypoint&, human_aware_collaboration_planner::Waypoint&);
Position close_pose(Position& orig, Position& dest, float dist);
Position close_pose(Position&, human_aware_collaboration_planner::Waypoint&, float);
Position close_pose_2D(Position&, Position&, float);
Position close_pose_2D(Position&, human_aware_collaboration_planner::Waypoint&, float);
human_aware_collaboration_planner::Waypoint central_position(std::vector<human_aware_collaboration_planner::Waypoint>&);

class HumanTarget {
  friend class Monitor;
  friend class DeliverTool;
  
  protected:
    std::string id_;
    Position position_;
  public:
    HumanTarget();
    HumanTarget(std::string id, std::string position_id, float x, float y, float z);
    HumanTarget(std::string id, float x, float y, float z);
    HumanTarget(std::string id, const Position& position);
    HumanTarget(const HumanTarget& h);
    ~HumanTarget();
		//Getters
		std::string getID();
		Position getPosition();
    //void updatePosition();
};

class Tool {
	friend class DeliverTool;

	protected:
		std::string id_;
		float weight_;
		Position position_;
	public:
    Tool();
    Tool(std::string id, float weight, std::string position_id, float x, float y, float z);
		Tool(std::string id, float weight, float x, float y, float z);
		Tool(std::string id, float weight, const Position& position);
    Tool(const Tool& t);
		~Tool();
		//Getters
		std::string getID();
		float getWeight();
		Position getPosition();
    //void updatePosition();
};

class Task {
  protected:
    std::string id_;
  public:
    Task();
    Task(std::string id);
    Task(const Task& t);
    ~Task();
		//Getters
    virtual std::string getID();
    virtual char getType();
		virtual const HumanTarget* getHumanPtr();
		virtual std::string getHumanID();
		virtual Position getHumanPosition();
		virtual float getDistance();
		virtual int getNumber();
		virtual std::vector<std::string> getAgentList();
		virtual std::vector<human_aware_collaboration_planner::Waypoint> getInspectWaypoints();
		virtual Tool getTool();
		virtual std::string getToolID();
		virtual const Tool* getToolPtr();
		virtual Position getToolPosition();
		virtual Position* getChargingStationPtr();
		virtual Position getChargingStation();
		virtual std::string getChargingStationID();
		virtual float getInitialPercentage();
		virtual float getFinalPercentage();
    virtual void print(std::ostream& os) const;
		//Setters
		virtual void setWaypoints(std::vector<human_aware_collaboration_planner::Waypoint> waypoints);
		virtual void setAgentList(std::vector<std::string> agent_list);
		virtual void setChargingStation(Position* charging_station);
		virtual void setInitialPercentage(float initial_percentage);
		virtual void setFinalPercentage(float final_percentage);
		virtual void updateParams(classes::Task* task);
};

class Monitor : public Task {
  protected:
    const HumanTarget* human_target_;
		float distance_;
		int number_;
		std::vector<std::string> agent_list_;
  public:
    Monitor();
    Monitor(std::string task_id, const HumanTarget* human_target, float distance, int number);
    Monitor(std::string task_id, const HumanTarget* human_target, float distance, int number, 
				std::vector<std::string> agent_list);
    Monitor(const Monitor& m);
    ~Monitor();
		//Getters
    std::string getID();
    char getType();
		const HumanTarget* getHumanPtr();
		std::string getHumanID();
		Position getHumanPosition();
		float getDistance();
		int getNumber();
		std::vector<std::string> getAgentList();
    void print(std::ostream& os) const;
		//Setters
		void setAgentList(std::vector<std::string> agent_list);
		void updateParams(classes::Task* task);
};

class Inspect : public Task {
  protected:
		std::vector<human_aware_collaboration_planner::Waypoint> waypoints_;
		std::vector<std::string> agent_list_;
  public:
    Inspect();
    Inspect(std::string task_id, std::vector<human_aware_collaboration_planner::Waypoint> waypoints);
    Inspect(std::string task_id, std::vector<human_aware_collaboration_planner::Waypoint> waypoints, 
				std::vector<std::string> agent_list);
    Inspect(const Inspect& i);
    ~Inspect();
		//Getters
    std::string getID();
    char getType();
		std::vector<human_aware_collaboration_planner::Waypoint> getInspectWaypoints();
		std::vector<std::string> getAgentList();
    void print(std::ostream& os) const;
		//Setters
		void setWaypoints(std::vector<human_aware_collaboration_planner::Waypoint> waypoints);
		void setAgentList(std::vector<std::string> agent_list);
		void updateParams(classes::Task* task);
};

class DeliverTool : public Task {
  protected:
    const Tool* tool_;
    const HumanTarget* human_target_;
  public:
    DeliverTool();
    DeliverTool(std::string task_id, const Tool* tool, const HumanTarget* human_target);
    DeliverTool(const DeliverTool& d);
    ~DeliverTool();
		//Getters
    std::string getID();
    char getType();
		Tool getTool();
		std::string getToolID();
		const Tool* getToolPtr();
		Position getToolPosition();
		const HumanTarget* getHumanPtr();
		std::string getHumanID();
		Position getHumanPosition();
    void print(std::ostream& os) const;
		//Setters
		void updateParams(classes::Task* task);
};

class Recharge : public Task {
  protected:
    Position* charging_station_;
		float initial_percentage_;
		float final_percentage_;
  public:
    Recharge();
    Recharge(std::string task_id, float initial_percentage, float final_percentage);
    Recharge(std::string task_id, Position* charging_station, float initial_percentage, float final_percentage);
    Recharge(const Recharge& r);
    ~Recharge();
		//Getters
    std::string getID();
    char getType();
		Position* getChargingStationPtr();
		Position getChargingStation();
		std::string getChargingStationID();
		float getInitialPercentage();
		float getFinalPercentage();
    void print(std::ostream& os) const;
		//Setters
		void setChargingStation(Position* charging_station);
		void setInitialPercentage(float initial_percentage);
		void setFinalPercentage(float final_percentage);
		void updateParams(classes::Task* task);
};

class Wait : public Task {
  protected:

  public:
    Wait();
    Wait(const Wait& w);
    ~Wait();
		//Getters
    std::string getID();
    char getType();
    void print(std::ostream& os) const;
		//Setters
};

std::ostream& operator << (std::ostream& os, const Task& m) {
	m.print(os);
	return os;
}

} //End of namespace

#endif
