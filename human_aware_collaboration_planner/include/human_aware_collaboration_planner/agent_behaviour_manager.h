#ifndef AGENT_H
#define AGENT_H

#include "ros/ros.h"
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

#include <string>
#include <sstream>
#include <queue>
#include <actionlib/server/simple_action_server.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "human_aware_collaboration_planner/classes.h"
#include "human_aware_collaboration_planner/AgentBeacon.h"
#include "human_aware_collaboration_planner/PlannerBeacon.h"
#include "human_aware_collaboration_planner/NewTaskListAction.h"
#include "human_aware_collaboration_planner/BatteryEnoughAction.h"
#include "human_aware_collaboration_planner/TaskResultAction.h"
#include "human_aware_collaboration_planner/MissionOver.h"
#include "human_aware_collaboration_planner/Waypoint.h"
#include "human_aware_collaboration_planner/Task.h"

#include "uav_abstraction_layer/Land.h"
#include "uav_abstraction_layer/TakeOff.h"
#include "uav_abstraction_layer/GoToWaypoint.h"
#include "uav_abstraction_layer/State.h"
#include "mrs_actionlib_interface/State.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "geographic_msgs/GeoPose.h"
#include "geographic_msgs/GeoPoint.h"
#include "mrs_msgs/UavStatus.h"
#include "mrs_actionlib_interface/commandAction.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/NavSatFix.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/decorator_node.h"

#include <uav_abstraction_layer/geographic_to_cartesian.h>

#include "ist_use_collaboration_msgs/RequestMobileChargingStationAction.h"

//Forward declaration
class AgentNode;

//Behavior Tree Nodes declaration ***********************************************************************************
//******************************* Actions
//GoNearChargingStation
class GoNearChargingStation : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearChargingStation(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearChargingStation();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//Recharge
class Recharge : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		Recharge(const std::string& name, const BT::NodeConfiguration& config);
		~Recharge();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//BackToStation
class BackToStation : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		BackToStation(const std::string& name, const BT::NodeConfiguration& config);
		~BackToStation();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearHumanTarget
class GoNearHumanTarget : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearHumanTarget(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearHumanTarget();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//MonitorHumanTarget
class MonitorHumanTarget : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		MonitorHumanTarget(const std::string& name, const BT::NodeConfiguration& config);
		~MonitorHumanTarget();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearUGV
class GoNearUGV : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearUGV(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearUGV();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//MonitorUGV
class MonitorUGV : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		MonitorUGV(const std::string& name, const BT::NodeConfiguration& config);
		~MonitorUGV();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearWP
class GoNearWP : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearWP(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearWP();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//TakeImage
class TakeImage : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		TakeImage(const std::string& name, const BT::NodeConfiguration& config);
		~TakeImage();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//InspectPVArray
class InspectPVArray : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		InspectPVArray(const std::string& name, const BT::NodeConfiguration& config);
		~InspectPVArray();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//GoNearStation
class GoNearStation : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		GoNearStation(const std::string& name, const BT::NodeConfiguration& config);
		~GoNearStation();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//PickTool
class PickTool : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		PickTool(const std::string& name, const BT::NodeConfiguration& config);
		~PickTool();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//DropTool
class DropTool : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		DropTool(const std::string& name, const BT::NodeConfiguration& config);
		~DropTool();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//DeliverTool
class DeliverTool : public BT::AsyncActionNode
{
  private:
    AgentNode* agent_;

  public:
		DeliverTool(const std::string& name, const BT::NodeConfiguration& config);
		~DeliverTool();
    void init(AgentNode* agent);
		static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    virtual void halt() override;
};

//******************************* Conditions
//MissionOver
class MissionOver : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		MissionOver(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//Idle
class Idle : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		Idle(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsBatteryEnough
class IsBatteryEnough : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsBatteryEnough(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsBatteryFull
class IsBatteryFull : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsBatteryFull(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskRecharge
class IsTaskRecharge : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskRecharge(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskMonitor
class IsTaskMonitor : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskMonitor(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskMonitorUGV
class IsTaskMonitorUGV : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskMonitorUGV(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskInspect
class IsTaskInspect : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskInspect(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskInspectPVArray
class IsTaskInspectPVArray : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskInspectPVArray(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsTaskDeliverTool
class IsTaskDeliverTool : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsTaskDeliverTool(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearChargingStation
class IsAgentNearChargingStation : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearChargingStation(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearHumanTarget
class IsAgentNearHumanTarget : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearHumanTarget(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearUGV
class IsAgentNearUGV : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearUGV(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearWP
class IsAgentNearWP : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearWP(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//NeedToDropTheTool
class NeedToDropTheTool : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		NeedToDropTheTool(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//HasAgentTheTool
class HasAgentTheTool : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		HasAgentTheTool(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//IsAgentNearStation
class IsAgentNearStation : public BT::ConditionNode
{
  private:
    AgentNode* agent_;

	public:
		IsAgentNearStation(const std::string& name);
    void init(AgentNode* agent);
		BT::NodeStatus tick() override;
};

//******************************* Decorators
//ForceRunnigNode
class ForceRunningNode : public BT::DecoratorNode
{
  public:
    ForceRunningNode(const std::string& name);

  private:
    virtual BT::NodeStatus tick() override;
};

//Behavior Tree Nodes registration function *************************************************************************
inline void RegisterNodes(BT::BehaviorTreeFactory& factory);

//Agent Node Class declaration **************************************************************************************
class AgentNode
{
  //Actions
	friend class GoNearChargingStation;
  friend class Recharge;
  friend class BackToStation;
  friend class GoNearHumanTarget;
  friend class MonitorHumanTarget;
  friend class GoNearUGV;
  friend class MonitorUGV;
  friend class GoNearWP;
  friend class TakeImage;
  friend class InspectPVArray;
  friend class GoNearStation;
  friend class PickTool;
  friend class DropTool;
  friend class DeliverTool;

  //Conditions
  friend class MissionOver;
  friend class Idle;
  friend class IsBatteryEnough;
  friend class IsBatteryFull;
  friend class IsTaskRecharge;
  friend class IsTaskMonitor;
  friend class IsTaskMonitorUGV;
  friend class IsTaskInspect;
  friend class IsTaskInspectPVArray;
  friend class IsTaskDeliverTool;
  friend class IsTaskRecharge;
  friend class IsAgentNearHumanTarget;
  friend class IsAgentNearUGV;
  friend class IsAgentNearWP;
  friend class NeedToDropTheTool;
  friend class HasAgentTheTool;
  friend class IsAgentNearStation;
  friend class IsAgentNearChargingStation;

  //Decorators
  friend class ForceRunningNode;

  private:
    //Node Handlers
    ros::NodeHandle nh_;

		std::string id_;
		std::string ns_prefix_;

		geographic_msgs::GeoPoint origin_geo_;

    //Action Server to receive TaskList
    actionlib::SimpleActionServer<human_aware_collaboration_planner::NewTaskListAction> ntl_as_;
    human_aware_collaboration_planner::NewTaskListFeedback ntl_feedback_;
    human_aware_collaboration_planner::NewTaskListResult ntl_result_;

		//Action Client to communicate with the MRS System
		actionlib::SimpleActionClient<mrs_actionlib_interface::commandAction> mrs_ac_;

    //Action Client to Battery Enough
    actionlib::SimpleActionClient<human_aware_collaboration_planner::BatteryEnoughAction> battery_ac_;

    // Subscribers: ual (position), mavros(battery), ual(state), UGV geo position
    ros::Subscriber position_sub_; 
    ros::Subscriber battery_sub_;
		ros::Subscriber state_sub_;
		ros::Subscriber mission_over_sub_;
		ros::Subscriber planner_beacon_sub_;
		ros::Subscriber atrvjr_geopose_sub_;
		ros::Subscriber jackal_geopose_sub_;
    classes::Position position_;
    float battery_; //percentage
		int state_;
		bool mission_over_;
		ros::Time last_beacon_;
		bool timeout_;
    classes::Position atrvjr_pose_;
    classes::Position jackal_pose_;

    // Publishers
    ros::Publisher beacon_pub_;
    human_aware_collaboration_planner::AgentBeacon beacon_;
    ros::Rate loop_rate_;

    std::queue<classes::Task*> task_queue_;
    bool battery_enough_;
		std::string tool_flag_;
		std::string pose_frame_id_;
		std::string low_level_interface_;
		std::string pose_topic_;
		std::string state_topic_;
		std::string battery_topic_;

    std::string config_file_;
    std::string config_file_evora_;

    std::map <std::string, std::map <std::string, classes::Position>> known_positions_;
    std::map <std::string, classes::HumanTarget> human_targets_;
    std::map <std::string, classes::Tool> tools_;

  protected:
    void readConfigFile(std::string config_file);
    void readEvoraConfigFile(std::string config_file);

  public:
    AgentNode(human_aware_collaboration_planner::AgentBeacon beacon);
    ~AgentNode();
		void isBatteryEnough();
    //Task queue methods
    void addTaskToQueue(classes::Task* task);
    void removeTaskFromQueue(std::string id, char type);
    void emptyTheQueue();
    int getQueueSize();
    void infoQueue();
		void taskQueueManager();
    //New Task List Action callback
    void newTaskList(const human_aware_collaboration_planner::NewTaskListGoalConstPtr& goal);
    void positionCallbackUAL(const geometry_msgs::PoseStamped& pose);
    void positionCallbackMRS(const mrs_msgs::UavStatus& pose);
    void batteryCallback(const sensor_msgs::BatteryState& battery);
		void stateCallbackUAL(const uav_abstraction_layer::State& state);
		void stateCallbackMRS(const mrs_actionlib_interface::State& state);
		void missionOverCallback(const human_aware_collaboration_planner::MissionOver& value);
		void beaconCallback(const human_aware_collaboration_planner::PlannerBeacon& beacon);
		bool checkBeaconTimeout(ros::Time now);
		void atrvjrPositionCallback(const geographic_msgs::GeoPoseStamped& geo_pose);
		void jackalPositionCallback(const geographic_msgs::GeoPoseStamped& geo_pose);
		// UAL/MRS Service calls
		bool land(bool blocking);
		bool take_off(float height, bool blocking);
		bool go_to_waypoint(float x, float y, float z, bool blocking);
		bool stop(bool blocking);
		bool checkIfGoToServiceSucceeded(float x, float y, float z);
};

#endif
