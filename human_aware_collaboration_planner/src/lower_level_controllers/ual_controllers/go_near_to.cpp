#include "ros/ros.h"
#include <ros/package.h>

#include <string>
#include <sstream>

#include <actionlib/server/simple_action_server.h>
#include "human_aware_collaboration_planner/GoNearToAction.h"

#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/ual_backend_dummy.h>
#include <ual_backend_mavros/ual_backend_mavros.h>
#include <ual_backend_gazebo_light/ual_backend_gazebo_light.h>
#include <uav_abstraction_layer/State.h>

class GoNearToAction {
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<human_aware_collaboration_planner::GoNearToAction> as_;
  
  human_aware_collaboration_planner::GoNearToFeedback feedback_;
  human_aware_collaboration_planner::GoNearToResult result_;

  std::string action_name_;

  grvc::ual::UAL *ual_;
  std::string ual_backend;

  std::string prefix;
  std::string id;

  ros::Subscriber position_sub_; 
  ros::Subscriber battery_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber mission_over_sub_;

public:

  GoNearToAction(std::string name) :
    as_(nh_, name, boost::bind(&GoNearToAction::goNearToCB, this, _1), false), action_name_(name) {
      ros::param::param<std::string>("~id", id, "uav_0");
      ros::param::param<std::string>("~ual_backend", ual_backend, "gazebo_light");

      grvc::ual::Backend *backend = nullptr;
      if (ual_backend == "dummy") {
        backend = new grvc::ual::BackendDummy();
      } else if (ual_backend == "mavros") {
        backend = new grvc::ual::BackendMavros();
      } else if (ual_backend == "gazebo_light") {
        backend = new grvc::ual::BackendGazeboLight();
      } else {
        throw std::runtime_error("Unexpected UAL backend");
      }

      ual_ = new grvc::ual::UAL(backend);

      position_sub_ = nh_.subscribe("/" + id + "/ual/pose", 1, &AgentNode::positionCallback, this);
      battery_sub_ = nh_.subscribe("/" + id + "/mavros/battery_fake", 1, &AgentNode::batteryCallback, this);
      state_sub_ = nh_.subscribe("/" + id + "/ual/state", 1, &AgentNode::stateCallback, this);
      mission_over_sub_ = nh_.subscribe("/mission_over", 1, &AgentNode::missionOverCallback, this);

      as_.start();
    }

  ~GoNearToAction(void){}

  void goNearToCB(const actionlib_tutorials::GoNearToGoalConstPtr &goal){
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      return;
    }
    as_.publishFeedback(feedback_);

    result_.sequence = feedback_.sequence;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }


  while(as_.isPreemptRequested() || !ros::ok())
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
              assigned_charging_station.getZ() + 0.4, false))
        {
          while(!agent_->checkIfUALGoToServiceSucceeded(assigned_charging_station.getX(),
                assigned_charging_station.getY(), assigned_charging_station.getZ() + 0.4))
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

  void positionCallback(const geometry_msgs::PoseStamped& pose){
    //TODO: Comment the following when lower level controller for travelling are integrated
    unsigned offset;
    sscanf(beacon_.id.c_str(), "uav_%u", &offset);
    //This correction is needed becouse of the UAL/Land service
    if(pose.pose.position.z - offset < 0.5)
      position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    else
      position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z - offset);
    //position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }

  void batteryCallback(const sensor_msgs::BatteryState& battery){battery_ = battery.percentage;}

  void stateCallback(const uav_abstraction_layer::State& state){state_ = state.state;}

  void missionOverCallback(const human_aware_collaboration_planner::MissionOver& value){mission_over_ = value.value;}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_near_to");

  GoNearToAction fibonacci("go_near_to");
  ros::spin();

  return 0;
}
