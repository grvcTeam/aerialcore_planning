#include "ros/ros.h"
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

#include <string>
#include <map>

#include "human_aware_collaboration_planner/classes.h"
#include "human_aware_collaboration_planner/BatteryControl.h"

#include "uav_abstraction_layer/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/BatteryState.h"

class BatteryFaker{
  private:
    std::string id_;

    int mode_;
    float battery_increase_;
    float battery_decrease_;
		int state_;

    classes::Position position_;

		std::string pose_topic_;
		std::string state_topic_;
    std::string config_file;
    std::map <std::string, std::map <std::string, classes::Position>> known_positions_;

    //Node Handlers
    ros::NodeHandle nh_;

    // Subscribers: battery_control, ual (position), ual(state)
    ros::Subscriber control_sub_; 
    ros::Subscriber position_sub_; 
		ros::Subscriber state_sub_;

    // Publishers
    ros::Publisher battery_pub_;
    sensor_msgs::BatteryState battery_;
    ros::Rate loop_rate_;

    //Service Server

  public:
    BatteryFaker() : loop_rate_(0.2), mode_(2), battery_increase_(0.01), battery_decrease_(0.01){
      ros::param::param<std::string>("~id", id_, "i");
      ros::param::param<std::string>("~pose_topic", pose_topic_, "/" + id_ + "/ual/pose");
      ros::param::param<std::string>("~state_topic", state_topic_, "/" + id_ + "/ual/state");

      //load of known position and human targets known positions
      std::string path = ros::package::getPath("human_aware_collaboration_planner");
      ros::param::param<std::string>("~config_file", config_file, path + "/config/conf.yaml");

      ROS_INFO("Reading config file...");
      readConfigFile(config_file);

      battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/" + id_ + "/battery_fake", 1);

      control_sub_ = nh_.subscribe("/" + id_ + "/battery_fake/control", 1, &BatteryFaker::controlCallback, this);

      position_sub_ = nh_.subscribe(pose_topic_, 1, &BatteryFaker::positionCallbackUAL, this);
      state_sub_ = nh_.subscribe(state_topic_, 1, &BatteryFaker::stateCallbackUAL, this);

      ROS_INFO("READY");
      battery_.percentage = 0.9;
      loop_rate_.reset();
      while(ros::ok())
      {
        ros::spinOnce();
        update_battery();
        loop_rate_.sleep();
      }
    }
    ~BatteryFaker(){}
    void update_battery(){
      int flag;
      switch(mode_)
      {
        case 1: //Recharge anywhere when landed
          switch(state_)
          {
            case 1: //LANDED_DISARMED
            case 2: //LANDED_ARMED
              battery_.percentage = battery_.percentage + battery_increase_;
              if(battery_.percentage > 1)
                battery_.percentage = 1;
              break;
            case 3: //TAKING_OFF
            case 4: //FLYING_AUTO
            case 5: //FLYING_MANUAL
            case 6: //LANDING
              battery_.percentage = battery_.percentage - battery_decrease_;
              if(battery_.percentage < 0)
                battery_.percentage = 0;
              break;
            case 0: //UNINITIALIZED
            default:
              break;
          }
        case 2: //Recharge only in recharging base
          switch(state_)
          {
            case 1: //LANDED_DISARMED
            case 2: //LANDED_ARMED
              flag = 0;
              for(auto& charging_station : known_positions_["charging_stations"])
              {
                if(classes::distance(position_, charging_station.second) < 0.5)
                  flag = 1;
              }
              if(flag)
              {
                battery_.percentage = battery_.percentage + battery_increase_;
                if(battery_.percentage > 1)
                  battery_.percentage = 1;
              }
              break;
            case 3: //TAKING_OFF
            case 4: //FLYING_AUTO
            case 5: //FLYING_MANUAL
            case 6: //LANDING
              battery_.percentage = battery_.percentage - battery_decrease_;
              if(battery_.percentage < 0)
                battery_.percentage = 0;
              break;
            case 0: //UNINITIALIZED
            default:
              break;
          }
        case 3: //Recharge disabled
          switch(state_)
          {
            case 3: //TAKING_OFF
            case 4: //FLYING_AUTO
            case 5: //FLYING_MANUAL
            case 6: //LANDING
              battery_.percentage = battery_.percentage - battery_decrease_;
              if(battery_.percentage < 0)
                battery_.percentage = 0;
              break;
            case 0: //UNINITIALIZED
            case 1: //LANDED_DISARMED
            case 2: //LANDED_ARMED
            default:
              break;
          }
        case 0: //Battery Static
        default:
          break;
      }
      battery_pub_.publish(battery_);
      //ROS_INFO_STREAM("Mode: " << mode_ << "\tUAV State: " << state_ << "\tPercentage: " << battery_.percentage);
    }
    void readConfigFile(std::string config_file){
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
    }
    //Callbacks
    void controlCallback(const human_aware_collaboration_planner::BatteryControl& control){
      if(control.percentage != -1)
        battery_.percentage = control.percentage;
      mode_ = control.mode;
      if(control.battery_increase >= 0)
        battery_increase_= control.battery_increase;
      if(control.battery_decrease >= 0)
        battery_decrease_= control.battery_decrease;
    }
    void positionCallbackUAL(const geometry_msgs::PoseStamped& pose){
      position_.update(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
    void stateCallbackUAL(const uav_abstraction_layer::State& state){state_ = state.state;}
};

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");

  BatteryFaker battery_faker;
  return 0;
}
