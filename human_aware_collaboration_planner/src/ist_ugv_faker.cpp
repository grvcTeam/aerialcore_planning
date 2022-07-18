#include "ros/ros.h"
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include "ist_use_collaboration_msgs/RequestMobileChargingStationAction.h"
#include "ist_use_collaboration_msgs/DoCloserInspectionAction.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include "geographic_msgs/GeoPoseStamped.h"

class ISTugvFaker{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ist_use_collaboration_msgs::RequestMobileChargingStationAction> mobile_station_as_;
    actionlib::SimpleActionServer<ist_use_collaboration_msgs::DoCloserInspectionAction> closer_inspection_as_;

    ist_use_collaboration_msgs::RequestMobileChargingStationResult mobile_station_result_;
    ist_use_collaboration_msgs::DoCloserInspectionResult closer_inspection_result_;

    // UGV Pose Fakers Publishers
    ros::Publisher atrvjr_pose_pub_;
    ros::Publisher jackal_pose_pub_;
    geographic_msgs::GeoPoseStamped atrvjr_pose_; 
    geographic_msgs::GeoPoseStamped jackal_pose_; 
    ros::Rate pose_rate_;

  public:
    ISTugvFaker() : pose_rate_(0.2),
      mobile_station_as_(nh_, "/jackal0/cooperation_use/request_mobile_charging_station", boost::bind(&ISTugvFaker::mobileStationCB, this, _1), false),
      closer_inspection_as_(nh_, "/atrvjr/cooperation_use/do_closer_inspection", boost::bind(&ISTugvFaker::closerInspectionCB, this, _1), false) {
        mobile_station_as_.start();
        closer_inspection_as_.start();

        atrvjr_pose_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("/atrvjr/geopose", 1);
        jackal_pose_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("/jackal0/geopose", 1);

        //Position: solar pannel row 14 column 2
        atrvjr_pose_.pose.position.latitude = 38.54143688611777;
        atrvjr_pose_.pose.position.longitude = -7.961302628908529;
        atrvjr_pose_.pose.position.altitude = 227.61875915527344;

        //Position: corner 4
        jackal_pose_.pose.position.latitude = 38.54131475760623;
        jackal_pose_.pose.position.longitude = -7.961691219061984;
        jackal_pose_.pose.position.altitude = 227.61875915527344;

        pose_rate_.reset();
        while(ros::ok())
        {
          ros::spinOnce();
          atrvjr_pose_pub_.publish(atrvjr_pose_);
          jackal_pose_pub_.publish(jackal_pose_);
          pose_rate_.sleep();
        }
      }

    ~ISTugvFaker(void){}

    void mobileStationCB(const ist_use_collaboration_msgs::RequestMobileChargingStationGoalConstPtr &goal) {
      mobile_station_result_.success = true;
      ROS_INFO("Requested mobile charging station. Returning SUCCESS");
      mobile_station_as_.setSucceeded(mobile_station_result_);
    }

    void closerInspectionCB(const ist_use_collaboration_msgs::DoCloserInspectionGoalConstPtr &goal) {
      closer_inspection_result_.success = true;
      ROS_INFO("Requested closer inspection. Returning SUCCESS");
      closer_inspection_as_.setSucceeded(closer_inspection_result_);
    }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "ist_ugv_faker");

  ISTugvFaker ist_ugv_faker;
  ROS_INFO("Ending");

  return 0;
}
