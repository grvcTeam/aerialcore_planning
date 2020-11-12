/**
 * AERIALCORE Project:
 *
 * Mission controller.
 * 
 */

#include <mission_controller.h>

namespace aerialcore {


// Constructor
MissionController::MissionController() {
    pnh_ = ros::NodeHandle("~");

    // Read parameters of the UAVs (from the launch):
    std::vector<int>   drones_id;
    std::vector<int>   drones_time_max_flying;
    std::vector<float> drones_speed_xy;
    std::vector<float> drones_speed_z_down;
    std::vector<float> drones_speed_z_up;
    pnh_.getParam("drones_id", drones_id);
    pnh_.getParam("drones_time_max_flying", drones_time_max_flying);
    pnh_.getParam("drones_speed_xy", drones_speed_xy);
    pnh_.getParam("drones_speed_z_down", drones_speed_z_down);
    pnh_.getParam("drones_speed_z_up", drones_speed_z_up);
    if ( (drones_id.size() + drones_time_max_flying.size() + drones_speed_xy.size() + drones_speed_z_down.size() + drones_speed_z_up.size())/5!=drones_id.size() ) {
        ROS_ERROR("Mission Controller: error in the description of the UAVs (launch file), all parameters should have the same size.");
        exit(EXIT_FAILURE);
    } else if (drones_id.size() == 0) {
        ROS_ERROR("Mission Controller: error in the description of the UAVs (launch file), no drones found.");
        exit(EXIT_FAILURE);
    }
    for (int i=0; i<drones_id.size(); i++) {
        UAV new_uav;
        new_uav.mission = new grvc::mission_ns::Mission(drones_id[i]);
        new_uav.id = drones_id[i];
        new_uav.time_max_flying = drones_time_max_flying[i];
        new_uav.speed_xy = drones_speed_xy[i];
        new_uav.speed_z_down = drones_speed_z_down[i];
        new_uav.speed_z_up = drones_speed_z_up[i];
        UAVs_.push_back(new_uav);
    }

    // Read parameters of the complete graph (from the yaml). Numeric parameters extracted from a string, so some steps are needed:
    std::string pylons_position_string;
    std::string connections_indexes_string;
    std::string recharge_land_stations_string;
    std::string regular_land_stations_string;
    n_.getParam("pylons_position", pylons_position_string);
    n_.getParam("connections_indexes", connections_indexes_string);
    n_.getParam("recharge_land_stations", recharge_land_stations_string);
    n_.getParam("regular_land_stations", regular_land_stations_string);
    // Remove new lines if exist in the strings for all strings and semicolons in all but connection_indexes:
    pylons_position_string.erase(std::remove(pylons_position_string.begin(), pylons_position_string.end(), '\n'), pylons_position_string.end());
    pylons_position_string.erase(std::remove(pylons_position_string.begin(), pylons_position_string.end(), '\r'), pylons_position_string.end());
    pylons_position_string.erase(std::remove(pylons_position_string.begin(), pylons_position_string.end(), ';'), pylons_position_string.end());
    connections_indexes_string.erase(std::remove(connections_indexes_string.begin(), connections_indexes_string.end(), '\n'), connections_indexes_string.end());
    connections_indexes_string.erase(std::remove(connections_indexes_string.begin(), connections_indexes_string.end(), '\r'), connections_indexes_string.end());
    recharge_land_stations_string.erase(std::remove(recharge_land_stations_string.begin(), recharge_land_stations_string.end(), '\n'), recharge_land_stations_string.end());
    recharge_land_stations_string.erase(std::remove(recharge_land_stations_string.begin(), recharge_land_stations_string.end(), '\r'), recharge_land_stations_string.end());
    recharge_land_stations_string.erase(std::remove(recharge_land_stations_string.begin(), recharge_land_stations_string.end(), ';'), recharge_land_stations_string.end());
    regular_land_stations_string.erase(std::remove(regular_land_stations_string.begin(), regular_land_stations_string.end(), '\n'), regular_land_stations_string.end());
    regular_land_stations_string.erase(std::remove(regular_land_stations_string.begin(), regular_land_stations_string.end(), '\r'), regular_land_stations_string.end());
    regular_land_stations_string.erase(std::remove(regular_land_stations_string.begin(), regular_land_stations_string.end(), ';'), regular_land_stations_string.end());
    // For strings with point information, convert the string data, clean and ready, to double (float64 because of point) with stod.
    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
    // The quantity of numbers have to be multiple of 2.
    // For connection_indexes do that but separating previously strings by semicolons:
    std::string::size_type sz;
    aerialcore_msgs::GraphNode current_graph_node;
    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_PYLON;
    while ( pylons_position_string.size()>0 ) {
        current_graph_node.x = std::stod (pylons_position_string,&sz);
        pylons_position_string = pylons_position_string.substr(sz);
        current_graph_node.y = std::stod (pylons_position_string,&sz);
        pylons_position_string = pylons_position_string.substr(sz);
        complete_graph_.push_back(current_graph_node);
    }
    for (int i=0; i<complete_graph_.size(); i++) {
        std::string delimiter = ";";
        std::string string_until_first_semicolon = connections_indexes_string.substr(0, connections_indexes_string.find(delimiter));
        int current_connection_index;
        while ( string_until_first_semicolon.size()>0 ) {
            current_connection_index = (int) std::stod (string_until_first_semicolon,&sz);
            string_until_first_semicolon = string_until_first_semicolon.substr(sz);
            complete_graph_[i].connections_indexes.push_back(current_connection_index-1);
        }
        connections_indexes_string.erase(0, connections_indexes_string.find(delimiter) + delimiter.length());
    }
    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION;
    while ( recharge_land_stations_string.size()>0 ) {
        current_graph_node.x = (float) std::stod (recharge_land_stations_string,&sz);
        recharge_land_stations_string = recharge_land_stations_string.substr(sz);
        current_graph_node.y = (float) std::stod (recharge_land_stations_string,&sz);
        recharge_land_stations_string = recharge_land_stations_string.substr(sz);
        complete_graph_.push_back(current_graph_node);
    }
    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION;
    while ( regular_land_stations_string.size()>0 ) {
        current_graph_node.x = (float) std::stod (regular_land_stations_string,&sz);
        regular_land_stations_string = regular_land_stations_string.substr(sz);
        current_graph_node.y = (float) std::stod (regular_land_stations_string,&sz);
        regular_land_stations_string = regular_land_stations_string.substr(sz);
        complete_graph_.push_back(current_graph_node);
    }
    // for (int i=0; i<complete_graph_.size(); i++) {      // Print complete_graph_ to check that the yaml file was parsed correctly:
    //     std::cout << "graph_node[ " << i << " ].type                     = " << (int) complete_graph_[i].type << std::endl;
    //     for (int j=0; j<complete_graph_[i].connections_indexes.size(); j++) {
    //         std::cout << "graph_node[ " << i << " ].connections_indexes[ " << j << " ] = " << complete_graph_[i].connections_indexes[j] << std::endl;
    //     }
    //     std::cout << "graph_node[ " << i << " ].x                        = " << complete_graph_[i].x << std::endl;
    //     std::cout << "graph_node[ " << i << " ].y                        = " << complete_graph_[i].y << std::endl;
    // }

    // Advertised services
    start_supervising_srv_ = n_.advertiseService("mission_controller/start_supervising", &MissionController::startSupervisingServiceCallback, this);
    stop_supervising_srv_ = n_.advertiseService("mission_controller/stop_supervising", &MissionController::stopSupervisingServiceCallback, this);
    do_specific_supervision_srv_ = n_.advertiseService("mission_controller/do_specific_supervision", &MissionController::doSpecificSupervisionServiceCallback, this);
    do_continuous_supervision_srv_ = n_.advertiseService("mission_controller/do_continuous_supervision", &MissionController::doContinuousSupervisionServiceCallback, this);

    // Timer Callback
    timer_ = n_.createTimer(ros::Duration(1), &MissionController::timerCallback, this); // 1 Hz

    // Make communications spin!
    spin_thread_ = std::thread([this]() {
        ros::MultiThreadedSpinner spinner(2); // Use 2 threads
        spinner.spin();
    });

    ROS_INFO("Mission Controller running!");

} // end MissionController constructor


// Brief Destructor
MissionController::~MissionController() {
    if(spin_thread_.joinable()) spin_thread_.join();
    for (UAV &uav : UAVs_) {
        delete uav.mission;
    }
} // end ~MissionController destructor


void MissionController::timerCallback(const ros::TimerEvent&) {
    for (int i=0; i<UAVs_.size(); i++) {
        UAVs_[i].pose_stamped = UAVs_[i].mission->pose();
        UAVs_[i].battery_percentage = UAVs_[i].mission->battery();
    }
} // end timerCallback


void MissionController::translateFlightPlanIntoUAVMission(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plan) {
    for (aerialcore_msgs::FlightPlan const &flight_plan_for_current_uav : _flight_plan) {
        int current_uav_index = -1;
        for (int i=0; i<UAVs_.size(); i++) {
            if (UAVs_[i].id == flight_plan_for_current_uav.uav_id) {
                current_uav_index = i;
                break;
            }
        }
        if (current_uav_index == -1) {
            ROS_ERROR("Mission Controller: UAV id provided in the FlightPlan not found on the Mission Controller.");
            exit(EXIT_FAILURE);
        }

        UAVs_[current_uav_index].mission->clear();

        bool first_iteration = true;
        bool flying_or_landed; // True if flying at the time after the wp is inserted, false if landed. Useful to track takeoffs vs landings, both with negative nodes.

        std::vector<geometry_msgs::PoseStamped> pass_poses;

        for (int const &current_node : flight_plan_for_current_uav.nodes) {
            geometry_msgs::PoseStamped current_pose_stamped;
            current_pose_stamped.pose.position.x = current_graph_[current_node].x;
            current_pose_stamped.pose.position.y = current_graph_[current_node].y;

            if (current_graph_[current_node].type != aerialcore_msgs::GraphNode::TYPE_PYLON) {
                if (pass_poses.size()>0) {
                    UAVs_[current_uav_index].mission->addPassWpList(pass_poses);
                }
                if (first_iteration) {
                    UAVs_[current_uav_index].mission->addTakeOffWp(current_pose_stamped);
                    flying_or_landed = true;
                    first_iteration=false;
                } else {
                    if (flying_or_landed) {
                        if (UAVs_[current_uav_index].mission->airframeType() == grvc::mission_ns::AirframeType::FIXED_WING) {
                            geometry_msgs::PoseStamped loiter_to_alt_start_landing_pose_pose_stamped;
                            loiter_to_alt_start_landing_pose_pose_stamped.pose.position.x = pass_poses.back().pose.position.x;
                            loiter_to_alt_start_landing_pose_pose_stamped.pose.position.y = pass_poses.back().pose.position.y;
                            UAVs_[current_uav_index].mission->addLandWp(loiter_to_alt_start_landing_pose_pose_stamped, current_pose_stamped);
                        } else {
                            UAVs_[current_uav_index].mission->addLandWp(current_pose_stamped);
                        }
                        flying_or_landed = false;
                    } else {
                        UAVs_[current_uav_index].mission->addTakeOffWp(current_pose_stamped);
                        flying_or_landed = true;
                    }
                }
                pass_poses.clear();
            } else {
                pass_poses.push_back(current_pose_stamped);
                flying_or_landed = true;
                if (first_iteration) {
                    first_iteration=false;
                }
            }
        }
    }
} // end translateFlightPlanIntoUAVMission


bool MissionController::startSupervisingServiceCallback(aerialcore_msgs::StartSupervising::Request& _req, aerialcore_msgs::StartSupervising::Response& _res) {
    if (_req.uav_id.size()>0) {     // There are specific UAVs to start.
        for (int const &current_uav_id : _req.uav_id) {
            int current_uav_index = -1;
            for (int i=0; i<UAVs_.size(); i++) {
                if (UAVs_[i].id == current_uav_id) {
                    current_uav_index = i;
                    break;
                }
            }
            if (current_uav_index == -1) {
                ROS_ERROR("Mission Controller: UAV id provided in the StartSupervising not found on the Mission Controller.");
                exit(EXIT_FAILURE);
            }

            UAVs_[current_uav_index].enabled_to_supervise = true;
        }
    } else {    // If empty start all UAVs.
        for (UAV &current_UAV : UAVs_) {
            current_UAV.enabled_to_supervise = true;
        }
    }
    // TODO: COMPLETE THIS
    // flight_plan_ = centralized_planner_.getPlan();
    // translateFlightPlanIntoUAVMission(flight_plan_);
    // for (UAV &current_UAV : UAVs_) {
    //     if( current_UAV.enabled_to_supervise == true) {
    //         current_UAV.mission->start();
    //     }
    // }
    _res.success=true;
    return true;
} // end startSupervisingServiceCallback


bool MissionController::stopSupervisingServiceCallback(aerialcore_msgs::StopSupervising::Request& _req, aerialcore_msgs::StopSupervising::Response& _res) {
    if (_req.uav_id.size()>0) {     // There are specific UAVs to stop.
        for (int const &current_uav_id : _req.uav_id) {
            int current_uav_index = -1;
            for (int i=0; i<UAVs_.size(); i++) {
                if (UAVs_[i].id == current_uav_id) {
                    current_uav_index = i;
                    break;
                }
            }
            if (current_uav_index == -1) {
                ROS_ERROR("Mission Controller: UAV id provided in the StopSupervising not found on the Mission Controller.");
                exit(EXIT_FAILURE);
            }

            UAVs_[current_uav_index].mission->stop();
            UAVs_[current_uav_index].enabled_to_supervise = false;
        }
    } else {    // If empty stop all UAVs.
        for (UAV &current_UAV : UAVs_) {
            current_UAV.mission->stop();
            current_UAV.enabled_to_supervise = false;
        }
    }
    _res.success=true;
    return true;
} // end stopSupervisingServiceCallback


bool MissionController::doSpecificSupervisionServiceCallback(aerialcore_msgs::DoSpecificSupervision::Request& _req, aerialcore_msgs::DoSpecificSupervision::Response& _res) {
    continuous_or_specific_supervision_ = false;
    specific_subgraph_.clear();
    for (aerialcore_msgs::GraphNode const &current_graph_node : _req.specific_subgraph) {
        specific_subgraph_.push_back(current_graph_node);
    }
    current_graph_.clear();
    current_graph_ = specific_subgraph_;
    _res.success=true;
    return true;
} // end doSpecificSupervisionServiceCallback


bool MissionController::doContinuousSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res) {
    continuous_or_specific_supervision_ = true;
    current_graph_.clear();
    current_graph_ = complete_graph_;
    _res.success=true;
    return true;
} // end doContinuousSupervisionServiceCallback


} // end namespace aerialcore