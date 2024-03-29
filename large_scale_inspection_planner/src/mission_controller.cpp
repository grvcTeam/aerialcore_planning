/**
 * AERIALCORE Project:
 *
 * Mission controller.
 *
 */

#include <mission_controller.h>

#include <XmlRpcValue.h>
#include <time.h>
#include <algorithm>

#ifdef USING_MS_TSP_PLANNER
#include "ms_tsp_planner/ConfigToFlightPlans.h"
#endif

// #define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)
// #define NUMERICAL_EXPERIMENTS

#define FLYING_THRESHOLD 1  // Height threshold above which an UAV is considered to be flying (in meters). TODO: better way to know if flying?

namespace aerialcore {


// Constructor
MissionController::MissionController() {
    pnh_ = ros::NodeHandle("~");

    pnh_.getParam("UAV_command_mode", UAV_command_mode_);
    if (UAV_command_mode_!="mission_lib" && UAV_command_mode_!="DJI_SDK_yaml" && UAV_command_mode_!="USE_CTU_yaml") {
        ROS_ERROR("Mission Controller: error, UAV_command_mode not valid. Using mission_lib mode as default.");
        std::string UAV_command_mode_ = "mission_lib";
    }

    pnh_.param<bool>("simulation", simulation_, true);
    n_.param<bool>("replanning_enabled", replanning_enabled_, true);

    n_.getParam("planner_method", planner_method_);
    if (planner_method_!="Greedy" && planner_method_!="MILP" && planner_method_!="VNS" && planner_method_!="MEM" && planner_method_!="Mstsp") {
        ROS_ERROR("Mission Controller: error, planner_method not valid. Using VNS algorithm as default.");
        std::string planner_method_ = "VNS";
    }

    n_.getParam("planner_method_electric_fault", planner_method_electric_fault_);
    if (planner_method_electric_fault_!="Minimax-VNS" && planner_method_electric_fault_!="Minimax-MEM" && planner_method_electric_fault_!="Minimax-Greedy") {
        ROS_ERROR("Mission Controller: error, planner_method_electric_fault not valid. Using Minimax-VNS algorithm as default.");
        std::string planner_method_electric_fault_ = "Minimax-VNS";
    }

    // Charge from the YAML file the time between iterations for the loops in the Parameter Estimator and Plan (Monitor and Planner) threads:
    n_.param<float>("parameter_estimator_time", parameter_estimator_time_, 5.0);
    n_.param<float>("plan_monitor_time", plan_monitor_time_, 5.0);

    // map_origin_geo is the geographic coordinate origin of the cartesian coordinates. Loaded to the param server in the YAML config file.
    std::vector<double> map_origin_geo_vector;
    n_.getParam("map_origin_geo", map_origin_geo_vector);
    map_origin_geo_.latitude  = map_origin_geo_vector[0];
    map_origin_geo_.longitude = map_origin_geo_vector[1];
    map_origin_geo_.altitude  = map_origin_geo_vector[2];

    ns_uav_prefix_ = "";
    ros::param::param<std::string>("~ns_uav_prefix",ns_uav_prefix_,"uav_");
    std::string node_name_space_prefix = "";
    if (ros::this_node::getNamespace() == "/") {
        node_name_space_prefix = "/" + ns_uav_prefix_;
    } else {
        node_name_space_prefix = ns_uav_prefix_;
    }

    // Read and construct parameters of the UAVs (from the yaml):
    std::map<std::string, std::string> drones;
    n_.getParam("drones", drones);              // Dictionary of the actual drones used in the simulation (key: UAV id, value: airframe type).
    if (drones.size() == 0) {
        ROS_ERROR("Mission Controller: error in the description of the UAVs (YAML file), no drones found. Are you sure you loaded to the server parameter the config YAML?");
        exit(EXIT_FAILURE);
    }
    for (std::map<std::string, std::string>::iterator it = drones.begin(); it != drones.end(); it++) {
        UAV new_uav;
        new_uav.id = stoi(it->first);
        new_uav.airframe_type = it->second;
        std::string UAV_node_name_space = ns_uav_prefix_ + std::to_string(new_uav.id)+"/";
        if (UAV_command_mode_=="mission_lib") {
            new_uav.mission = new grvc::Mission(new_uav.id, UAV_node_name_space);
        } else {
            new_uav.mission = nullptr;
        }
        n_.getParam(it->second+"/time_max_flying", new_uav.time_max_flying);
        n_.getParam(it->second+"/speed_xy", new_uav.speed_xy);
        n_.getParam(it->second+"/speed_z_down", new_uav.speed_z_down);
        n_.getParam(it->second+"/speed_z_up", new_uav.speed_z_up);
        n_.getParam(it->second+"/minimum_battery", new_uav.minimum_battery);
        n_.getParam(it->second+"/time_until_fully_charged", new_uav.time_until_fully_charged);

        float capacity_in_Wh;
        float number_of_batteries;
        n_.param<float>(it->second+"/Wh", capacity_in_Wh, 0);
        n_.param<float>(it->second+"/nb", number_of_batteries, 0);
        new_uav.joules = capacity_in_Wh*3600/* [s/h] */ * number_of_batteries;

        UAVs_.push_back(new_uav);
    }

    // Read the no-fly zones parameter from the list of polygons:
    XmlRpc::XmlRpcValue no_fly_zones_geo;
    n_.getParam("no_fly_zones_geo", no_fly_zones_geo);
    if (no_fly_zones_geo.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i=0; i<no_fly_zones_geo.size(); i++) {
            if (no_fly_zones_geo[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                geometry_msgs::Polygon current_polygon_obstacle;
                for (int j=0; j<no_fly_zones_geo[i].size(); j++) {
                    geographic_msgs::GeoPoint current_geo_point;
                    std::string::size_type sz;

                    std::stringstream latitude_stringstream;
                    latitude_stringstream << no_fly_zones_geo[i][j][0];
                    current_geo_point.latitude = (float) std::stod ( latitude_stringstream.str() , &sz);

                    std::stringstream longitude_stringstream;
                    longitude_stringstream << no_fly_zones_geo[i][j][1];
                    current_geo_point.longitude = (float) std::stod ( longitude_stringstream.str() , &sz);

                    geometry_msgs::Point32 current_cartesian_point = geographic_to_cartesian(current_geo_point, map_origin_geo_);
                    current_cartesian_point.z = 0;
                    current_polygon_obstacle.points.push_back(current_cartesian_point);
                }
                no_fly_zones_.push_back(current_polygon_obstacle);
            }
        }
    }

    // Read the geofence parameter:
    XmlRpc::XmlRpcValue geofence;
    n_.getParam("geofence", geofence);
    if (geofence.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i=0; i<geofence.size(); i++) {
            if (geofence[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                geographic_msgs::GeoPoint current_geo_point;
                std::string::size_type sz;

                std::stringstream latitude_stringstream;
                latitude_stringstream << geofence[i][0];
                current_geo_point.latitude = (float) std::stod ( latitude_stringstream.str() , &sz);

                std::stringstream longitude_stringstream;
                longitude_stringstream << geofence[i][1];
                current_geo_point.longitude = (float) std::stod ( longitude_stringstream.str() , &sz);

                geometry_msgs::Point32 current_cartesian_point = geographic_to_cartesian(current_geo_point, map_origin_geo_);
                current_cartesian_point.z = 0;
                geofence_.points.push_back(current_cartesian_point);
            }
        }
    }

    if (geofence_.points.size()>0 && no_fly_zones_.size()>0) {
        path_planner_ = grvc::PathPlanner(no_fly_zones_, geofence_);
    }

#ifdef DEBUG
    for (int i=0; i<geofence_.points.size(); i++) {
        std::cout << "geofence_.points[ " << i << " ].x = " << geofence_.points[i].x << std::endl;
        std::cout << "geofence_.points[ " << i << " ].y = " << geofence_.points[i].y << std::endl;
    }
    for (int i=0; i<no_fly_zones_.size(); i++) {
        for (int j=0; j<no_fly_zones_[i].points.size(); j++) {
            std::cout << "no_fly_zones_[ " << i << " ].points[ " << j << " ].x = " << no_fly_zones_[i].points[j].x << std::endl;
            std::cout << "no_fly_zones_[ " << i << " ].points[ " << j << " ].y = " << no_fly_zones_[i].points[j].y << std::endl;
        }
    }
#endif

    // Read parameters of the complete graph (from the yaml). Numeric parameters extracted from a string, so some steps are needed:
    XmlRpc::XmlRpcValue pylons_position_cartesian;          n_.getParam("pylons_position_cartesian",         pylons_position_cartesian);
    XmlRpc::XmlRpcValue pylons_position_geographic;         n_.getParam("pylons_position_geographic",        pylons_position_geographic);
    XmlRpc::XmlRpcValue pylons_connections_indexes;         n_.getParam("pylons_connections_indexes",        pylons_connections_indexes);
    XmlRpc::XmlRpcValue recharge_land_stations_cartesian;   n_.getParam("recharge_land_stations_cartesian",  recharge_land_stations_cartesian);
    XmlRpc::XmlRpcValue recharge_land_stations_geographic;  n_.getParam("recharge_land_stations_geographic", recharge_land_stations_geographic);
    XmlRpc::XmlRpcValue regular_land_stations_cartesian;    n_.getParam("regular_land_stations_cartesian",   regular_land_stations_cartesian);
    XmlRpc::XmlRpcValue regular_land_stations_geographic;   n_.getParam("regular_land_stations_geographic",  regular_land_stations_geographic);

    aerialcore_msgs::GraphNode current_graph_node;

    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_PYLON;
    if (pylons_position_cartesian.getType()==XmlRpc::XmlRpcValue::TypeArray && pylons_position_geographic.getType()==XmlRpc::XmlRpcValue::TypeArray && pylons_connections_indexes.getType()==XmlRpc::XmlRpcValue::TypeArray && pylons_position_cartesian.size()==pylons_position_geographic.size() && pylons_connections_indexes.size()==pylons_position_geographic.size()) {
        for (int i=0; i<pylons_position_cartesian.size(); i++) {
            if (pylons_position_cartesian[i].getType()==XmlRpc::XmlRpcValue::TypeArray && pylons_position_geographic[i].getType()==XmlRpc::XmlRpcValue::TypeArray) {
                std::string::size_type sz;

                std::stringstream x_stringstream;
                x_stringstream << pylons_position_cartesian[i][0];
                current_graph_node.x = (float) std::stod ( x_stringstream.str() , &sz);

                std::stringstream y_stringstream;
                y_stringstream << pylons_position_cartesian[i][1];
                current_graph_node.y = (float) std::stod ( y_stringstream.str() , &sz);

                std::stringstream z_stringstream;
                z_stringstream << pylons_position_cartesian[i][2];
                current_graph_node.z = (float) std::stod ( z_stringstream.str() , &sz);

                std::stringstream latitude_stringstream;
                latitude_stringstream << pylons_position_geographic[i][0];
                current_graph_node.latitude = (float) std::stod ( latitude_stringstream.str() , &sz);

                std::stringstream longitude_stringstream;
                longitude_stringstream << pylons_position_geographic[i][1];
                current_graph_node.longitude = (float) std::stod ( longitude_stringstream.str() , &sz);

                std::stringstream altitude_stringstream;
                altitude_stringstream << pylons_position_geographic[i][2];
                current_graph_node.altitude = (float) std::stod ( altitude_stringstream.str() , &sz);

                // NOTE: parsing with XmlRpc is tricky because it fails to parse numbers with and without comma in the same list.
                // That's why stringstream is being used to parse, because it helps to pass it to string, and then it is parsed to float, not caring if it has .0 or not at the end of the number.
                // For the connections_indexes, as it's always a list of ints, it's parsed directly.

                current_graph_node.connections_indexes.clear();
                for (int j=0; j<pylons_connections_indexes[i].size(); j++) {
                    current_graph_node.connections_indexes.push_back((int) pylons_connections_indexes[i][j]-1);
                }

                complete_graph_.push_back(current_graph_node);
            }
        }
    }

    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION;
    current_graph_node.connections_indexes.clear();
    if (recharge_land_stations_cartesian.getType()==XmlRpc::XmlRpcValue::TypeArray && recharge_land_stations_geographic.getType()==XmlRpc::XmlRpcValue::TypeArray && recharge_land_stations_cartesian.size()==recharge_land_stations_geographic.size()) {
        for (int i=0; i<recharge_land_stations_cartesian.size(); i++) {
            if (recharge_land_stations_cartesian[i].getType()==XmlRpc::XmlRpcValue::TypeArray && recharge_land_stations_geographic[i].getType()==XmlRpc::XmlRpcValue::TypeArray) {
                std::string::size_type sz;

                std::stringstream x_stringstream;
                x_stringstream << recharge_land_stations_cartesian[i][0];
                current_graph_node.x = (float) std::stod ( x_stringstream.str() , &sz);

                std::stringstream y_stringstream;
                y_stringstream << recharge_land_stations_cartesian[i][1];
                current_graph_node.y = (float) std::stod ( y_stringstream.str() , &sz);

                std::stringstream z_stringstream;
                z_stringstream << recharge_land_stations_cartesian[i][2];
                current_graph_node.z = (float) std::stod ( z_stringstream.str() , &sz);

                std::stringstream latitude_stringstream;
                latitude_stringstream << recharge_land_stations_geographic[i][0];
                current_graph_node.latitude = (float) std::stod ( latitude_stringstream.str() , &sz);

                std::stringstream longitude_stringstream;
                longitude_stringstream << recharge_land_stations_geographic[i][1];
                current_graph_node.longitude = (float) std::stod ( longitude_stringstream.str() , &sz);

                std::stringstream altitude_stringstream;
                altitude_stringstream << recharge_land_stations_geographic[i][2];
                current_graph_node.altitude = (float) std::stod ( altitude_stringstream.str() , &sz);

                complete_graph_.push_back(current_graph_node);
            }
        }
    }

    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION;
    if (regular_land_stations_cartesian.getType()==XmlRpc::XmlRpcValue::TypeArray && regular_land_stations_geographic.getType()==XmlRpc::XmlRpcValue::TypeArray && regular_land_stations_cartesian.size()==regular_land_stations_geographic.size()) {
        for (int i=0; i<regular_land_stations_cartesian.size(); i++) {
            if (regular_land_stations_cartesian[i].getType()==XmlRpc::XmlRpcValue::TypeArray && regular_land_stations_geographic[i].getType()==XmlRpc::XmlRpcValue::TypeArray) {
                std::string::size_type sz;

                std::stringstream x_stringstream;
                x_stringstream << regular_land_stations_cartesian[i][0];
                current_graph_node.x = (float) std::stod ( x_stringstream.str() , &sz);

                std::stringstream y_stringstream;
                y_stringstream << regular_land_stations_cartesian[i][1];
                current_graph_node.y = (float) std::stod ( y_stringstream.str() , &sz);

                std::stringstream z_stringstream;
                z_stringstream << regular_land_stations_cartesian[i][2];
                current_graph_node.z = (float) std::stod ( z_stringstream.str() , &sz);

                std::stringstream latitude_stringstream;
                latitude_stringstream << regular_land_stations_geographic[i][0];
                current_graph_node.latitude = (float) std::stod ( latitude_stringstream.str() , &sz);

                std::stringstream longitude_stringstream;
                longitude_stringstream << regular_land_stations_geographic[i][1];
                current_graph_node.longitude = (float) std::stod ( longitude_stringstream.str() , &sz);

                std::stringstream altitude_stringstream;
                altitude_stringstream << regular_land_stations_geographic[i][2];
                current_graph_node.altitude = (float) std::stod ( altitude_stringstream.str() , &sz);

                complete_graph_.push_back(current_graph_node);
            }
        }
    }

    current_graph_mutex_.lock();
    current_graph_ = complete_graph_;
    removeGraphNodesAndEdgesAboveNoFlyZones(current_graph_);
    complete_graph_cleaned_ = current_graph_;
    current_graph_mutex_.unlock();

#ifdef DEBUG
    printCurrentGraph();
#endif

    // Advertised services:
    start_supervising_srv_ = n_.advertiseService("mission_controller/start_supervising", &MissionController::startSupervisingServiceCallback, this);
    stop_supervising_srv_ = n_.advertiseService("mission_controller/stop_supervising", &MissionController::stopSupervisingServiceCallback, this);
    do_complete_supervision_srv_ = n_.advertiseService("mission_controller/do_complete_supervision", &MissionController::doCompleteSupervisionServiceCallback, this);
    do_specific_supervision_srv_ = n_.advertiseService("mission_controller/do_specific_supervision", &MissionController::doSpecificSupervisionServiceCallback, this);
    do_continuous_supervision_srv_ = n_.advertiseService("mission_controller/do_continuous_supervision", &MissionController::doContinuousSupervisionServiceCallback, this);
    do_fast_supervision_srv_ = n_.advertiseService("mission_controller/do_fast_supervision", &MissionController::doFastSupervisionServiceCallback, this);
    start_specific_supervision_plan_srv_ = n_.advertiseService("mission_controller/start_specific_supervision_plan", &MissionController::startSpecificSupervisionPlanServiceCallback, this);
    trigger_replanning_manually_srv_ = n_.advertiseService("mission_controller/trigger_replanning_manually", &MissionController::triggerReplanningManuallyServiceCallback, this);

    // Clients:
    post_yaml_client_ = n_.serviceClient<aerialcore_msgs::PostString>("post_yaml");
#ifdef USING_MS_TSP_PLANNER
    ms_tsp_planner_ = n_.serviceClient<ms_tsp_planner::ConfigToFlightPlans>("ms_tsp_planner/config_to_flight_plan");
#endif

    // Make communications spin!
    spin_thread_ = std::thread([this]() {
        ros::MultiThreadedSpinner spinner(2); // Use 2 threads
        spinner.spin();
    });

    // Start the battery faker thread if simulation_==true and we have continuous battery information with mission_lib (TODO: can DJI SDK also provide it?):
    if (simulation_ && UAV_command_mode_=="mission_lib") {
        battery_faker_thread_ = std::thread(&MissionController::batteryFakerThread, this);
    }

    ROS_INFO("Mission Controller: running! Waiting for commands.");

} // end MissionController constructor


// Brief Destructor
MissionController::~MissionController() {
    if(spin_thread_.joinable()) spin_thread_.join();
    if(battery_faker_thread_.joinable()) battery_faker_thread_.join();

    stop_current_supervising_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if(parameter_estimator_thread_.joinable()) parameter_estimator_thread_.join();
    if(plan_thread_.joinable()) plan_thread_.join();
    stop_current_supervising_ = false;

    for (UAV& uav : UAVs_) {
        delete uav.mission;
    }
} // end ~MissionController destructor


bool MissionController::startSupervisingServiceCallback(aerialcore_msgs::StartSupervising::Request& _req, aerialcore_msgs::StartSupervising::Response& _res) {
    if (_req.uav_id.size()>0) {     // There are specific UAVs to start.
        // First disable all UAVs from supervising to then enable only the ones used:
        for (UAV& current_uav : UAVs_) {
            current_uav.enabled_to_supervise = false;
        }

        for (const int& current_uav_id : _req.uav_id) {
            int current_uav_index = findUavIndexById(current_uav_id);
            if (current_uav_index == -1) continue;  // If UAV id not found just skip it.

            UAVs_[current_uav_index].enabled_to_supervise = true;
        }
    } else {    // If empty start all UAVs.
        for (UAV& current_uav : UAVs_) {
            current_uav.enabled_to_supervise = true;
        }
    }

    // Stop the current supervising threads, consisting on the parameter estimator (module of same name) and plan thread (plan monitor and planner modules):
    stop_current_supervising_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if(parameter_estimator_thread_.joinable()) parameter_estimator_thread_.join();
    if(plan_thread_.joinable()) plan_thread_.join();
    stop_current_supervising_ = false;

    // Start the supervising threads, consisting on the parameter estimator (module of same name) and plan thread (plan monitor and planner modules):
    parameter_estimator_thread_ = std::thread(&MissionController::parameterEstimatorThread, this);
    plan_thread_ = std::thread(&MissionController::planThread, this);

    _res.success=true;
    return true;
} // end startSupervisingServiceCallback


// Parameter Estimator thread:
void MissionController::parameterEstimatorThread(void) {
    std::vector<aerialcore_msgs::GraphNode> estimator_current_graph;

    ros::Rate loop_rate(1.0/parameter_estimator_time_); // [Hz, inverse of seconds]
    while (!stop_current_supervising_) {
        current_graph_mutex_.lock();
        estimator_current_graph = current_graph_;
        current_graph_mutex_.unlock();

        update_matrices_mutex_.lock();
        parameter_estimator_.updateMatrices(estimator_current_graph, no_fly_zones_, geofence_, false, true);  // Don't update here the UAVs initial costs, update the wind.
        update_matrices_mutex_.unlock();

        loop_rate.sleep();
    }
} // end parameterEstimatorThread


// Plan thread:
void MissionController::planThread(void) {

    std::vector<aerialcore_msgs::GraphNode> planner_current_graph;

    centralized_planner_.resetInspectedEdges();

    std::map<int, float> battery_level_when_planned;

    ros::Rate loop_rate(1.0/plan_monitor_time_);        // [Hz, inverse of seconds]
    while (!stop_current_supervising_) {

        // Reset the next current graph:
        if (complete_or_specific_graph_supervision_) {
            planner_current_graph = complete_graph_cleaned_;
        } else {
            planner_current_graph = specific_subgraph_cleaned_;
        }

        bool plan_from_scratch = flight_plans_.size()==0;

        std::vector< std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> > drone_info;
        for (const UAV& current_uav : UAVs_) {
            if (current_uav.enabled_to_supervise == true) {
                std::tuple<float, float, float, int, int, int, int, int, int, bool, bool> new_tuple;

                if (UAV_command_mode_=="mission_lib") {
                    if (simulation_) {
                        std::get<0>(new_tuple) = current_uav.battery_faked;
                    } else {
                        std::get<0>(new_tuple) = current_uav.mission->battery();
                    }
                } else if (UAV_command_mode_=="DJI_SDK_yaml" || UAV_command_mode_=="USE_CTU_yaml") {
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/battery", std::get<0>(new_tuple));    // TODO: should read this from the SDK or other place.
                }

                if (plan_from_scratch) {
                    battery_level_when_planned[current_uav.id] = std::get<0>(new_tuple);
                }
                std::get<1>(new_tuple) = battery_level_when_planned[current_uav.id];

                std::get<2>(new_tuple) = current_uav.minimum_battery;
                std::get<3>(new_tuple) = current_uav.time_until_fully_charged;
                std::get<4>(new_tuple) = current_uav.time_max_flying;
                std::get<5>(new_tuple) = current_uav.speed_xy;
                std::get<6>(new_tuple) = current_uav.speed_z_down;
                std::get<7>(new_tuple) = current_uav.speed_z_up;
                std::get<8>(new_tuple) = current_uav.id;
                std::get<9>(new_tuple) = UAV_command_mode_=="mission_lib" ? current_uav.mission->armed() : false;  // TODO: better way to know if flying?
                std::get<10>(new_tuple) = current_uav.recharging;
                drone_info.push_back(new_tuple);

                aerialcore_msgs::GraphNode initial_position_graph_node;
                initial_position_graph_node.type = aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION;
                initial_position_graph_node.id = current_uav.id;
                if (UAV_command_mode_=="mission_lib") {

                    geometry_msgs::PoseStamped uav_pose = current_uav.mission->pose();

                    initial_position_graph_node.x = uav_pose.pose.position.x;
                    initial_position_graph_node.y = uav_pose.pose.position.y;
                    initial_position_graph_node.z = uav_pose.pose.position.z;

                    geometry_msgs::Point32 aux_point32;
                    aux_point32.x = initial_position_graph_node.x;
                    aux_point32.y = initial_position_graph_node.y;
                    aux_point32.z = initial_position_graph_node.z;
                    geographic_msgs::GeoPoint current_geopoint = cartesian_to_geographic(aux_point32, map_origin_geo_);

                    initial_position_graph_node.latitude  = current_geopoint.latitude;
                    initial_position_graph_node.longitude = current_geopoint.longitude;
                    initial_position_graph_node.altitude  = current_geopoint.altitude;
                } else if (UAV_command_mode_=="DJI_SDK_yaml" || UAV_command_mode_=="USE_CTU_yaml") {
                    // TODO: should read these from the SDK or other place.
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/x", initial_position_graph_node.x);
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/y", initial_position_graph_node.y);
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/z", initial_position_graph_node.z);
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/latitude", initial_position_graph_node.latitude);
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/longitude", initial_position_graph_node.longitude);
                    n_.getParam(ns_uav_prefix_+std::to_string( current_uav.id )+"/altitude", initial_position_graph_node.altitude);
                }

                planner_current_graph.push_back(initial_position_graph_node);

                current_graph_mutex_.lock();
                current_graph_ = planner_current_graph;
                current_graph_mutex_.unlock();

            }
        }

        bool replan = false;

        if (plan_from_scratch) {
            // For the first time, force the parameter_estimator_ calculate the matrices with new costs of the initial UAVs:
            update_matrices_mutex_.lock();
            parameter_estimator_.updateMatrices(planner_current_graph, no_fly_zones_, geofence_, true, false);
            update_matrices_mutex_.unlock();
        } else {
            if (trigger_replanning_manually_) {
                trigger_replanning_manually_ = false;
                replan = true;
                plan_monitor_.enoughDeviationToReplan(planner_current_graph, flight_plans_, drone_info, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices());
            } else {
                replan = replanning_enabled_ ? plan_monitor_.enoughDeviationToReplan(planner_current_graph, flight_plans_, drone_info, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices()) : false;
            }
        }

        if (replan || plan_from_scratch) {

// #ifdef DEBUG
            printCurrentGraph();
// #endif

#ifdef NUMERICAL_EXPERIMENTS
            std::vector<std::string> methods{"Greedy"/*, "MILP"*/, "MEM", "VNS"/*, "Mstsp"*/, "Minimax-MEM", "Minimax-VNS", "Minimax-Greedy"};
            for (int method=0; method<methods.size(); method++) {

            planner_method_ = methods[method];
            planner_method_electric_fault_ = methods[method];
            if (method<methods.size()-2) {                      // IF NOT MINIMAX
                continuous_or_fast_supervision_ = true;
            } else {
                continuous_or_fast_supervision_ = false;
            }

            for (int it_exp=1; it_exp<=30; it_exp++) {
            centralized_planner_.resetInspectedEdges();
#endif

            // Force the parameter_estimator_ calculate the matrices with new costs of the initial UAVs only if replanning:
            if (replan) {
                update_matrices_mutex_.lock();
                parameter_estimator_.updateMatrices(planner_current_graph, no_fly_zones_, geofence_, true, false);
                update_matrices_mutex_.unlock();
            }

            // Calculate the solution with computation time:

            ROS_INFO("Mission Controller: start the computation of the plan calling the solver.");
            clock_t t_begin, t_end;
            t_begin = clock();

            std::vector<aerialcore_msgs::FlightPlan> flight_plans_new;
            if (continuous_or_fast_supervision_) {
                if (planner_method_ == "Greedy") {
                    flight_plans_new = centralized_planner_.getPlanGreedy(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
                } else if (planner_method_ == "MILP") {
#ifdef USING_ORTOOLS
                    flight_plans_new =  centralized_planner_.getPlanMILP(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
#else
                    ROS_ERROR("Mission Controller: MILP selected but OR-Tools not installed or configured in the cmakelist.");
#endif
                } else if (planner_method_ == "VNS") {
                    flight_plans_new =  centralized_planner_.getPlanVNS(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
                } else if (planner_method_ == "MEM") {
                    flight_plans_new = centralized_planner_.getPlanMEM(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
                } else if (planner_method_ == "Mstsp") {
#ifdef USING_MS_TSP_PLANNER
                    ms_tsp_planner::ConfigToFlightPlans config_to_flight_plans;
                    config_to_flight_plans.request.configFile = "default.yaml";
                    if (ms_tsp_planner_.call(config_to_flight_plans)) {
                        ROS_INFO("Mission Controller: plan from ms_tsp_planner received.");
                        flight_plans_new = config_to_flight_plans.response.plans;

                        // TODO? this for loop is a patch because mstsp.solution_to_flight_plans(sol) don't fill the uav_id field:
                        for (int i=0; i<flight_plans_new.size(); i++) {
                            flight_plans_new[i].uav_id = i+1;
                        }

                        for (int i=0; i<flight_plans_new.size(); i++) {
                            for (int j=0; j<flight_plans_new[i].nodes.size(); j++) {
                                if (planner_current_graph[ flight_plans_new[i].nodes[j] ].z <= FLYING_THRESHOLD) {
                                    planner_current_graph[ flight_plans_new[i].nodes[j] ].z += 30;
                                    flight_plans_new[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP);
                                } else if ( j==flight_plans_new[i].nodes.size()-1 ) {
                                    flight_plans_new[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_LAND_WP);
                                } else {
                                    flight_plans_new[i].type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP);
                                }

                                geometry_msgs::PoseStamped current_pose;
                                current_pose.pose.position.x = planner_current_graph[ flight_plans_new[i].nodes[j] ].x;
                                current_pose.pose.position.y = planner_current_graph[ flight_plans_new[i].nodes[j] ].y;
                                current_pose.pose.position.z = planner_current_graph[ flight_plans_new[i].nodes[j] ].z;
                                flight_plans_new[i].poses.push_back(current_pose);
                            }
                        }

                    } else {
                        ROS_ERROR("Mission Controller: ms_tsp_planner service didn't answer.");
                    }

#else
                ROS_ERROR("Mission Controller: ms_tsp_planner selected but not installed or configured in the cmakelist.");
#endif
                }
            } else {
                if (planner_method_electric_fault_ == "Minimax-Greedy") {
                    flight_plans_new = centralized_planner_.getPlanMinimaxGreedy(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
                } else if (planner_method_electric_fault_ == "Minimax-MEM") {
                    flight_plans_new = centralized_planner_.getPlanMinimaxMEM(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
                } else if (planner_method_electric_fault_ == "Minimax-VNS") {
                    flight_plans_new =  centralized_planner_.getPlanMinimaxVNS(planner_current_graph, drone_info, no_fly_zones_, geofence_, parameter_estimator_.getTimeCostMatrices(), parameter_estimator_.getBatteryDropMatrices(), plan_monitor_.getLastGraphNodes());
                }
            }

            // Before saving the new plan, check if there are drones not used in this new plan, in which case clear its plan:
            for (const aerialcore_msgs::FlightPlan& flight_plan_original : flight_plans_) {
                bool uav_match_found = false;
                for (const aerialcore_msgs::FlightPlan& flight_plan_new : flight_plans_new) {
                    if (flight_plan_original.uav_id == flight_plan_new.uav_id) {
                        uav_match_found = true;
                        break;
                    }
                }
                if (!uav_match_found) {
                    UAVs_[ findUavIndexById(flight_plan_original.uav_id) ].mission->stop();
                    UAVs_[ findUavIndexById(flight_plan_original.uav_id) ].mission->clear();
                    UAVs_[ findUavIndexById(flight_plan_original.uav_id) ].mission->pushClear();
                }
            }
            std::sort(flight_plans_new.begin(), flight_plans_new.end(), compareFlightPlanById);
            flight_plans_ = flight_plans_new;

            t_end = clock();

            double seconds = ((float)(t_end-t_begin))/CLOCKS_PER_SEC;
            // std::cout << "Computation time for the MILP planning with OR-Tools: " << seconds << " seconds." << std::endl << std::endl;
            ROS_INFO("Mission Controller: computation time of the planning: %f seconds.", seconds);

// #ifdef DEBUG
            if (continuous_or_fast_supervision_ && planner_method_ == "Mstsp") {
                std::cout << "Planner's results calculated in the MC:" << std::endl;
                float total_time_cost = 0;
                float total_battery_cost = 0;
                std::map<int, std::map<int, std::map<int, float> > > time_cost_matrices = parameter_estimator_.getTimeCostMatrices();
                std::map<int, std::map<int, std::map<int, float> > > battery_drop_matrices = parameter_estimator_.getBatteryDropMatrices();
                for (int i=0; i<flight_plans_.size(); i++) {
                    std::cout << "flight_plans[ " << i << " ].uav_id       = " << flight_plans_[i].uav_id << std::endl;
                    float time_cost = 0;
                    float battery_cost = 0;
                    for (int j=0; j<flight_plans_[i].nodes.size()-1; j++) {
                        if (time_cost_matrices[flight_plans_[i].uav_id][ flight_plans_[i].nodes[j] ][ flight_plans_[i].nodes[j+1] ] != -1) {
                            time_cost += time_cost_matrices[flight_plans_[i].uav_id][ flight_plans_[i].nodes[j] ][ flight_plans_[i].nodes[j+1] ];
                            battery_cost += battery_drop_matrices[flight_plans_[i].uav_id][ flight_plans_[i].nodes[j] ][ flight_plans_[i].nodes[j+1] ];
                        }
                    }
                    std::cout << "flight_plans[ " << i << " ].time_cost    = " << time_cost << std::endl;
                    std::cout << "flight_plans[ " << i << " ].battery_cost = " << battery_cost << std::endl;
                    total_time_cost += time_cost;
                    total_battery_cost += battery_cost;
                }
                std::cout << "Total plan time cost:    " << total_time_cost << std::endl;
                std::cout << "Total plan battery cost: " << total_battery_cost << std::endl;
                std::cout << "UAVs involved:           " << flight_plans_.size() << std::endl << std::endl;
            } else {
                std::cout << "printPlan() called from the MC:" << std::endl;
                centralized_planner_.printPlan();
            }

#ifdef NUMERICAL_EXPERIMENTS
            // % of inspection edges covered:
            auto distance_cost_matrix = parameter_estimator_.getDistanceCostMatrix();
            float total_distance_to_inspect = 0;
            std::vector< std::pair<int,int> > connection_edges;    // Edges or connections of the graph, being the pair of indexes of the graph nodes connected by wires. Always the first int index lower than the second of the pair.
            for (int i=0; i<planner_current_graph.size(); i++) {   // Construct connection_edges from scratch:
                if (planner_current_graph[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                    for (int j=0; j<planner_current_graph[i].connections_indexes.size(); j++) {
                        if (i < planner_current_graph[i].connections_indexes[j]) {
                            std::pair <int,int> current_edge (i, planner_current_graph[i].connections_indexes[j]);
                            total_distance_to_inspect += distance_cost_matrix[ current_edge.first ][ current_edge.second ];
                            connection_edges.push_back(current_edge);
                        }
                    }
                }
            }
            for (int i=0; i<flight_plans_.size(); i++) {    // Erase from connection_edges the one covered by the plan:
                for (int j=0; j<flight_plans_[i].nodes.size()-1; j++) {
                    if (planner_current_graph[ flight_plans_[i].nodes[j] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON && planner_current_graph[ flight_plans_[i].nodes[j+1] ].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
                        std::pair <int,int> current_edge;
                        if (flight_plans_[i].nodes[j] < flight_plans_[i].nodes[j+1]) {
                            current_edge.first  = flight_plans_[i].nodes[j];
                            current_edge.second = flight_plans_[i].nodes[j+1];
                        } else {
                            current_edge.first  = flight_plans_[i].nodes[j+1];
                            current_edge.second = flight_plans_[i].nodes[j];
                        }
                        for (int k=0; k<connection_edges.size(); k++) {
                            if (connection_edges[k] == current_edge) {
                                connection_edges.erase(connection_edges.begin() + k);
                                break;
                            }
                        }
                    }
                }
            }
            float distance_not_covered = 0;
            for (int k=0; k<connection_edges.size(); k++) {
                distance_not_covered += distance_cost_matrix[ connection_edges[k].first ][ connection_edges[k].second ];
            }

            std::ofstream file_out("experimental_results.txt", std::ios_base::app);
            file_out /*<< it_exp*/ /* Number of iteration of the numerical experiment */ << " " << planner_method_ << " " << seconds /* computation time */ << " " << total_time_cost << " " << total_battery_cost << " " << flight_plans_.size() /* Nº UAVs in the solution*/ << " " << UAVs_.size() /* UAVs available */ << " " << distance_not_covered << " " << total_distance_to_inspect << std::endl;
            file_out.close();
#endif

#ifdef NUMERICAL_EXPERIMENTS
            }
            }
#endif

// #endif

            if (UAV_command_mode_=="mission_lib") {
                translateFlightPlanIntoUAVMission(flight_plans_);

                for (const aerialcore_msgs::FlightPlan& flight_plan : flight_plans_) {

                    int current_uav_index = findUavIndexById(flight_plan.uav_id);
                    battery_level_when_planned[ flight_plan.uav_id ] = simulation_ ? UAVs_[current_uav_index].battery_faked : UAVs_[current_uav_index].mission->battery();

                    if (current_uav_index == -1) continue;  // If UAV id not found just skip it.
// #ifdef DEBUG
                    UAVs_[current_uav_index].mission->print();
// #endif
                    UAVs_[current_uav_index].mission->push();
                    UAVs_[current_uav_index].mission->start();
                }
            } else if (UAV_command_mode_=="DJI_SDK_yaml") {
                aerialcore_msgs::PostString post_yaml_string;
                post_yaml_string.request.data = translateFlightPlanIntoDJIyaml(flight_plans_);

                std::ofstream file_out("plan.yaml");
                file_out << post_yaml_string.request.data << std::endl;
                file_out.close();

                if (post_yaml_client_.call(post_yaml_string)) {
                    ROS_INFO("Mission Controller: yaml sent to DJI SDK.");
                } else {
                    ROS_WARN("Mission Controller: yaml not sent to DJI SDK, service didn't answer.");
                }
            } else if (UAV_command_mode_=="USE_CTU_yaml") {
                aerialcore_msgs::PostString post_yaml_string;
                post_yaml_string.request.data = translateFlightPlanIntoStdYaml(flight_plans_);

                std::ofstream file_out("plan.yaml");
                file_out << post_yaml_string.request.data << std::endl;
                file_out.close();

                if (post_yaml_client_.call(post_yaml_string)) {
                    ROS_INFO("Mission Controller: standard yaml sent.");
                } else {
                    ROS_WARN("Mission Controller: standard yaml not sent, service didn't answer.");
                }
            }

            // // After a replanning wait for 10x the plan_monitor_time_ before checking again if it needs replanning:
            // std::this_thread::sleep_for(std::chrono::seconds((int) plan_monitor_time_*10));
            // continue;
        }

        loop_rate.sleep();
    }
} // end planThread


void MissionController::translateFlightPlanIntoUAVMission(std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {
    float takeoff_delay = 0;    // Only used if several UAVs and continous supervision.
    auto time_cost_matrices = parameter_estimator_.getTimeCostMatrices();

    for (aerialcore_msgs::FlightPlan& flight_plan : _flight_plans) {
        int current_uav_index = findUavIndexById(flight_plan.uav_id);
        if (current_uav_index == -1) continue;  // If UAV id not found just skip it.

        UAVs_[current_uav_index].mission->clear();

        std::vector<geometry_msgs::PoseStamped> pass_poses;

        if (flight_plan.poses.size() != flight_plan.type.size()) {
            ROS_ERROR("Mission Controller: UAV id=%d has a faulty FlightPlan which poses and type vectors have different sizes.", flight_plan.uav_id);
        }

        bool only_sum_delay_once = true;

        for (int i=0; i<flight_plan.poses.size(); i++) {
            geometry_msgs::PoseStamped current_pose_stamped = flight_plan.poses[i];

            if (flight_plan.type[i] == aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP
             || flight_plan.type[i] == aerialcore_msgs::FlightPlan::TYPE_PASS_NFZ_WP) {
                pass_poses.push_back(current_pose_stamped);

            } else {
                if (pass_poses.size()>0) {
                    UAVs_[current_uav_index].mission->addPassWpList(pass_poses);
                }

                if (flight_plan.type[i] == aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP) {
                    if (planner_method_ == "Mstsp") { current_pose_stamped.pose.position.z += 30; } // TODO: harcoded takeoff height.
                    if (i==0) {     // Only delay the first takeoff.
                        UAVs_[current_uav_index].mission->addTakeOffWp(current_pose_stamped, takeoff_delay);
                        flight_plan.header.stamp.sec += takeoff_delay;
                    } else {
                        UAVs_[current_uav_index].mission->addTakeOffWp(current_pose_stamped, 0);
                    }


                } else if (flight_plan.type[i] == aerialcore_msgs::FlightPlan::TYPE_LAND_WP) {
                    if (UAVs_[current_uav_index].mission->airframeType() == grvc::AirframeType::FIXED_WING) {

                        // PX4 gives error "adjust landing approach" if the last wp previous to the landing isn't appropriate. So calculate a previous wp to the landing that is at a distance of 200 meters minimum (500 prefered) of the landing spot and 15 meters height.
                        // TODO: maybe insert this point in the planning to take it into account during the planning. Land pose delayed.
                        geometry_msgs::PoseStamped wp_previous_to_landing_1, wp_previous_to_landing_2;

                        float runway_heading;
                        n_.param<float>("runway_heading", runway_heading, 0);

                        wp_previous_to_landing_1.pose.position.x = current_pose_stamped.pose.position.x + cos(runway_heading * M_PI/180) * 500;
                        wp_previous_to_landing_1.pose.position.y = current_pose_stamped.pose.position.y + sin(runway_heading * M_PI/180) * 500;
                        wp_previous_to_landing_1.pose.position.z = current_pose_stamped.pose.position.z + 15;

                        wp_previous_to_landing_2.pose.position.x = current_pose_stamped.pose.position.x - cos(runway_heading * M_PI/180) * 500;
                        wp_previous_to_landing_2.pose.position.y = current_pose_stamped.pose.position.y - sin(runway_heading * M_PI/180) * 500;
                        wp_previous_to_landing_2.pose.position.z = current_pose_stamped.pose.position.z + 15;

                        if ( sqrt( pow(wp_previous_to_landing_1.pose.position.x - pass_poses.back().pose.position.x,2) + pow(wp_previous_to_landing_1.pose.position.y - pass_poses.back().pose.position.y,2) ) < sqrt( pow(wp_previous_to_landing_2.pose.position.x - pass_poses.back().pose.position.x,2) + pow(wp_previous_to_landing_2.pose.position.y - pass_poses.back().pose.position.y,2) ) ) {
                            float langing_yaw = atan2(wp_previous_to_landing_1.pose.position.y - current_pose_stamped.pose.position.y, wp_previous_to_landing_1.pose.position.x - current_pose_stamped.pose.position.x);

                            UAVs_[current_uav_index].mission->addLandWp(wp_previous_to_landing_1, current_pose_stamped, langing_yaw);
                        } else {
                            float langing_yaw = atan2(wp_previous_to_landing_2.pose.position.y - current_pose_stamped.pose.position.y, wp_previous_to_landing_1.pose.position.x - current_pose_stamped.pose.position.x);

                            UAVs_[current_uav_index].mission->addLandWp(wp_previous_to_landing_2, current_pose_stamped, langing_yaw);
                        }

                    } else {
                        UAVs_[current_uav_index].mission->addLandWp(current_pose_stamped);
                    }
                }

                pass_poses.clear();
            }

            if (i==0 && continuous_or_fast_supervision_ && only_sum_delay_once) {  // Delay the takeoff of the next UAV if the inspection mode is set to continuous, which is not fast (one UAV at the time).
                for (int j=0; j<flight_plan.nodes.size()-1; j++) {
                    try {
                        takeoff_delay += time_cost_matrices.at( flight_plan.uav_id ).at( flight_plan.nodes[j] ).at( flight_plan.nodes[j+1] );
                    } catch (...) { // catch any exception. Mainly the problem is time_cost_matrices not built yet when using a specific supervision plan or Mstsp.
                        try {
                            update_matrices_mutex_.lock();
                            parameter_estimator_.updateMatrices(current_graph_, no_fly_zones_, geofence_, true, false);
                            time_cost_matrices = parameter_estimator_.getTimeCostMatrices();
                            update_matrices_mutex_.unlock();

                            takeoff_delay += time_cost_matrices.at( flight_plan.uav_id ).at( flight_plan.nodes[j] ).at( flight_plan.nodes[j+1] );
                        } catch (...) { // catch any exception. Mainly the problem is time_cost_matrices not built yet when using Mstsp.
                            takeoff_delay += 0;
                        }
                    }
                }
                only_sum_delay_once = false;
            }
        }
    }
} // end translateFlightPlanIntoUAVMission


std::string MissionController::translateFlightPlanIntoDJIyaml(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {
    std::string yaml_to_return = {R"(frame_id: /gps

uav_n: )"};
    yaml_to_return.append(std::to_string(_flight_plans.size()).c_str());
    yaml_to_return.append("\n\n");
    for (const aerialcore_msgs::FlightPlan& flight_plan : _flight_plans) {
        yaml_to_return.append("uav_");
        yaml_to_return.append(std::to_string(flight_plan.uav_id).c_str());
        yaml_to_return.append({R"(:
  wp_n: )"});
        yaml_to_return.append(std::to_string(flight_plan.poses.size()-2 ).c_str());
        for (int i=1; i<flight_plan.poses.size()-1; i++) {
            geometry_msgs::Point32 aux_point32;
            aux_point32.x = flight_plan.poses[i].pose.position.x;
            aux_point32.y = flight_plan.poses[i].pose.position.y;
            aux_point32.z = flight_plan.poses[i].pose.position.z;

            geographic_msgs::GeoPoint current_geopoint = cartesian_to_geographic(aux_point32, map_origin_geo_);
            current_geopoint.altitude = flight_plan.poses[i].pose.position.z;

            yaml_to_return.append({R"(
  wp_)"});
            yaml_to_return.append(std::to_string(i-1).c_str());
            yaml_to_return.append(": [");
            yaml_to_return.append(std::to_string(current_geopoint.latitude).c_str());
            yaml_to_return.append(", ");
            yaml_to_return.append(std::to_string(current_geopoint.longitude).c_str());
            yaml_to_return.append(", ");
            yaml_to_return.append(std::to_string(current_geopoint.altitude).c_str());
            yaml_to_return.append("]");
        }
        yaml_to_return.append("\n\n");
    }
#ifdef DEBUG
    std::cout << yaml_to_return << std::endl;
#endif
    return yaml_to_return;
} // end translateFlightPlanIntoDJIyaml


std::string MissionController::translateFlightPlanIntoStdYaml(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plans) {
    std::string yaml_to_return;
    for (const aerialcore_msgs::FlightPlan& flight_plan : _flight_plans) {
        yaml_to_return.append("waypoints_");
        yaml_to_return.append(std::to_string(flight_plan.uav_id).c_str());
        yaml_to_return.append(": \"");
        for (const geometry_msgs::PoseStamped& current_pose : flight_plan.poses) {
            yaml_to_return.append(std::to_string(current_pose.pose.position.x).c_str());
            yaml_to_return.append(" ");
            yaml_to_return.append(std::to_string(current_pose.pose.position.y).c_str());
            yaml_to_return.append(" ");
            yaml_to_return.append(std::to_string(current_pose.pose.position.z).c_str());
            yaml_to_return.append(" 0;\n");
        }
        yaml_to_return.back() = '\"';
        yaml_to_return.append("\n\n");
    }
#ifdef DEBUG
    std::cout << yaml_to_return << std::endl;
#endif
    return yaml_to_return;
} // end translateFlightPlanIntoStdYaml


bool MissionController::stopSupervisingServiceCallback(aerialcore_msgs::StopSupervising::Request& _req, aerialcore_msgs::StopSupervising::Response& _res) {
    stop_current_supervising_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if(parameter_estimator_thread_.joinable()) parameter_estimator_thread_.join();
    if(plan_thread_.joinable()) plan_thread_.join();
    stop_current_supervising_ = false;

    if (_req.uav_id.size()>0) {     // There are specific UAVs to stop.
        for (const int& current_uav_id : _req.uav_id) {
            int current_uav_index = findUavIndexById(current_uav_id);
            if (current_uav_index == -1) continue;  // If UAV id not found just skip it.

            UAVs_[current_uav_index].mission->stop();
            UAVs_[current_uav_index].enabled_to_supervise = false;
        }
    } else {    // If empty stop all UAVs.
        for (UAV& current_uav : UAVs_) {
            current_uav.mission->stop();
            current_uav.enabled_to_supervise = false;
        }
    }

    _res.success=true;
    return true;
} // end stopSupervisingServiceCallback


bool MissionController::doCompleteSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res) {
    complete_or_specific_graph_supervision_ = true;

    current_graph_mutex_.lock();
    current_graph_.clear();
    current_graph_ = complete_graph_cleaned_;
    current_graph_mutex_.unlock();

    centralized_planner_.resetInspectedEdges();

    _res.success=true;
    return true;
} // end doCompleteSupervisionServiceCallback


bool MissionController::doSpecificSupervisionServiceCallback(aerialcore_msgs::DoSpecificSupervision::Request& _req, aerialcore_msgs::DoSpecificSupervision::Response& _res) {
    complete_or_specific_graph_supervision_ = false;
    specific_subgraph_.clear();
    for (const aerialcore_msgs::GraphNode& current_graph_node : _req.specific_subgraph) {
        specific_subgraph_.push_back(current_graph_node);
    }

    current_graph_mutex_.lock();
    current_graph_ = specific_subgraph_;
    removeGraphNodesAndEdgesAboveNoFlyZones(current_graph_);
    specific_subgraph_cleaned_ = current_graph_;
    current_graph_mutex_.unlock();

    centralized_planner_.resetInspectedEdges();

    _res.success=true;
    return true;
} // end doSpecificSupervisionServiceCallback


bool MissionController::doContinuousSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res) {
    continuous_or_fast_supervision_ = true;

    centralized_planner_.resetInspectedEdges();

    _res.success=true;
    return true;
} // end doContinuousSupervisionServiceCallback


bool MissionController::doFastSupervisionServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res) {
    continuous_or_fast_supervision_ = false;

    centralized_planner_.resetInspectedEdges();

    _res.success=true;
    return true;
} // end doFastSupervisionServiceCallback


bool MissionController::triggerReplanningManuallyServiceCallback(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res) {
    trigger_replanning_manually_ = true;

    _res.success=true;
    return true;
} // end triggerReplanningManuallyServiceCallback


bool MissionController::startSpecificSupervisionPlanServiceCallback(aerialcore_msgs::PostString::Request& _req, aerialcore_msgs::PostString::Response& _res) {
    ROS_INFO("Mission Controller: start specific supervision plan servive received.");

    // Stop supervising threads, if exist:
    stop_current_supervising_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if(parameter_estimator_thread_.joinable()) parameter_estimator_thread_.join();
    if(plan_thread_.joinable()) plan_thread_.join();
    stop_current_supervising_ = false;

    // Load to the parameter server the yaml string received:
    std::remove("current_plan.yaml");
    std::ofstream out("current_plan.yaml");
    out << _req.data;
    out.close();
    for (int id=1; id<=20; id++) {  // Remove previous waypoints parameters (if exist):
        if (ros::param::has("waypoints_"+std::to_string( id ))) {
            system((std::string("rosparam delete /waypoints_").append(std::to_string( id ).c_str())).c_str());
        }
    }
    system("rosparam load current_plan.yaml");

    current_graph_mutex_.lock();
    current_graph_.clear();
    current_graph_mutex_.unlock();

    ros::Time planning_time = ros::Time::now();
    flight_plans_.clear();
    aerialcore_msgs::FlightPlan current_flight_plan;

    // Iterate the parameters loaded and create from them the graph:
    for (int id=1; id<=20; id++) {
        if (ros::param::has("waypoints_"+std::to_string( id ))) {
            current_flight_plan.uav_id = id;

            // Build the initial UAV positions and regular land stations (same spot where the UAV is when yaml string received) graph nodes for the current graph:
            aerialcore_msgs::GraphNode initial_graph_node;
            initial_graph_node.type = aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION;
            int current_uav_index = findUavIndexById(id);
            if (current_uav_index == -1) continue;  // If UAV id not found just skip it.

            if (UAV_command_mode_=="mission_lib") {
                geometry_msgs::PoseStamped uav_pose = UAVs_[current_uav_index].mission->pose();
                initial_graph_node.x = uav_pose.pose.position.x;
                initial_graph_node.y = uav_pose.pose.position.y;
                initial_graph_node.z = uav_pose.pose.position.z;

                geometry_msgs::Point32 aux_point32;
                aux_point32.x = initial_graph_node.x;
                aux_point32.y = initial_graph_node.y;
                aux_point32.z = initial_graph_node.z;
                geographic_msgs::GeoPoint uav_geopoint = cartesian_to_geographic(aux_point32, map_origin_geo_);
                initial_graph_node.latitude  = uav_geopoint.latitude;
                initial_graph_node.longitude = uav_geopoint.longitude;
                initial_graph_node.altitude  = uav_geopoint.altitude;
            } else if (UAV_command_mode_=="DJI_SDK_yaml" || UAV_command_mode_=="USE_CTU_yaml") {
                // TODO: should read these from the SDK or other place.
                n_.getParam(ns_uav_prefix_+std::to_string(id)+"/x", initial_graph_node.x);
                n_.getParam(ns_uav_prefix_+std::to_string(id)+"/y", initial_graph_node.y);
                n_.getParam(ns_uav_prefix_+std::to_string(id)+"/z", initial_graph_node.z);
                n_.getParam(ns_uav_prefix_+std::to_string(id)+"/latitude", initial_graph_node.latitude);
                n_.getParam(ns_uav_prefix_+std::to_string(id)+"/longitude", initial_graph_node.longitude);
                n_.getParam(ns_uav_prefix_+std::to_string(id)+"/altitude", initial_graph_node.altitude);
            }

            geometry_msgs::PoseStamped uav_pose;
            uav_pose.pose.position.x = initial_graph_node.x;
            uav_pose.pose.position.y = initial_graph_node.y;
            uav_pose.pose.position.z = initial_graph_node.z;
            if (uav_pose.pose.position.z <= FLYING_THRESHOLD) {
                uav_pose.pose.position.z += 30;
                current_flight_plan.type.push_back(aerialcore_msgs::FlightPlan::TYPE_TAKEOFF_WP);
            } else {
                current_flight_plan.type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP);
            }
            current_flight_plan.poses.push_back(uav_pose);
            current_flight_plan.nodes.push_back(current_graph_.size());

            current_graph_mutex_.lock();
            current_graph_.push_back(initial_graph_node);
            initial_graph_node.type = aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION;
            initial_graph_node.id = id;
            current_graph_.push_back(initial_graph_node);
            current_graph_mutex_.unlock();

            // Build the pylons graph nodes for the current graph from the yaml string received:
            std::string waypoints;
            ros::param::get("waypoints_"+std::to_string( id ),waypoints);
            // Remove new lines if exist in the strings for all strings and semicolons:
            waypoints.erase(std::remove(waypoints.begin(), waypoints.end(), '\n'), waypoints.end());
            waypoints.erase(std::remove(waypoints.begin(), waypoints.end(), '\r'), waypoints.end());
            waypoints.erase(std::remove(waypoints.begin(), waypoints.end(), ','), waypoints.end());
            waypoints.erase(std::remove(waypoints.begin(), waypoints.end(), ';'), waypoints.end());
            // For strings with point information, convert the string data, clean and ready, to double (float64 because of point) with stod.
            // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
            // The quantity of numbers have to be multiple of 4.
            std::string::size_type sz;
            aerialcore_msgs::GraphNode pylon_graph_node;
            pylon_graph_node.type = aerialcore_msgs::GraphNode::TYPE_PYLON;
            while ( waypoints.size()>0 && waypoints!=" " ) {
                pylon_graph_node.x = (float) std::stod (waypoints,&sz);
                waypoints = waypoints.substr(sz);
                pylon_graph_node.y = (float) std::stod (waypoints,&sz);
                waypoints = waypoints.substr(sz);
                pylon_graph_node.z = (float) std::stod (waypoints,&sz);
                waypoints = waypoints.substr(sz);
                (float) std::stod (waypoints,&sz);  // Don't save right now the heading.
                waypoints = waypoints.substr(sz);

                geometry_msgs::Point32 aux_point32;
                aux_point32.x = pylon_graph_node.x;
                aux_point32.y = pylon_graph_node.y;
                aux_point32.z = pylon_graph_node.z;
                geographic_msgs::GeoPoint pylon_geopoint = cartesian_to_geographic(aux_point32, map_origin_geo_);
                pylon_graph_node.latitude  = pylon_geopoint.latitude;
                pylon_graph_node.longitude = pylon_geopoint.longitude;
                pylon_graph_node.altitude  = pylon_geopoint.altitude;

                geometry_msgs::PoseStamped pylon_pose;
                pylon_pose.pose.position.x = pylon_graph_node.x;
                pylon_pose.pose.position.y = pylon_graph_node.y;
                pylon_pose.pose.position.z = pylon_graph_node.z;
                current_flight_plan.poses.push_back(pylon_pose);
                current_flight_plan.type.push_back(aerialcore_msgs::FlightPlan::TYPE_PASS_PYLON_WP);
                current_flight_plan.nodes.push_back(current_graph_.size());

                current_graph_mutex_.lock();
                current_graph_.push_back(pylon_graph_node);
                current_graph_mutex_.unlock();
            }

            geometry_msgs::PoseStamped land_pose;
            land_pose.pose.position.x = current_flight_plan.poses.back().pose.position.x;
            land_pose.pose.position.y = current_flight_plan.poses.back().pose.position.y;
            land_pose.pose.position.z = current_flight_plan.poses.back().pose.position.z;
            current_flight_plan.poses.push_back(land_pose);
            current_flight_plan.type.push_back(aerialcore_msgs::FlightPlan::TYPE_LAND_WP);

            flight_plans_.push_back(current_flight_plan);

            current_flight_plan.nodes.clear();
            current_flight_plan.poses.clear();
            current_flight_plan.type.clear();
        }
    }
#ifdef DEBUG
    printCurrentGraph();
#endif

    // First disable all UAVs from supervising to then enable only the ones used:
    for (UAV& current_uav : UAVs_) {
        current_uav.enabled_to_supervise = false;
    }

    if (UAV_command_mode_=="mission_lib") {
        translateFlightPlanIntoUAVMission(flight_plans_);

        for (const aerialcore_msgs::FlightPlan& flight_plan : flight_plans_) {
            int current_uav_index = findUavIndexById(flight_plan.uav_id);
            if (current_uav_index == -1) continue;  // If UAV id not found just skip it.
            UAVs_[current_uav_index].enabled_to_supervise = true;
#ifdef DEBUG
            UAVs_[current_uav_index].mission->print();
#endif
            UAVs_[current_uav_index].mission->push();
            UAVs_[current_uav_index].mission->start();
        }
        _res.success=true;
    } else if (UAV_command_mode_=="DJI_SDK_yaml") {
        aerialcore_msgs::PostString post_yaml_string;
        post_yaml_string.request.data = translateFlightPlanIntoDJIyaml(flight_plans_);

        std::ofstream file_out("plan.yaml");
        file_out << post_yaml_string.request.data << std::endl;
        file_out.close();

        if (post_yaml_client_.call(post_yaml_string)) {
            ROS_INFO("Mission Controller: yaml sent to DJI SDK.");
            _res.success=true;
        } else {
            ROS_WARN("Mission Controller: yaml not sent to DJI SDK, service didn't answer.");
            _res.success=false;
        }
    } else if (UAV_command_mode_=="USE_CTU_yaml") {
        aerialcore_msgs::PostString post_yaml_string;
        post_yaml_string.request.data = translateFlightPlanIntoStdYaml(flight_plans_);

        std::ofstream file_out("plan.yaml");
        file_out << post_yaml_string.request.data << std::endl;
        file_out.close();

        if (post_yaml_client_.call(post_yaml_string)) {
            ROS_INFO("Mission Controller: standard yaml sent.");
        } else {
            ROS_WARN("Mission Controller: standard yaml not sent, service didn't answer.");
        }
    }

    return true;
} // end startSpecificSupervisionPlanServiceCallback


void MissionController::removeGraphNodesAndEdgesAboveNoFlyZones(std::vector<aerialcore_msgs::GraphNode>& _graph_to_edit) {

    // Check before if there is a geofence and no-fly zones, if not do nothing:
    if (geofence_.points.size()==0 && no_fly_zones_.size()==0) {
        return;
    }

    // If any node is unvalid (inside obstacles or outside the geofence), remove the node and its connections:
    for (int i=0; i<_graph_to_edit.size(); i++) {
        geometry_msgs::Point32 test_point;
        test_point.x = _graph_to_edit[i].x;
        test_point.y = _graph_to_edit[i].y;

        if (!path_planner_.checkIfPointIsValid(test_point)) {
            if (_graph_to_edit[i].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
                for (int j=0; j<_graph_to_edit.size(); j++) {
                    if (_graph_to_edit[j].type==aerialcore_msgs::GraphNode::TYPE_PYLON && i!=j) {
                        for (int k=_graph_to_edit[j].connections_indexes.size()-1; k>=0; k--) {
                            if (_graph_to_edit[j].connections_indexes[k] == i) {
                                _graph_to_edit[j].connections_indexes.erase(_graph_to_edit[j].connections_indexes.begin()+k);
                            }
                        }
                    }
                }
            }
            _graph_to_edit[i].type = aerialcore_msgs::GraphNode::TYPE_NO_FLY_ZONE;  // Nodes marked as TYPE_NO_FLY_ZONE will be considered removed.
        }
    }

    // Check if there are edges above no-fly zones, and in that cases remove those connections:
    for (int i=0; i<_graph_to_edit.size(); i++) {
        if (_graph_to_edit[i].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
            for (int j=_graph_to_edit[i].connections_indexes.size()-1; j>=0; j--) {
                geometry_msgs::Point32 test_point_1, test_point_2;
                test_point_1.x = _graph_to_edit[i].x;       test_point_2.x = _graph_to_edit[ _graph_to_edit[i].connections_indexes[j] ].x;
                test_point_1.y = _graph_to_edit[i].y;       test_point_2.y = _graph_to_edit[ _graph_to_edit[i].connections_indexes[j] ].y;
                test_point_1.z = _graph_to_edit[i].z;       test_point_2.z = _graph_to_edit[ _graph_to_edit[i].connections_indexes[j] ].z;
                if (!path_planner_.checkIfTwoPointsAreVisible(test_point_1, test_point_2)) {
                    _graph_to_edit[i].connections_indexes.erase(_graph_to_edit[i].connections_indexes.begin()+j);
                }
            }
        }
    }

    // Check if after removing nodes and connections some nodes are now alone without connections. In that case remove them:
    for (int i=0; i<_graph_to_edit.size(); i++) {
        if (_graph_to_edit[i].type==aerialcore_msgs::GraphNode::TYPE_PYLON && _graph_to_edit[i].connections_indexes.size()==0) {
            _graph_to_edit[i].type = aerialcore_msgs::GraphNode::TYPE_NO_FLY_ZONE;
        }
    }

} // end removeGraphNodesAndEdgesAboveNoFlyZones


int MissionController::findUavIndexById(int _UAV_id) {
    int uav_index = -1;
    for (int i=0; i<UAVs_.size(); i++) {
        if (UAVs_[i].id == _UAV_id) {
            uav_index = i;
            break;
        }
    }
    if (uav_index == -1) {
        ROS_ERROR("Mission Controller: UAV id=%d provided not found on the Mission Controller.", _UAV_id);
    }
    return uav_index;
} // end findUavIndexById


void MissionController::printCurrentGraph() {
    ROS_INFO("Mission Controller: printing the current graph.");
    current_graph_mutex_.lock();
    for (int i=0; i<current_graph_.size(); i++) {      // Print current_graph_ to check that the yaml file was parsed correctly:
        if (current_graph_[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
            std::cout << "graph_node[ " << i << " ].type                     = TYPE_PYLON" << std::endl;
        } else if (current_graph_[i].type == aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) {
            std::cout << "graph_node[ " << i << " ].type                     = TYPE_RECHARGE_LAND_STATION" << std::endl;
        } else if (current_graph_[i].type == aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
            std::cout << "graph_node[ " << i << " ].type                     = TYPE_REGULAR_LAND_STATION" << std::endl;
        } else if (current_graph_[i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
            std::cout << "graph_node[ " << i << " ].type                     = TYPE_UAV_INITIAL_POSITION" << std::endl;
        } else if (current_graph_[i].type == aerialcore_msgs::GraphNode::TYPE_NO_FLY_ZONE) {
            std::cout << "graph_node[ " << i << " ].type                     = TYPE_NO_FLY_ZONE" << std::endl;
        }
        std::cout << "graph_node[ " << i << " ].id                       = " << current_graph_[i].id << std::endl;
        for (int j=0; j<current_graph_[i].connections_indexes.size(); j++) {
            std::cout << "graph_node[ " << i << " ].connections_indexes[ " << j << " ] = " << current_graph_[i].connections_indexes[j] << std::endl;
        }
        std::cout << "graph_node[ " << i << " ].x                        = " << current_graph_[i].x << std::endl;
        std::cout << "graph_node[ " << i << " ].y                        = " << current_graph_[i].y << std::endl;
        std::cout << "graph_node[ " << i << " ].z                        = " << current_graph_[i].z << std::endl;
        std::cout << "graph_node[ " << i << " ].latitude                 = " << current_graph_[i].latitude << std::endl;
        std::cout << "graph_node[ " << i << " ].longitude                = " << current_graph_[i].longitude << std::endl;
        std::cout << "graph_node[ " << i << " ].altitude                 = " << current_graph_[i].altitude << std::endl;
    }
    current_graph_mutex_.unlock();
}


bool MissionController::compareFlightPlanById(const aerialcore_msgs::FlightPlan &a, const aerialcore_msgs::FlightPlan &b) {
    return a.uav_id < b.uav_id;
}


// Battery faker thread (if simulation_==true):
void MissionController::batteryFakerThread(void) {

    float sample_time = 1.0;    // Battery update time.

    // In simulation batteries are supposed to start fully charged.
    for (int i=0; i<UAVs_.size(); i++) {
        UAVs_[i].battery_faked = 1;
    }

    ros::Rate loop_rate(1.0/sample_time);   // [Hz] Update rate of the battery.
    while (ros::ok()) {
        for (int i=0; i<UAVs_.size(); i++) {

            geometry_msgs::PoseStamped uav_pose = UAVs_[i].mission->pose();

            // Landed (disarmed):
            if (UAVs_[i].mission->armed()==false) {

                // Calculate the closest land station:
                int closest_land_station = -1;
                float closest_land_station_distance = std::numeric_limits<float>::max();
                current_graph_mutex_.lock();
                for (int j=0; j<current_graph_.size(); j++) {
                    if (current_graph_[j].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION || current_graph_[j].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
                        float current_distance = sqrt( pow(current_graph_[j].x-uav_pose.pose.position.x,2) + pow(current_graph_[j].y-uav_pose.pose.position.y,2) );
                        if ( closest_land_station_distance > current_distance ) {
                            closest_land_station = j;
                            closest_land_station_distance = current_distance;
                        }
                    }
                }
                current_graph_mutex_.unlock();

                // Landed on a recharge land station (closest land station is with recharge and closer than 5 meters):
                if (current_graph_[closest_land_station].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION && closest_land_station_distance<=5) {
                    UAVs_[i].battery_faked = UAVs_[i].battery_faked + sample_time/UAVs_[i].time_until_fully_charged <= 1.0 ? UAVs_[i].battery_faked + sample_time/UAVs_[i].time_until_fully_charged : 1.0;
                    UAVs_[i].recharging = true;

                // Landed on a regular land station (no recharge):
                } else {
                    UAVs_[i].battery_faked = UAVs_[i].battery_faked;
                    UAVs_[i].recharging = false;
                }

            // Flying (supposed if armed):
            } else {

                if (UAVs_[i].airframe_type=="MULTICOPTER") {

                    geometry_msgs::TwistStamped uav_velocity = UAVs_[i].mission->velocity();
                    float horizontal_velocity = sqrt( pow(uav_velocity.twist.linear.x,2) + pow(uav_velocity.twist.linear.y,2) );

                    // Hovering:
                    if ( abs(uav_velocity.twist.linear.z) < 0.3 && horizontal_velocity < 0.3) {
                        UAVs_[i].battery_faked = UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterHoverPower(UAVs_[i].id) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ >= 0 ? UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterHoverPower(UAVs_[i].id) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ : 0;
                        UAVs_[i].recharging = false;

                    // Ascending:
                    } else if ( abs(uav_velocity.twist.linear.z) > horizontal_velocity && uav_velocity.twist.linear.z > 0) {
                        UAVs_[i].battery_faked = UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterClimbPower(UAVs_[i].id, uav_velocity.twist.linear.z) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ >= 0 ? UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterClimbPower(UAVs_[i].id, uav_velocity.twist.linear.z) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ : 0;
                        UAVs_[i].recharging = false;

                    // Descending:
                    } else if ( abs(uav_velocity.twist.linear.z) > horizontal_velocity && uav_velocity.twist.linear.z < 0) {
                        UAVs_[i].battery_faked = UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterDescentPower(UAVs_[i].id, uav_velocity.twist.linear.z) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ >= 0 ? UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterDescentPower(UAVs_[i].id, uav_velocity.twist.linear.z) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ : 0;
                        UAVs_[i].recharging = false;

                    // Forward:
                    } else {
                        UAVs_[i].battery_faked = UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterForwardPower(UAVs_[i].id, uav_velocity) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ >= 0 ? UAVs_[i].battery_faked - sample_time /* [s] */ * parameter_estimator_.multicopterForwardPower(UAVs_[i].id, uav_velocity) /* [W = J/s] */ /UAVs_[i].joules /* [J] */ : 0;
                        UAVs_[i].recharging = false;
                    }

                } else {
                    UAVs_[i].battery_faked = UAVs_[i].battery_faked - sample_time/UAVs_[i].time_max_flying >= 0 ? UAVs_[i].battery_faked - sample_time/UAVs_[i].time_max_flying : 0;
                    UAVs_[i].recharging = false;
                }

            }

// #ifdef DEBUG
//             float battery = UAVs_[i].battery_faked * 100;
//             std::cout << "UAVs_[ " << UAVs_[i].id << " ].battery_faked = " << battery << std::endl;
// #endif
        }

        loop_rate.sleep();
    }
}


} // end namespace aerialcore