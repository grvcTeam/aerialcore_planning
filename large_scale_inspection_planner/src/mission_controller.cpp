/**
 * AERIALCORE Project:
 *
 * Mission controller.
 * 
 */

#include <mission_controller.h>

#define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)

namespace aerialcore {


// Constructor
MissionController::MissionController() {
    pnh_ = ros::NodeHandle("~");

    pnh_.getParam("commanding_UAV_with_mission_lib_or_DJI_SDK", commanding_UAV_with_mission_lib_or_DJI_SDK_);

    // map_origin_geo is the geographic coordinate origin of the cartesian coordinates. Loaded to the param server in the YAML config file.
    std::vector<double> map_origin_geo_vector;
    n_.getParam("map_origin_geo", map_origin_geo_vector);
    map_origin_geo_.latitude  = map_origin_geo_vector[0];
    map_origin_geo_.longitude = map_origin_geo_vector[1];
    map_origin_geo_.altitude  = map_origin_geo_vector[2];

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
        new_uav.mission = new grvc::mission_ns::Mission( new_uav.id );
        int time_max_flying;            n_.getParam(it->second+"/time_max_flying", time_max_flying);                     new_uav.time_max_flying = time_max_flying;
        float speed_xy;                 n_.getParam(it->second+"/speed_xy", speed_xy);                                   new_uav.speed_xy = speed_xy;
        float speed_z_down;             n_.getParam(it->second+"/speed_z_down", speed_z_down);                           new_uav.speed_z_down = speed_z_down;
        float speed_z_up;               n_.getParam(it->second+"/speed_z_up", speed_z_up);                               new_uav.speed_z_up = speed_z_up;
        float minimum_battery;          n_.getParam(it->second+"/minimum_battery", minimum_battery);                     new_uav.minimum_battery = minimum_battery;
        int time_until_fully_charged;   n_.getParam(it->second+"/time_until_fully_charged", time_until_fully_charged);   new_uav.time_until_fully_charged = time_until_fully_charged;
        UAVs_.push_back(new_uav);
    }

    // Read parameters of the complete graph (from the yaml). Numeric parameters extracted from a string, so some steps are needed:
    std::string pylons_position_string;
    std::string pylons_position_geo_string;
    std::string connections_indexes_string;
    std::string recharge_land_stations_string;
    std::string recharge_land_stations_geo_string;
    std::string regular_land_stations_string;
    std::string regular_land_stations_geo_string;
    n_.getParam("pylons_position_cartesian", pylons_position_string);
    n_.getParam("pylons_position_geographic", pylons_position_geo_string);
    n_.getParam("pylons_connections_indexes", connections_indexes_string);
    n_.getParam("recharge_land_stations_cartesian", recharge_land_stations_string);
    n_.getParam("recharge_land_stations_geographic", recharge_land_stations_geo_string);
    n_.getParam("regular_land_stations_cartesian", regular_land_stations_string);
    n_.getParam("regular_land_stations_geographic", regular_land_stations_geo_string);
    // Remove new lines if exist in the strings for all strings and semicolons in all but connection_indexes:
    pylons_position_string.erase(std::remove(pylons_position_string.begin(), pylons_position_string.end(), '\n'), pylons_position_string.end());
    pylons_position_string.erase(std::remove(pylons_position_string.begin(), pylons_position_string.end(), '\r'), pylons_position_string.end());
    pylons_position_string.erase(std::remove(pylons_position_string.begin(), pylons_position_string.end(), ';'), pylons_position_string.end());
    pylons_position_geo_string.erase(std::remove(pylons_position_geo_string.begin(), pylons_position_geo_string.end(), '\n'), pylons_position_geo_string.end());
    pylons_position_geo_string.erase(std::remove(pylons_position_geo_string.begin(), pylons_position_geo_string.end(), '\r'), pylons_position_geo_string.end());
    pylons_position_geo_string.erase(std::remove(pylons_position_geo_string.begin(), pylons_position_geo_string.end(), ';'), pylons_position_geo_string.end());
    connections_indexes_string.erase(std::remove(connections_indexes_string.begin(), connections_indexes_string.end(), '\n'), connections_indexes_string.end());
    connections_indexes_string.erase(std::remove(connections_indexes_string.begin(), connections_indexes_string.end(), '\r'), connections_indexes_string.end());
    recharge_land_stations_string.erase(std::remove(recharge_land_stations_string.begin(), recharge_land_stations_string.end(), '\n'), recharge_land_stations_string.end());
    recharge_land_stations_string.erase(std::remove(recharge_land_stations_string.begin(), recharge_land_stations_string.end(), '\r'), recharge_land_stations_string.end());
    recharge_land_stations_string.erase(std::remove(recharge_land_stations_string.begin(), recharge_land_stations_string.end(), ';'), recharge_land_stations_string.end());
    recharge_land_stations_geo_string.erase(std::remove(recharge_land_stations_geo_string.begin(), recharge_land_stations_geo_string.end(), '\n'), recharge_land_stations_geo_string.end());
    recharge_land_stations_geo_string.erase(std::remove(recharge_land_stations_geo_string.begin(), recharge_land_stations_geo_string.end(), '\r'), recharge_land_stations_geo_string.end());
    recharge_land_stations_geo_string.erase(std::remove(recharge_land_stations_geo_string.begin(), recharge_land_stations_geo_string.end(), ';'), recharge_land_stations_geo_string.end());
    regular_land_stations_string.erase(std::remove(regular_land_stations_string.begin(), regular_land_stations_string.end(), '\n'), regular_land_stations_string.end());
    regular_land_stations_string.erase(std::remove(regular_land_stations_string.begin(), regular_land_stations_string.end(), '\r'), regular_land_stations_string.end());
    regular_land_stations_string.erase(std::remove(regular_land_stations_string.begin(), regular_land_stations_string.end(), ';'), regular_land_stations_string.end());
    regular_land_stations_geo_string.erase(std::remove(regular_land_stations_geo_string.begin(), regular_land_stations_geo_string.end(), '\n'), regular_land_stations_geo_string.end());
    regular_land_stations_geo_string.erase(std::remove(regular_land_stations_geo_string.begin(), regular_land_stations_geo_string.end(), '\r'), regular_land_stations_geo_string.end());
    regular_land_stations_geo_string.erase(std::remove(regular_land_stations_geo_string.begin(), regular_land_stations_geo_string.end(), ';'), regular_land_stations_geo_string.end());
    // For strings with point information, convert the string data, clean and ready, to double (float64 because of point) with stod.
    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
    // The quantity of numbers have to be multiple of 3.
    // For connection_indexes do that but separating previously strings by semicolons:
    std::string::size_type sz;
    aerialcore_msgs::GraphNode current_graph_node;
    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_PYLON;
    while ( pylons_position_string.size()>0 && pylons_position_string!=" " ) {
        current_graph_node.x = (float) std::stod (pylons_position_string,&sz);
        pylons_position_string = pylons_position_string.substr(sz);
        current_graph_node.y = (float) std::stod (pylons_position_string,&sz);
        pylons_position_string = pylons_position_string.substr(sz);
        current_graph_node.z = (float) std::stod (pylons_position_string,&sz);
        pylons_position_string = pylons_position_string.substr(sz);
        current_graph_node.latitude  = (float) std::stod (pylons_position_geo_string,&sz);
        pylons_position_geo_string = pylons_position_geo_string.substr(sz);
        current_graph_node.longitude = (float) std::stod (pylons_position_geo_string,&sz);
        pylons_position_geo_string = pylons_position_geo_string.substr(sz);
        current_graph_node.altitude  = (float) std::stod (pylons_position_geo_string,&sz);
        pylons_position_geo_string = pylons_position_geo_string.substr(sz);
        complete_graph_.push_back(current_graph_node);
    }
    for (int i=0; i<complete_graph_.size(); i++) {
        std::string delimiter = ";";
        std::string string_until_first_semicolon = connections_indexes_string.substr(0, connections_indexes_string.find(delimiter));
        int current_connection_index;
        while ( string_until_first_semicolon.size()>0 && string_until_first_semicolon!=" " ) {
            current_connection_index = (int) std::stod (string_until_first_semicolon,&sz);
            string_until_first_semicolon = string_until_first_semicolon.substr(sz);
            complete_graph_[i].connections_indexes.push_back(current_connection_index-1);
        }
        connections_indexes_string.erase(0, connections_indexes_string.find(delimiter) + delimiter.length());
    }
    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION;
    while ( recharge_land_stations_string.size()>0 && recharge_land_stations_string!=" " ) {
        current_graph_node.x = (float) std::stod (recharge_land_stations_string,&sz);
        recharge_land_stations_string = recharge_land_stations_string.substr(sz);
        current_graph_node.y = (float) std::stod (recharge_land_stations_string,&sz);
        recharge_land_stations_string = recharge_land_stations_string.substr(sz);
        current_graph_node.z = (float) std::stod (recharge_land_stations_string,&sz);
        recharge_land_stations_string = recharge_land_stations_string.substr(sz);
        current_graph_node.latitude  = (float) std::stod (recharge_land_stations_geo_string,&sz);
        recharge_land_stations_geo_string = recharge_land_stations_geo_string.substr(sz);
        current_graph_node.longitude = (float) std::stod (recharge_land_stations_geo_string,&sz);
        recharge_land_stations_geo_string = recharge_land_stations_geo_string.substr(sz);
        current_graph_node.altitude  = (float) std::stod (recharge_land_stations_geo_string,&sz);
        recharge_land_stations_geo_string = recharge_land_stations_geo_string.substr(sz);
        complete_graph_.push_back(current_graph_node);
    }
    current_graph_node.type = aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION;
    while ( regular_land_stations_string.size()>0 && regular_land_stations_string!=" " ) {
        current_graph_node.x = (float) std::stod (regular_land_stations_string,&sz);
        regular_land_stations_string = regular_land_stations_string.substr(sz);
        current_graph_node.y = (float) std::stod (regular_land_stations_string,&sz);
        regular_land_stations_string = regular_land_stations_string.substr(sz);
        current_graph_node.z = (float) std::stod (regular_land_stations_string,&sz);
        regular_land_stations_string = regular_land_stations_string.substr(sz);
        current_graph_node.latitude  = (float) std::stod (regular_land_stations_geo_string,&sz);
        regular_land_stations_geo_string = regular_land_stations_geo_string.substr(sz);
        current_graph_node.longitude = (float) std::stod (regular_land_stations_geo_string,&sz);
        regular_land_stations_geo_string = regular_land_stations_geo_string.substr(sz);
        current_graph_node.altitude  = (float) std::stod (regular_land_stations_geo_string,&sz);
        regular_land_stations_geo_string = regular_land_stations_geo_string.substr(sz);
        complete_graph_.push_back(current_graph_node);
    }
    current_graph_ = complete_graph_;
#ifdef DEBUG
    for (int i=0; i<complete_graph_.size(); i++) {      // Print complete_graph_ to check that the yaml file was parsed correctly:
        std::cout << "graph_node[ " << i << " ].type                     = " << (int) complete_graph_[i].type << std::endl;
        std::cout << "graph_node[ " << i << " ].id                       = " << complete_graph_[i].id << std::endl;
        for (int j=0; j<complete_graph_[i].connections_indexes.size(); j++) {
            std::cout << "graph_node[ " << i << " ].connections_indexes[ " << j << " ] = " << complete_graph_[i].connections_indexes[j] << std::endl;
        }
        std::cout << "graph_node[ " << i << " ].x                        = " << complete_graph_[i].x << std::endl;
        std::cout << "graph_node[ " << i << " ].y                        = " << complete_graph_[i].y << std::endl;
        std::cout << "graph_node[ " << i << " ].z                        = " << complete_graph_[i].z << std::endl;
        std::cout << "graph_node[ " << i << " ].latitude                 = " << complete_graph_[i].latitude << std::endl;
        std::cout << "graph_node[ " << i << " ].longitude                = " << complete_graph_[i].longitude << std::endl;
        std::cout << "graph_node[ " << i << " ].altitude                 = " << complete_graph_[i].altitude << std::endl;
    }
#endif

    // Advertised services:
    start_supervising_srv_ = n_.advertiseService("mission_controller/start_supervising", &MissionController::startSupervisingServiceCallback, this);
    stop_supervising_srv_ = n_.advertiseService("mission_controller/stop_supervising", &MissionController::stopSupervisingServiceCallback, this);
    do_specific_supervision_srv_ = n_.advertiseService("mission_controller/do_specific_supervision", &MissionController::doSpecificSupervisionServiceCallback, this);
    do_continuous_supervision_srv_ = n_.advertiseService("mission_controller/do_continuous_supervision", &MissionController::doContinuousSupervisionServiceCallback, this);
    start_specific_supervision_plan_srv_ = n_.advertiseService("mission_controller/start_specific_supervision_plan", &MissionController::startSpecificSupervisionPlanServiceCallback, this);

    // Clients:
    post_yaml_client_ = n_.serviceClient<aerialcore_msgs::PostString>("post_yaml");

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
    for (UAV& uav : UAVs_) {
        delete uav.mission;
    }
} // end ~MissionController destructor


bool MissionController::startSupervisingServiceCallback(aerialcore_msgs::StartSupervising::Request& _req, aerialcore_msgs::StartSupervising::Response& _res) {
    if (_req.uav_id.size()>0) {     // There are specific UAVs to start.
        for (const int& current_uav_id : _req.uav_id) {
            UAVs_[findUavIndexById(current_uav_id)].enabled_to_supervise = true;
        }
    } else {    // If empty start all UAVs.
        for (UAV& current_uav : UAVs_) {
            current_uav.enabled_to_supervise = true;
        }
    }

    // Erase the graph nodes corresponding to the initial position of the UAVs (if there are any because of previously started missions).
    std::vector<int> indexes_to_erase;              // Dont't erase directly the indexes, it's safer erase later with a sorted decreasing vector.
    for (int i=0; i<current_graph_.size(); i++) {
        if (current_graph_[i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
            indexes_to_erase.push_back(i);
        }
    }
    // Reverse order of the vector of indexes to erase (from more to less now):
    std::reverse(indexes_to_erase.begin(), indexes_to_erase.end());
    // Erase indexes of the graph:
    for (int current_index_to_erase: indexes_to_erase) {
        current_graph_.erase( current_graph_.begin() + current_index_to_erase );
    }

    std::vector< std::tuple<float, float, int, int, int, int, int, int, bool, bool> > drone_info;
    for (const UAV& current_uav : UAVs_) {
        if (current_uav.enabled_to_supervise == true) {
            std::tuple<float, float, int, int, int, int, int, int, bool, bool> new_tuple;
            std::get<0>(new_tuple) = commanding_UAV_with_mission_lib_or_DJI_SDK_ ? current_uav.mission->battery() : 1.0;
            std::get<1>(new_tuple) = current_uav.minimum_battery;
            std::get<2>(new_tuple) = current_uav.time_until_fully_charged;
            std::get<3>(new_tuple) = current_uav.time_max_flying;
            std::get<4>(new_tuple) = current_uav.speed_xy;
            std::get<5>(new_tuple) = current_uav.speed_z_down;
            std::get<6>(new_tuple) = current_uav.speed_z_up;
            std::get<7>(new_tuple) = current_uav.id;
            std::get<8>(new_tuple) = commanding_UAV_with_mission_lib_or_DJI_SDK_ ? current_uav.mission->armed() : false;  // TODO: better way to know if flying?
            std::get<9>(new_tuple) = current_uav.recharging;
            drone_info.push_back(new_tuple);

            aerialcore_msgs::GraphNode current_uav_initial_position_graph_node;
            current_uav_initial_position_graph_node.type = aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION;
            current_uav_initial_position_graph_node.id = current_uav.id;
            if (commanding_UAV_with_mission_lib_or_DJI_SDK_) {
                current_uav_initial_position_graph_node.x = current_uav.mission->pose().pose.position.x;
                current_uav_initial_position_graph_node.y = current_uav.mission->pose().pose.position.y;
                current_uav_initial_position_graph_node.z = current_uav.mission->pose().pose.position.z;
            } else {
                // // HARDCODED INITIAL POSES FOR NOVEMBER EXPERIMENTS?
                // current_uav_initial_position_graph_node.x = current_uav.id==1 ? 28 : 36.119532;
                // current_uav_initial_position_graph_node.y = current_uav.id==1 ? 61 : 63.737163;
                // current_uav_initial_position_graph_node.z = current_uav.id==1 ? 0.32 : 0;
            }
            current_graph_.push_back(current_uav_initial_position_graph_node);
        }
    }

    flight_plan_ = centralized_planner_.getPlan(current_graph_, drone_info);

    // TODO: no-fly zones.

#ifdef DEBUG
    centralized_planner_.printPlan();
#endif

    if (commanding_UAV_with_mission_lib_or_DJI_SDK_) {
        translateFlightPlanIntoUAVMission(flight_plan_);

        for (const aerialcore_msgs::FlightPlan& flight_plan_for_current_uav : flight_plan_) {
            // // HARDCODED WAITS BETWEEN TAKEOFFS FOR NOVEMBER EXPERIMENTS:
            // if (flight_plan_for_current_uav.uav_id==2) sleep(10);
            // if (flight_plan_for_current_uav.uav_id==3) sleep(74);
#ifdef DEBUG
            UAVs_[findUavIndexById(flight_plan_for_current_uav.uav_id)].mission->print();
#endif
            UAVs_[findUavIndexById(flight_plan_for_current_uav.uav_id)].mission->push();
            UAVs_[findUavIndexById(flight_plan_for_current_uav.uav_id)].mission->start();
        }
        _res.success=true;
    } else {
        aerialcore_msgs::PostString post_yaml_string;
        post_yaml_string.request.data = translateFlightPlanIntoDJIyaml(flight_plan_);
        if (post_yaml_client_.call(post_yaml_string)) {
            ROS_INFO("Mission Controller: yaml sent to DJI SDK.");
            _res.success=true;
        } else {
            ROS_WARN("Mission Controller: yaml not sent to DJI SDK, service didn't answer.");
            _res.success=false;
        }
    }
    return true;
} // end startSupervisingServiceCallback


void MissionController::translateFlightPlanIntoUAVMission(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plan) {
    for (const aerialcore_msgs::FlightPlan& flight_plan_for_current_uav : _flight_plan) {
        int current_uav_index = findUavIndexById(flight_plan_for_current_uav.uav_id);

        UAVs_[current_uav_index].mission->clear();

        bool first_iteration = true;
        bool flying_or_landed; // True if flying at the time after the wp is inserted, false if landed. Useful to track takeoffs vs landings, both with negative nodes.

        std::vector<geometry_msgs::PoseStamped> pass_poses;

        for (int i=0; i<flight_plan_for_current_uav.nodes.size(); i++) {
            geometry_msgs::PoseStamped current_pose_stamped;
            current_pose_stamped.pose.position.x = current_graph_[ flight_plan_for_current_uav.nodes[i] ].x;
            current_pose_stamped.pose.position.y = current_graph_[ flight_plan_for_current_uav.nodes[i] ].y;
            current_pose_stamped.pose.position.z = current_graph_[ flight_plan_for_current_uav.nodes[i] ].altitude == 0 ? current_graph_[ flight_plan_for_current_uav.nodes[i] ].z : current_graph_[ flight_plan_for_current_uav.nodes[i] ].z + (current_graph_[ flight_plan_for_current_uav.nodes[i] ].altitude - map_origin_geo_.altitude);

            if (current_graph_[ flight_plan_for_current_uav.nodes[i] ].type != aerialcore_msgs::GraphNode::TYPE_PYLON) {

                if (pass_poses.size()>0) {
                    UAVs_[current_uav_index].mission->addPassWpList(pass_poses);
                }
                if (first_iteration) {
                    current_pose_stamped.pose.position.z = current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].altitude == 0 ? current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].z : current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].z + (current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].altitude - map_origin_geo_.altitude);
                    UAVs_[current_uav_index].mission->addTakeOffWp(current_pose_stamped);
                    flying_or_landed = true;
                    first_iteration=false;
                } else {
                    if (flying_or_landed) {
                        if (UAVs_[current_uav_index].mission->airframeType() == grvc::mission_ns::AirframeType::FIXED_WING) {
                            geometry_msgs::PoseStamped loiter_to_alt_start_landing_pose_pose_stamped;
                            loiter_to_alt_start_landing_pose_pose_stamped.pose.position.x = pass_poses.back().pose.position.x;
                            loiter_to_alt_start_landing_pose_pose_stamped.pose.position.y = pass_poses.back().pose.position.y;
                            loiter_to_alt_start_landing_pose_pose_stamped.pose.position.z = pass_poses.back().pose.position.z;
                            UAVs_[current_uav_index].mission->addLandWp(loiter_to_alt_start_landing_pose_pose_stamped, current_pose_stamped);
                        } else {
                            UAVs_[current_uav_index].mission->addLandWp(current_pose_stamped);
                        }
                        flying_or_landed = false;
                    } else {
                        current_pose_stamped.pose.position.z = current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].altitude == 0 ? current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].z : current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].z + (current_graph_[ flight_plan_for_current_uav.nodes[i+1] ].altitude - map_origin_geo_.altitude);
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


std::string MissionController::translateFlightPlanIntoDJIyaml(const std::vector<aerialcore_msgs::FlightPlan>& _flight_plan) {
    std::string yaml_to_return = {R"(frame_id: /gps

uav_n: )"};
    yaml_to_return.append(std::to_string(_flight_plan.size()).c_str());
    yaml_to_return.append("\n\n");
    for (const aerialcore_msgs::FlightPlan& flight_plan_for_current_uav : _flight_plan) {
        yaml_to_return.append("uav_");
        yaml_to_return.append(std::to_string(flight_plan_for_current_uav.uav_id).c_str());
        yaml_to_return.append({R"(:
  wp_n: )"});
        yaml_to_return.append(std::to_string(flight_plan_for_current_uav.nodes.size()-2 ).c_str());
        for (int i=1; i<flight_plan_for_current_uav.nodes.size()-1; i++) {
            yaml_to_return.append({R"(
  wp_)"});
            yaml_to_return.append(std::to_string(i-1).c_str());
            yaml_to_return.append(": [");
            yaml_to_return.append(std::to_string(current_graph_[flight_plan_for_current_uav.nodes[i]].latitude).c_str());
            yaml_to_return.append(", ");
            yaml_to_return.append(std::to_string(current_graph_[flight_plan_for_current_uav.nodes[i]].longitude).c_str());
            yaml_to_return.append(", ");
            yaml_to_return.append(std::to_string(current_graph_[flight_plan_for_current_uav.nodes[i]].altitude).c_str());
            yaml_to_return.append("]");
        }
        yaml_to_return.append("\n\n");
    }
#ifdef DEBUG
    std::cout << yaml_to_return << std::endl;
#endif
    return yaml_to_return;
} // end translateFlightPlanIntoDJIyaml



bool MissionController::stopSupervisingServiceCallback(aerialcore_msgs::StopSupervising::Request& _req, aerialcore_msgs::StopSupervising::Response& _res) {
    if (_req.uav_id.size()>0) {     // There are specific UAVs to stop.
        for (const int& current_uav_id : _req.uav_id) {
            int current_uav_index = findUavIndexById(current_uav_id);

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


bool MissionController::doSpecificSupervisionServiceCallback(aerialcore_msgs::DoSpecificSupervision::Request& _req, aerialcore_msgs::DoSpecificSupervision::Response& _res) {
    continuous_or_specific_supervision_ = false;
    specific_subgraph_.clear();
    for (const aerialcore_msgs::GraphNode& current_graph_node : _req.specific_subgraph) {
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


bool MissionController::startSpecificSupervisionPlanServiceCallback(aerialcore_msgs::PostString::Request& _req, aerialcore_msgs::PostString::Response& _res) {
    ROS_INFO("Mission Controller: start specific supervision plan servive received.");

    // Load to the parameter server the yaml string received:
    std::remove("current_plan.yaml");
    std::ofstream out("current_plan.yaml");
    out << _req.data;
    out.close();
    system("rosparam load current_plan.yaml");

    current_graph_.clear();

    // Iterate the parameters loaded and create from them the graph:
    for (int id=1; id<=20; id++) {
        if (ros::param::has("waypoints_"+std::to_string( id ))) {

            // Build the initial UAV positions and regular land stations (same spot where the UAV is when yaml string received) graph nodes for the current graph:
            aerialcore_msgs::GraphNode initial_graph_node;
            initial_graph_node.type = aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION;
            if (commanding_UAV_with_mission_lib_or_DJI_SDK_) {
                initial_graph_node.x = UAVs_[findUavIndexById(id)].mission->pose().pose.position.x;
                initial_graph_node.y = UAVs_[findUavIndexById(id)].mission->pose().pose.position.y;
                initial_graph_node.z = UAVs_[findUavIndexById(id)].mission->pose().pose.position.z;
            } else {
                // // HARDCODED INITIAL POSES ?
            }
            current_graph_.push_back(initial_graph_node);
            initial_graph_node.type = aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION;
            initial_graph_node.id = id;
            current_graph_.push_back(initial_graph_node);

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
                current_graph_.push_back(pylon_graph_node);
            }
        }
    }
#ifdef DEBUG
    for (int i=0; i<current_graph_.size(); i++) {      // Print current_graph_ to check that the yaml file was parsed correctly:
        std::cout << "graph_node[ " << i << " ].type                     = " << (int) current_graph_[i].type << std::endl;
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
#endif

    // Build the flight_plan_ so we can translate it later to use the mission_lib or DJI SDK:
    flight_plan_.clear();
    aerialcore_msgs::FlightPlan current_flight_plan;
    int regular_landing_station_index = -1;
    for (int i=0; i<=current_graph_.size(); i++) {
        if (i==current_graph_.size() || current_graph_[i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION && regular_landing_station_index!=-1) {
            current_flight_plan.nodes.push_back(regular_landing_station_index);
            flight_plan_.push_back(current_flight_plan);
            current_flight_plan.nodes.clear();
            if (i==current_graph_.size()) {
                break;
            }
        }
        if (current_graph_[i].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
            regular_landing_station_index = i-1;
            current_flight_plan.uav_id = current_graph_[i].id;
            current_flight_plan.nodes.push_back(i);
        } else if (current_graph_[i].type==aerialcore_msgs::GraphNode::TYPE_PYLON) {
            current_flight_plan.nodes.push_back(i);
        }
    }

    if (commanding_UAV_with_mission_lib_or_DJI_SDK_) {
        translateFlightPlanIntoUAVMission(flight_plan_);

        for (const aerialcore_msgs::FlightPlan& flight_plan_for_current_uav : flight_plan_) {
            // // HARDCODED WAITS BETWEEN TAKEOFFS FOR NOVEMBER EXPERIMENTS:
            // if (flight_plan_for_current_uav.uav_id==2) sleep(10);
            // if (flight_plan_for_current_uav.uav_id==3) sleep(74);
#ifdef DEBUG
            UAVs_[findUavIndexById(flight_plan_for_current_uav.uav_id)].mission->print();
#endif
            UAVs_[findUavIndexById(flight_plan_for_current_uav.uav_id)].mission->push();
            UAVs_[findUavIndexById(flight_plan_for_current_uav.uav_id)].mission->start();
        }
        _res.success=true;
    } else {
        aerialcore_msgs::PostString post_yaml_string;
        post_yaml_string.request.data = translateFlightPlanIntoDJIyaml(flight_plan_);
        if (post_yaml_client_.call(post_yaml_string)) {
            ROS_INFO("Mission Controller: yaml sent to DJI SDK.");
            _res.success=true;
        } else {
            ROS_WARN("Mission Controller: yaml not sent to DJI SDK, service didn't answer.");
            _res.success=false;
        }
    }

    return true;
} // end startSpecificSupervisionPlanServiceCallback


int MissionController::findUavIndexById(int _UAV_id) {
    int uav_index = -1;
    for (int i=0; i<UAVs_.size(); i++) {
        if (UAVs_[i].id == _UAV_id) {
            uav_index = i;
            break;
        }
    }
    if (uav_index == -1) {
        ROS_ERROR("Mission Controller: UAV id provided not found on the Mission Controller.");
        exit(EXIT_FAILURE);
    }
    return uav_index;
} // end findUavIndexById


} // end namespace aerialcore