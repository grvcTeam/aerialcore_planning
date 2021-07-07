/**
 * AERIALCORE Project:
 *
 * Parameter Estimator.
 * 
 */

#include <parameter_estimator.h>

#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <XmlRpcValue.h>
#include <fstream>
#include <curl/curl.h>

#define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)

namespace aerialcore {


// Brief Constructor
ParameterEstimator::ParameterEstimator() {
    // ros::param::get("~construct_distance_cost_matrix", construct_distance_cost_matrix_);
    // ros::param::get("~distance_cost_matrix_yaml_path", distance_cost_matrix_yaml_path_);

    // Read and construct parameters of the UAVs (from the yaml):
    std::map<std::string, std::string> drones;
    ros::param::get("drones", drones);              // Dictionary of the actual drones used in the simulation (key: UAV id, value: airframe type).
    if (drones.size() == 0) {
        ROS_ERROR("Parameter Estimator: error in the description of the UAVs (YAML file), no drones found. Are you sure you loaded to the server parameter the config YAML?");
        exit(EXIT_FAILURE);
    }
    for (std::map<std::string, std::string>::iterator it = drones.begin(); it != drones.end(); it++) {
        UAV new_uav;
        new_uav.id = stoi(it->first);
        new_uav.airframe_type = it->second;

        ros::param::get(it->second+"/time_delay_between_wps", new_uav.time_delay_between_wps);
        if (new_uav.airframe_type == "MULTICOPTER") {
            ros::param::get(it->second+"/takeoff_climb_speed", new_uav.takeoff_climb_speed);
            ros::param::get(it->second+"/landing_descend_speed", new_uav.landing_descend_speed);
            ros::param::get(it->second+"/hardcoded_takeoff_landing_height", new_uav.hardcoded_takeoff_landing_height);

        } else if (new_uav.airframe_type == "FIXED_WING") {
            ros::param::get(it->second+"/time_delay_landing", new_uav.time_delay_landing);

        } else if (new_uav.airframe_type == "VTOL") {
            ros::param::get(it->second+"/takeoff_climb_speed", new_uav.takeoff_climb_speed);
            ros::param::get(it->second+"/landing_descend_speed", new_uav.landing_descend_speed);
            ros::param::get(it->second+"/time_delay_start_transition", new_uav.time_delay_start_transition);
            ros::param::get(it->second+"/time_delay_end_transition", new_uav.time_delay_end_transition);
        }

        ros::param::get(it->second+"/time_max_flying", new_uav.time_max_flying);
        ros::param::get(it->second+"/speed_xy", new_uav.speed_xy);
        ros::param::get(it->second+"/speed_z_down", new_uav.speed_z_down);
        ros::param::get(it->second+"/speed_z_up", new_uav.speed_z_up);
        ros::param::get(it->second+"/minimum_battery", new_uav.minimum_battery);
        ros::param::get(it->second+"/time_until_fully_charged", new_uav.time_until_fully_charged);
        UAVs_.push_back(new_uav);
    }
}


// Brief Destructor
ParameterEstimator::~ParameterEstimator() {}


const std::map<int, std::map<int, float> >& ParameterEstimator::getDistanceCostMatrix() {
    distance_cost_matrix_mutex_.lock(); // Wait here until the attribute is fully created before the getter returns the reference.
    distance_cost_matrix_mutex_.unlock();
    return distance_cost_matrix_;
}


const std::map<int, std::map<int, std::map<int, float> > >& ParameterEstimator::getTimeCostMatrices() {
    time_cost_matrices_mutex_.lock();     // Wait here until the attribute is fully created before the getter returns the reference.
    time_cost_matrices_mutex_.unlock();
    return time_cost_matrices_;
}


const std::map<int, std::map<int, std::map<int, float> > >& ParameterEstimator::getBatteryDropMatrices() {
    battery_drop_matrices_mutex_.lock();  // Wait here until the attribute is fully created before the getter returns the reference.
    battery_drop_matrices_mutex_.unlock();
    return battery_drop_matrices_;
}


// Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
void ParameterEstimator::updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence /* poses, batteries, plan, wind sensor?*/) {

    std::map<int, std::map<int, float> > new_distance_cost_matrix;
    std::map<int, std::map<int, Wps> > new_paths_matrix;
    std::map<int, std::map<int, std::map<int, float> > > new_time_cost_matrices;

    if (distance_cost_matrix_.size()==0 || true /* bypass this */) {  // Only enter the first time this method is called. // TODO: initial position changing all the time.
        // if (construct_distance_cost_matrix_) {
            // Construct the distance_cost_matrix and export it to a default yaml file.

            // Construct the path_planner_:
            if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
                path_planner_ = grvc::PathPlanner(_no_fly_zones, _geofence);
            }

            // Create the vector with the nodes, which has this order: UAV initial positions, regular land stations, recharging land stations and pylons.
            std::vector<int> uav_initial_positions_indexes, uav_regular_land_stations_indexes, uav_recharging_land_stations_indexes, pylons_indexes;
            for (int i=0; i<_graph.size(); i++) {
                if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION) {
                    uav_initial_positions_indexes.push_back(i);
                } else if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION) {
                    uav_regular_land_stations_indexes.push_back(i);
                } else if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) {
                    uav_recharging_land_stations_indexes.push_back(i);
                } else if (_graph[i].type == aerialcore_msgs::GraphNode::TYPE_PYLON) {
                    pylons_indexes.push_back(i);
                }
            }
            nodes_indexes_in_order_.clear();
            nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), uav_initial_positions_indexes.begin(), uav_initial_positions_indexes.end());
            nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), uav_regular_land_stations_indexes.begin(), uav_regular_land_stations_indexes.end());
            nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), uav_recharging_land_stations_indexes.begin(), uav_recharging_land_stations_indexes.end());
            nodes_indexes_in_order_.insert(nodes_indexes_in_order_.end(), pylons_indexes.begin(), pylons_indexes.end());

            // Initialize the distance_cost_matrix with -1 in all elements:
            for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
                for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                    new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = -1;
                }
            }

            // Call the path planner between initial UAV poses and landing stations to pylons, and between pylons:
            geometry_msgs::Point32 from_here;
            geometry_msgs::Point32 to_here;
            for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
                for (int j= i >= uav_initial_positions_indexes.size()+uav_regular_land_stations_indexes.size()+uav_recharging_land_stations_indexes.size() ? i+1 : uav_initial_positions_indexes.size()+uav_regular_land_stations_indexes.size()+uav_recharging_land_stations_indexes.size() ; j<nodes_indexes_in_order_.size(); j++) {
                    from_here.x = _graph[ nodes_indexes_in_order_[i] ].x;
                    from_here.y = _graph[ nodes_indexes_in_order_[i] ].y;
                    from_here.z = _graph[ nodes_indexes_in_order_[i] ].z;
                    to_here.x =   _graph[ nodes_indexes_in_order_[j] ].x;
                    to_here.y =   _graph[ nodes_indexes_in_order_[j] ].y;
                    to_here.z =   _graph[ nodes_indexes_in_order_[j] ].z;
                    auto path = path_planner_.getPath(from_here, to_here);
                    if (path.size() == 0) { // No path found.
                        new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = -1;
                        new_distance_cost_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = -1;

                        Wps empty_wps;
                        new_paths_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = empty_wps;
                        new_paths_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = empty_wps;
                    } else {
                        new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = path_planner_.getFlatDistance();   // TODO: more precise path with also precise height distances?
                        new_distance_cost_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = path_planner_.getFlatDistance();

                        Wps new_wps, new_wps_reversed;
                        new_wps.path = path;
                        path.insert(path.begin(), from_here);   // Path returned doesn' contain the first wp, but it will be convenient to have it.
                        for (int k=1; k<path.size(); k++) {
                            new_wps.angle.push_back( atan2(path[k].y-path[k-1].y, path[k].x-path[k-1].x) );
                            new_wps.distance.push_back( sqrt( pow(path[k].y-path[k-1].y,2) + pow(path[k].x-path[k-1].x,2) ) );
                        }
                        new_paths_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = new_wps;
                        for (int k=path.size()-2; k>=0; k--) {
                            new_wps_reversed.path.push_back( path[k] );
                            new_wps_reversed.angle.push_back( M_PI + new_wps.angle[k] );
                            new_wps_reversed.distance.push_back( new_wps.distance[k] );
                        }
                        new_paths_matrix[ nodes_indexes_in_order_[j] ][ nodes_indexes_in_order_[i] ] = new_wps_reversed;

                    }
                }
            }

        //     // Now save this new distance_cost_matrix into the yaml file:
        //     std::string new_distance_cost_matrix_file_string = "distance_cost_matrix: [\n";
        //     for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
        //         new_distance_cost_matrix_file_string.append("  [");
        //         if (i==0) {
        //             new_distance_cost_matrix_file_string.append(std::to_string(-1.0).c_str());
        //             new_distance_cost_matrix_file_string.append(", ");
        //             for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
        //                 if (j>0) {
        //                     new_distance_cost_matrix_file_string.append(", ");
        //                 }
        //                 new_distance_cost_matrix_file_string.append(std::to_string( nodes_indexes_in_order_[j] ).c_str());
        //             }
        //             new_distance_cost_matrix_file_string.append("],\n  [");
        //         }
        //         new_distance_cost_matrix_file_string.append(std::to_string( nodes_indexes_in_order_[i] ).c_str());
        //         new_distance_cost_matrix_file_string.append(", ");
        //         for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
        //             if (j>0) {
        //                 new_distance_cost_matrix_file_string.append(", ");
        //             }
        //             new_distance_cost_matrix_file_string.append(std::to_string(new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ]).c_str());
        //         }
        //         new_distance_cost_matrix_file_string.append("],\n");
        //     }
        //     new_distance_cost_matrix_file_string.append("]");
        //     std::ofstream distance_cost_matrix_yaml_file(distance_cost_matrix_yaml_path_, std::ofstream::trunc);
        //     distance_cost_matrix_yaml_file << new_distance_cost_matrix_file_string;
        //     distance_cost_matrix_yaml_file.close();
        // } else {
        //     std::vector< std::vector<float> > new_distance_cost_matrix_vector;
        //     // Import the distance_cost_matrix from the default yaml file:
        //     XmlRpc::XmlRpcValue distance_cost_matrix_XmlRpc;
        //     ros::param::get("distance_cost_matrix", distance_cost_matrix_XmlRpc);
        //     if (distance_cost_matrix_XmlRpc.getType()==XmlRpc::XmlRpcValue::TypeArray) {
        //         for (int i=0; i<distance_cost_matrix_XmlRpc.size(); i++) {
        //             if (distance_cost_matrix_XmlRpc[i].getType()==XmlRpc::XmlRpcValue::TypeArray) {
        //                 std::vector<float> current_time_cost_row;
        //                 for (int j=0; j<distance_cost_matrix_XmlRpc[i].size(); j++) {
        //                     if (distance_cost_matrix_XmlRpc[i][j].getType()==XmlRpc::XmlRpcValue::TypeArray) {
        //                         std::string::size_type sz;

        //                         std::stringstream time_cost_element_stringstream;
        //                         time_cost_element_stringstream << distance_cost_matrix_XmlRpc[i][j];
        //                         float time_cost_element = (float) std::stod ( time_cost_element_stringstream.str() , &sz);

        //                         current_time_cost_row.push_back(time_cost_element);
        //                     }
        //                 }
        //                 new_distance_cost_matrix_vector.push_back(current_time_cost_row);
        //             }
        //         }
        //     }

        //     // Once buildt the distance matrix with vectors, pass it to maps:
        //     for (int i=1; i<new_distance_cost_matrix_vector.size(); i++) {
        //         for (int j=1; j<new_distance_cost_matrix_vector.size(); j++) {
        //             new_distance_cost_matrix[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ] = new_distance_cost_matrix_vector[i][j];
        //         }
        //     }
        // }
        distance_cost_matrix_mutex_.lock();
        distance_cost_matrix_.clear();
        distance_cost_matrix_ = new_distance_cost_matrix;
        paths_matrix_.clear();
        paths_matrix_ = new_paths_matrix;
        distance_cost_matrix_mutex_.unlock();

        // For that distance_cost_matrix, create the time_cost_matrices for each UAV (depending on its speed):
        for (int k=0; k<UAVs_.size(); k++) {    // The time is different for each UAV because of the different speed.
            std::map<int, std::map<int, float> > new_time_cost_matrix;
            for (std::map<int, std::map<int, float> >::iterator it_i = new_distance_cost_matrix.begin(); it_i != new_distance_cost_matrix.end(); it_i++) {
                for (std::map<int, float>::iterator it_j = it_i->second.begin(); it_j != it_i->second.end(); it_j++) {
                    if (new_distance_cost_matrix[ it_i->first ][ it_j->first ]==-1) {
                        new_time_cost_matrix[ it_i->first ][ it_j->first ] = new_distance_cost_matrix[ it_i->first ][ it_j->first ];
                    } else {

                        if (UAVs_[k].airframe_type == "MULTICOPTER") {

                            if (_graph[it_i->first].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION
                            || _graph[it_i->first].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
                            || _graph[it_i->first].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Takeoff edge with navigation for a multicopter.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = UAVs_[k].hardcoded_takeoff_landing_height / UAVs_[k].takeoff_climb_speed
                                + new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * (new_paths_matrix[ it_i->first ][ it_j->first ].path.size() + 1);

                                new_time_cost_matrix[ it_i->first ][ it_j->first ] += _graph[it_j->first].z > UAVs_[k].hardcoded_takeoff_landing_height ? (_graph[it_j->first].z - UAVs_[k].hardcoded_takeoff_landing_height)/UAVs_[k].speed_z_up : (UAVs_[k].hardcoded_takeoff_landing_height - _graph[it_j->first].z)/UAVs_[k].speed_z_down ;

                            } else if (_graph[it_j->first].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
                            || _graph[it_j->first].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Landing edge with navigation for a multicopter.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = UAVs_[k].hardcoded_takeoff_landing_height / UAVs_[k].landing_descend_speed
                                + new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * (new_paths_matrix[ it_i->first ][ it_j->first ].path.size() + 1);

                                new_time_cost_matrix[ it_i->first ][ it_j->first ] += _graph[it_i->first].z > UAVs_[k].hardcoded_takeoff_landing_height ? (_graph[it_i->first].z - UAVs_[k].hardcoded_takeoff_landing_height)/UAVs_[k].speed_z_down : (UAVs_[k].hardcoded_takeoff_landing_height - _graph[it_i->first].z)/UAVs_[k].speed_z_up ;

                            } else {                                                                                // Regular navigation or inspection edge for a multicopter.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * new_paths_matrix[ it_i->first ][ it_j->first ].path.size();
                            }

                        } else if (UAVs_[k].airframe_type == "FIXED_WING") {

                            if (_graph[it_j->first].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
                            || _graph[it_j->first].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Landing edge with navigation for a fixed wing.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * (new_paths_matrix[ it_i->first ][ it_j->first ].path.size() + 1)
                                + UAVs_[k].time_delay_landing;

                            } else {                                                                                // Takeoff or regular navigation or inspection edge for a fixed wing.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * new_paths_matrix[ it_i->first ][ it_j->first ].path.size();
                            }

                        } else if (UAVs_[k].airframe_type == "VTOL") {

                            if (_graph[it_i->first].type==aerialcore_msgs::GraphNode::TYPE_UAV_INITIAL_POSITION
                            || _graph[it_i->first].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
                            || _graph[it_i->first].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Takeoff edge with navigation for a VTOL.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = (_graph[it_j->first].z-_graph[it_i->first].z) / UAVs_[k].takeoff_climb_speed
                                + new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * (new_paths_matrix[ it_i->first ][ it_j->first ].path.size() + 1)
                                + UAVs_[k].time_delay_start_transition;

                            } else if (_graph[it_j->first].type==aerialcore_msgs::GraphNode::TYPE_REGULAR_LAND_STATION
                            || _graph[it_j->first].type==aerialcore_msgs::GraphNode::TYPE_RECHARGE_LAND_STATION) { // Landing edge with navigation for a VTOL.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = (_graph[it_i->first].z-_graph[it_j->first].z) / UAVs_[k].landing_descend_speed
                                + new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * (new_paths_matrix[ it_i->first ][ it_j->first ].path.size() + 1)
                                + UAVs_[k].time_delay_end_transition;

                            } else {                                                                                // Regular navigation or inspection edge for a VTOL.
                                new_time_cost_matrix[ it_i->first ][ it_j->first ] = new_distance_cost_matrix[ it_i->first ][ it_j->first ] / UAVs_[k].speed_xy
                                + UAVs_[k].time_delay_between_wps * new_paths_matrix[ it_i->first ][ it_j->first ].path.size();
                            }

                        }

                    }
                }
            }
            new_time_cost_matrices[ UAVs_[k].id ] = new_time_cost_matrix;
        }
        time_cost_matrices_mutex_.lock();
        time_cost_matrices_.clear();
        time_cost_matrices_ = new_time_cost_matrices;
        time_cost_matrices_mutex_.unlock();
    }   // End of building cost matrices the first time this method is called.

    // Construct the battery_drop_matrices.
    std::map<int, std::map<int, std::map<int, float> > > new_battery_drop_matrices;
    for (std::map<int, std::map<int, std::map<int, float> > >::iterator it_k = new_time_cost_matrices.begin(); it_k != new_time_cost_matrices.end(); it_k++) {
        std::map<int, std::map<int, float> > new_battery_drop_matrix;
        int uav_index = findUavIndexById(it_k->first);
        for (std::map<int, std::map<int, float> >::iterator it_i = it_k->second.begin(); it_i != it_k->second.end(); it_i++) {
            for (std::map<int, float>::iterator it_j = it_i->second.begin(); it_j != it_i->second.end(); it_j++) {
                if (it_j->second==-1) {
                    new_battery_drop_matrix[ it_i->first ][ it_j->first ] = it_j->second;
                } else {
                    if (UAVs_[uav_index].airframe_type == "MULTICOPTER") {
                        new_battery_drop_matrix[ it_i->first ][ it_j->first ] = batteryDropMulticopter(it_k->first, it_i->first, it_j->first, new_time_cost_matrices);
                    } else if (UAVs_[uav_index].airframe_type == "FIXED_WING") {
                        new_battery_drop_matrix[ it_i->first ][ it_j->first ] = batteryDropFixedWing(it_k->first, it_i->first, it_j->first, new_time_cost_matrices);
                    } else if (UAVs_[uav_index].airframe_type == "VTOL") {
                        new_battery_drop_matrix[ it_i->first ][ it_j->first ] = batteryDropVTOL(it_k->first, it_i->first, it_j->first, new_time_cost_matrices);
                    }
                }
            }
        }
        new_battery_drop_matrices[ it_k->first ] = new_battery_drop_matrix;
    }
    battery_drop_matrices_mutex_.lock();
    battery_drop_matrices_.clear();
    battery_drop_matrices_ = new_battery_drop_matrices;
    battery_drop_matrices_mutex_.unlock();

# ifdef DEBUG
    getWindFromInternet();

    std::string time_cost_matrices_string = "time_cost_matrices: [\n";
    time_cost_matrices_mutex_.lock();
    for (std::map<int, std::map<int, std::map<int, float> > >::iterator it_k = time_cost_matrices_.begin(); it_k != time_cost_matrices_.end(); it_k++) {
        time_cost_matrices_string.append("  [ uav_id = ");
        time_cost_matrices_string.append(std::to_string( it_k->first ).c_str());
        time_cost_matrices_string.append("\n");
        for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
            time_cost_matrices_string.append("    [");
            if (i==0) {
                time_cost_matrices_string.append(std::to_string(-1.0).c_str());
                time_cost_matrices_string.append(", ");
                for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                    if (j>0) {
                        time_cost_matrices_string.append(", ");
                    }
                    time_cost_matrices_string.append(std::to_string( nodes_indexes_in_order_[j] ).c_str());
                }
                time_cost_matrices_string.append("],\n    [");
            }
            time_cost_matrices_string.append(std::to_string( nodes_indexes_in_order_[i] ).c_str());
            time_cost_matrices_string.append(", ");
            for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                if (j>0) {
                    time_cost_matrices_string.append(", ");
                }
                time_cost_matrices_string.append(std::to_string( it_k->second[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ]).c_str());
            }
            time_cost_matrices_string.append("],\n");
        }
        time_cost_matrices_string.append("  ],\n");
    }
    time_cost_matrices_string.append("]");
    time_cost_matrices_mutex_.unlock();

    std::string battery_drop_matrices_string = "battery_drop_matrices: [\n";
    battery_drop_matrices_mutex_.lock();
    for (std::map<int, std::map<int, std::map<int, float> > >::iterator it_k = battery_drop_matrices_.begin(); it_k != battery_drop_matrices_.end(); it_k++) {
        battery_drop_matrices_string.append("  [ uav_id = ");
        battery_drop_matrices_string.append(std::to_string( it_k->first ).c_str());
        battery_drop_matrices_string.append("\n");
        for (int i=0; i<nodes_indexes_in_order_.size(); i++) {
            battery_drop_matrices_string.append("    [");
            if (i==0) {
                battery_drop_matrices_string.append(std::to_string(-1.0).c_str());
                battery_drop_matrices_string.append(", ");
                for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                    if (j>0) {
                        battery_drop_matrices_string.append(", ");
                    }
                    battery_drop_matrices_string.append(std::to_string( nodes_indexes_in_order_[j] ).c_str());
                }
                battery_drop_matrices_string.append("],\n    [");
            }
            battery_drop_matrices_string.append(std::to_string( nodes_indexes_in_order_[i] ).c_str());
            battery_drop_matrices_string.append(", ");
            for (int j=0; j<nodes_indexes_in_order_.size(); j++) {
                if (j>0) {
                    battery_drop_matrices_string.append(", ");
                }
                battery_drop_matrices_string.append(std::to_string( it_k->second[ nodes_indexes_in_order_[i] ][ nodes_indexes_in_order_[j] ]).c_str());
            }
            battery_drop_matrices_string.append("],\n");
        }
        battery_drop_matrices_string.append("  ],\n");
    }
    battery_drop_matrices_string.append("]");
    battery_drop_matrices_mutex_.unlock();

    std::cout << time_cost_matrices_string << std::endl << std::endl;
    std::cout << battery_drop_matrices_string << std::endl << std::endl;
#endif

}


void ParameterEstimator::getWindFromInternet() {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        // Prepare the URL for cURL:
        std::string url = "https://api.openweathermap.org/data/2.5/weather?lat=38.138728&lon=-3.173825&appid=73bc471b3a4c39f688d5c0e79647db71";   // TODO: as a parameter.

        // Get string data to the readBuffer from the URL with cURL:
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        // std::cout << readBuffer << std::endl;
    }

    if (readBuffer.size() == 0) {
        ROS_ERROR("Parameter Estimator: openweathermap didn't answer. Returning empty wind vector (no wind).");
    } else {
        std::string wind_speed_search_pattern = "\"speed\":";       // 8 elements
        std::string wind_direction_deg_search_pattern = "\"deg\":"; // 6 elements
        std::string end_pattern = ",";

        // Find the place where the first wind direction and speed are:
        size_t wind_speed_found = readBuffer.find(wind_speed_search_pattern);
        size_t wind_speed_end_found = readBuffer.find(end_pattern, wind_speed_found+8);
        size_t wind_direction_deg_found = readBuffer.find(wind_direction_deg_search_pattern);
        size_t wind_direction_deg_end_found = readBuffer.find(end_pattern, wind_direction_deg_found+6);

        if (wind_speed_found==std::string::npos || wind_speed_end_found==std::string::npos || wind_direction_deg_found==std::string::npos || wind_direction_deg_end_found==std::string::npos) {
            ROS_ERROR("Parameter Estimator: parse error from openweathermap");
        }

        std::string wind_speed_string = readBuffer.substr(wind_speed_found+8, wind_speed_end_found-(wind_speed_found+8));
        std::string wind_direction_deg_string = readBuffer.substr(wind_direction_deg_found+6, wind_direction_deg_end_found-(wind_direction_deg_found+6));

# ifdef DEBUG
        // std::cout << "wind_speed_found = "<< wind_speed_found << std::endl;
        // std::cout << "wind_speed_end_found = "<< wind_speed_end_found << std::endl;
        // std::cout << "wind_direction_deg_found = "<< wind_direction_deg_found << std::endl;
        // std::cout << "wind_direction_deg_end_found = "<< wind_direction_deg_end_found << std::endl;
        std::cout << std::endl << "wind_speed_string = " << wind_speed_string << std::endl;
        std::cout << "wind_direction_deg_string = " << wind_direction_deg_string << std::endl << std::endl;
#endif

        std::string::size_type sz;

        std::stringstream wind_speed_stringstream, wind_direction_deg_stringstream;
        wind_speed_stringstream << wind_speed_string;
        wind_direction_deg_stringstream << wind_direction_deg_string;

        float wind_direction_deg;

        try {   // try to parse into float the wind strings...
            wind_speed_ = (float) std::stod ( wind_speed_stringstream.str() , &sz);
            wind_direction_deg = (float) std::stod ( wind_direction_deg_stringstream.str() , &sz);
        } catch (...) { // catch any exception
            ROS_ERROR("Parameter Estimator: error reading the parsed string from openweathermap");
            return;
        }

        // Wind direction is reported by the direction from which it originates (from which is COMING). For example, a north wind blows from the north to the south, with a direction angle of 0° (or 360°). A wind blowing from the east has a wind direction referred to as 90°, etc. 
        wind_vector_.x = -wind_speed_*sin(wind_direction_deg*M_PI/180.0);
        wind_vector_.y = -wind_speed_*cos(wind_direction_deg*M_PI/180.0);
        wind_vector_.z = 0;

# ifdef DEBUG
        std::cout << "wind_speed_ = " << wind_speed_ << std::endl;
        std::cout << "wind_vector_.x = " << wind_vector_.x << std::endl;
        std::cout << "wind_vector_.y = " << wind_vector_.y << std::endl;
        std::cout << "wind_vector_.z = " << wind_vector_.z << std::endl << std::endl;
#endif
    }
}


size_t ParameterEstimator::writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}   // end writeCallback


float ParameterEstimator::batteryDropMulticopter(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices) { // TODO: more parameters needed (propellers radious, mass, etc.)
    return _time_cost_matrices.at(_uav_id).at(_index_i).at(_index_j) / UAVs_[ findUavIndexById(_uav_id) ].time_max_flying;    // TODO: calculate BETTER (consider WIND and BATTERY HEALTH, UAV type, maybe also mAh?).
}


float ParameterEstimator::batteryDropFixedWing(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices) {
    return _time_cost_matrices.at(_uav_id).at(_index_i).at(_index_j) / UAVs_[ findUavIndexById(_uav_id) ].time_max_flying;    // TODO: calculate BETTER (consider WIND and BATTERY HEALTH, UAV type, maybe also mAh?).
}


float ParameterEstimator::batteryDropVTOL(int _uav_id, int _index_i, int _index_j, const std::map<int, std::map<int, std::map<int, float> > >& _time_cost_matrices) {
    return _time_cost_matrices.at(_uav_id).at(_index_i).at(_index_j) / UAVs_[ findUavIndexById(_uav_id) ].time_max_flying;    // TODO: calculate BETTER (consider WIND and BATTERY HEALTH, UAV type, maybe also mAh?).
}


int ParameterEstimator::findUavIndexById(int _UAV_id) {
    int uav_index = -1;
    for (int i=0; i<UAVs_.size(); i++) {
        if (UAVs_[i].id == _UAV_id) {
            uav_index = i;
            break;
        }
    }
    return uav_index;
} // end findUavIndexById


} // end namespace aerialcore
