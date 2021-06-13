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

// #define DEBUG       // UNCOMMENT FOR PRINTING VISUALIZATION OF RESULTS (DEBUG MODE)

namespace aerialcore {


// Brief Constructor
ParameterEstimator::ParameterEstimator() {
    ros::param::get("~construct_distance_cost_matrix", construct_distance_cost_matrix_);
    ros::param::get("~distance_cost_matrix_yaml_path", distance_cost_matrix_yaml_path_);

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


const std::vector< std::vector<float> >& ParameterEstimator::getDistanceCostMatrix() {
    distance_cost_matrix_mutex_.lock(); // Wait here until the attribute is fully created before the getter returns the reference.
    distance_cost_matrix_mutex_.unlock();
    return distance_cost_matrix_;
}


const std::map<int, std::vector< std::vector<float> > >& ParameterEstimator::getTimeCostMatrices() {
    time_cost_matrices_mutex_.lock();     // Wait here until the attribute is fully created before the getter returns the reference.
    time_cost_matrices_mutex_.unlock();
    return time_cost_matrices_;
}


const std::map<int, std::vector< std::vector<float> > >& ParameterEstimator::getBatteryDropMatrices() {
    battery_drop_matrices_mutex_.lock();  // Wait here until the attribute is fully created before the getter returns the reference.
    battery_drop_matrices_mutex_.unlock();
    return battery_drop_matrices_;
}


// Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
void ParameterEstimator::updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence /* poses, batteries, plan, wind sensor?*/) {

    if (distance_cost_matrix_.size()==0) {  // Only enter the first time this method is called.
        std::vector< std::vector<float> > new_distance_cost_matrix;
        if (construct_distance_cost_matrix_) {
            // Construct the distance_cost_matrix and export it to a default yaml file.

            // Construct the path_planner_:
            if (_geofence.points.size()>0 && _no_fly_zones.size()>0) {
                path_planner_ = grvc::PathPlanner(_no_fly_zones, _geofence);
            }

            // Create the first row and column of the matrix, which has this order: UAV initial positions, regular land stations, recharging land stations and pylons.
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
            std::vector<float> first_row_and_column;
            first_row_and_column.push_back(-1);
            first_row_and_column.insert(first_row_and_column.end(), uav_initial_positions_indexes.begin(), uav_initial_positions_indexes.end());
            first_row_and_column.insert(first_row_and_column.end(), uav_regular_land_stations_indexes.begin(), uav_regular_land_stations_indexes.end());
            first_row_and_column.insert(first_row_and_column.end(), uav_recharging_land_stations_indexes.begin(), uav_recharging_land_stations_indexes.end());
            first_row_and_column.insert(first_row_and_column.end(), pylons_indexes.begin(), pylons_indexes.end());

            // Initialize the distance_cost_matrix with -1 in all elements but those in the first row and column:
            new_distance_cost_matrix.push_back(first_row_and_column);
            std::vector<float> initial_rows(first_row_and_column.size(), -1);
            for (int i=1; i<first_row_and_column.size(); i++) {
                initial_rows[0] = first_row_and_column[i];
                new_distance_cost_matrix.push_back(initial_rows);
            }

            // Call the path planner between initial UAV poses and landing stations to pylons, and between pylons:
            geometry_msgs::Point32 from_here;
            geometry_msgs::Point32 to_here;
            for (int i=1; i<first_row_and_column.size(); i++) {
                for (int j= i+1 >= 1+uav_initial_positions_indexes.size()+uav_regular_land_stations_indexes.size()+uav_recharging_land_stations_indexes.size() ? i+1 : 1+uav_initial_positions_indexes.size()+uav_regular_land_stations_indexes.size()+uav_recharging_land_stations_indexes.size() ; j<first_row_and_column.size(); j++) {
                    from_here.x = _graph[ first_row_and_column[i] ].x;
                    from_here.y = _graph[ first_row_and_column[i] ].y;
                    from_here.z = _graph[ first_row_and_column[i] ].z;
                    to_here.x =   _graph[ first_row_and_column[j] ].x;
                    to_here.y =   _graph[ first_row_and_column[j] ].y;
                    to_here.z =   _graph[ first_row_and_column[j] ].z;
                    auto path = path_planner_.getPath(from_here, to_here);
                    if (path.size() == 0) { // No path found.
                        new_distance_cost_matrix[i][j] = -1;
                        new_distance_cost_matrix[j][i] = -1;
                    } else {
                        new_distance_cost_matrix[i][j] = path_planner_.getFlatDistance();   // TODO: more precise path with also precise height distances?
                        new_distance_cost_matrix[j][i] = path_planner_.getFlatDistance();
                    }
                }
            }

            // Now save this new distance_cost_matrix into the yaml file:
            std::string new_distance_cost_matrix_file_string = "distance_cost_matrix: [\n";
            for (int i=0; i<new_distance_cost_matrix.size(); i++) {
                new_distance_cost_matrix_file_string.append("  [");
                for (int j=0; j<new_distance_cost_matrix[i].size(); j++) {
                    if (j>0) {
                        new_distance_cost_matrix_file_string.append(", ");
                    }
                    new_distance_cost_matrix_file_string.append(std::to_string(new_distance_cost_matrix[i][j]).c_str());
                }
                new_distance_cost_matrix_file_string.append("],\n");
            }
            new_distance_cost_matrix_file_string.append("]");
            std::ofstream distance_cost_matrix_yaml_file(distance_cost_matrix_yaml_path_, std::ofstream::trunc);
            distance_cost_matrix_yaml_file << new_distance_cost_matrix_file_string;
            distance_cost_matrix_yaml_file.close();
        } else {
            // Import the distance_cost_matrix from the default yaml file:
            XmlRpc::XmlRpcValue distance_cost_matrix_XmlRpc;
            ros::param::get("distance_cost_matrix", distance_cost_matrix_XmlRpc);
            if (distance_cost_matrix_XmlRpc.getType()==XmlRpc::XmlRpcValue::TypeArray) {
                for (int i=0; i<distance_cost_matrix_XmlRpc.size(); i++) {
                    if (distance_cost_matrix_XmlRpc[i].getType()==XmlRpc::XmlRpcValue::TypeArray) {
                        std::vector<float> current_time_cost_row;
                        for (int j=0; j<distance_cost_matrix_XmlRpc[i].size(); j++) {
                            if (distance_cost_matrix_XmlRpc[i][j].getType()==XmlRpc::XmlRpcValue::TypeArray) {
                                std::string::size_type sz;

                                std::stringstream time_cost_element_stringstream;
                                time_cost_element_stringstream << distance_cost_matrix_XmlRpc[i][j];
                                float time_cost_element = (float) std::stod ( time_cost_element_stringstream.str() , &sz);

                                current_time_cost_row.push_back(time_cost_element);
                            }
                        }
                        new_distance_cost_matrix.push_back(current_time_cost_row);
                    }
                }
            }
        }
        distance_cost_matrix_mutex_.lock();
        distance_cost_matrix_ = new_distance_cost_matrix;
        distance_cost_matrix_mutex_.unlock();

        // For that distance_cost_matrix, create the time_cost_matrices for each UAV (depending on its speed):
        std::map<int, std::vector< std::vector<float> > > new_time_cost_matrices;
        for (int k=0; k<UAVs_.size(); k++) {    // The time is different for each UAV because of the different speed.
            std::vector< std::vector<float> > new_time_cost_matrix(distance_cost_matrix_.size(), std::vector<float>(distance_cost_matrix_[0].size(), -1.0));
            for (int i=0; i<distance_cost_matrix_.size(); i++) {
                for (int j=0; j<distance_cost_matrix_[i].size(); j++) {
                    if (i==0 || j==0 || distance_cost_matrix_[i][j]==-1) {
                        new_time_cost_matrix[i][j] = distance_cost_matrix_[i][j];
                    } else {
                        new_time_cost_matrix[i][j] = distance_cost_matrix_[i][j] / UAVs_[k].speed_xy;   // TODO: more precise speed with also height speeds?
                    }
                }
            }
            new_time_cost_matrices[ UAVs_[k].id ] = new_time_cost_matrix;
        }
        time_cost_matrices_mutex_.lock();
        time_cost_matrices_ = new_time_cost_matrices;
        time_cost_matrices_mutex_.unlock();
    }   // End of building cost matrices the first time this method is called.

    // Construct the battery_drop_matrices.
    std::map<int, std::vector< std::vector<float> > > new_battery_drop_matrices;
    for (int k=0; k<UAVs_.size(); k++) {
        std::vector< std::vector<float> > new_battery_drop_matrix(time_cost_matrices_[k].size(), std::vector<float>(time_cost_matrices_[k][0].size(), -1.0));
        for (int i=0; i<time_cost_matrices_[k].size(); i++) {
            for (int j=0; j<time_cost_matrices_[k][i].size(); j++) {
                if (i==0 || j==0 || time_cost_matrices_[k][i][j]==-1) {
                    new_battery_drop_matrix[i][j] = time_cost_matrices_[k][i][j];
                } else {
                    new_battery_drop_matrix[i][j] = time_cost_matrices_[k][i][j] / UAVs_[k].time_max_flying;    // TODO: calculate BETTER (consider WIND and BATTERY HEALTH, maybe also mAh?).
                }
            }
        }
        new_battery_drop_matrices[ UAVs_[k].id ] = new_battery_drop_matrix;
    }
    battery_drop_matrices_mutex_.lock();
    battery_drop_matrices_ = new_battery_drop_matrices;
    battery_drop_matrices_mutex_.unlock();

# ifdef DEBUG
    std::string time_cost_matrices_string = "time_cost_matrices: [\n";
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = time_cost_matrices_.begin(); it != time_cost_matrices_.end(); it++) {
        time_cost_matrices_string.append("  [\n");
        for (int i=0; i<it->second.size(); i++) {
            time_cost_matrices_string.append("    [");
            for (int j=0; j<it->second[i].size(); j++) {
                if (j>0) {
                    time_cost_matrices_string.append(", ");
                }
                time_cost_matrices_string.append(std::to_string(it->second[i][j]).c_str());
            }
            time_cost_matrices_string.append("],\n");
        }
        time_cost_matrices_string.append("  ],\n");
    }
    time_cost_matrices_string.append("]");

    std::string battery_drop_matrices_string = "battery_drop_matrices: [\n";
    for (std::map<int, std::vector< std::vector<float> > >::iterator it = battery_drop_matrices_.begin(); it != battery_drop_matrices_.end(); it++) {
        battery_drop_matrices_string.append("  [\n");
        for (int i=0; i<it->second.size(); i++) {
            battery_drop_matrices_string.append("    [");
            for (int j=0; j<it->second[i].size(); j++) {
                if (j>0) {
                    battery_drop_matrices_string.append(", ");
                }
                battery_drop_matrices_string.append(std::to_string(it->second[i][j]).c_str());
            }
            battery_drop_matrices_string.append("],\n");
        }
        battery_drop_matrices_string.append("  ],\n");
    }
    battery_drop_matrices_string.append("]");

    std::cout << time_cost_matrices_string << std::endl << std::endl;
    std::cout << battery_drop_matrices_string << std::endl << std::endl;
#endif

}


} // end namespace aerialcore
