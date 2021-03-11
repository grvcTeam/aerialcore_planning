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

namespace aerialcore {


// Brief Constructor
ParameterEstimator::ParameterEstimator() {
    ros::param::get("~construct_distance_cost_matrix", construct_distance_cost_matrix_);
    ros::param::get("~distance_cost_matrix_yaml_path", distance_cost_matrix_yaml_path_);
}


// Brief Destructor
ParameterEstimator::~ParameterEstimator() {}


// Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
void ParameterEstimator::updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence /* poses, batteries, plan, wind sensor?*/) {

    if (distance_cost_matrix_.size()==0) {  // Only enter the first time this function is called.
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
                    if (path.size() == 0) continue;         // No path found.
                    new_distance_cost_matrix[i][j] = path_planner_.getDistance();
                    new_distance_cost_matrix[j][i] = path_planner_.getDistance();
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
        distance_cost_matrix_ = new_distance_cost_matrix;
    }

    // Construct the battery_drop_matrix.
    std::vector< std::vector<float> > new_battery_drop_matrix;
    // ...
    battery_drop_matrix_ = new_battery_drop_matrix;

    // Update matrices, atomic variables doesn't work for vectors.
}


} // end namespace aerialcore
