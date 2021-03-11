/**
 * AERIALCORE Project:
 *
 * Parameter Estimator.
 * 
 */

#ifndef PARAMETER_ESTIMATOR_H
#define PARAMETER_ESTIMATOR_H

#include <string>
#include <iostream>
#include <vector>
#include <tuple>
#include <utility>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geographic_msgs/GeoPoint.h>
#include <aerialcore_msgs/GraphNode.h>

#include <path_planner.h>

namespace aerialcore {

/// ParameterEstimator class that works as interface
class ParameterEstimator {

public:
    ParameterEstimator();
    ~ParameterEstimator();

    // Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
    void updateMatrices(const std::vector<aerialcore_msgs::GraphNode>& _graph, const std::vector<geometry_msgs::Polygon>& _no_fly_zones, const geometry_msgs::Polygon& _geofence /* poses, batteries, plan, wind sensor?*/);

    // Getters that the Mission Controller will use for the Planner and Plan Monitor:
    const std::vector< std::vector<float> >& getDistanceCostMatrix() const { return distance_cost_matrix_; }
    const std::vector< std::vector<float> >& getBatteryDropMatrix()  const { return battery_drop_matrix_; }

private:
    std::vector< std::vector<float> > distance_cost_matrix_;    // Square non-symetrical matrix, one row and column for each pylon and land station. The elements represent the time (in seconds) to cover that edge if exist, if not the value is -1.
    std::vector< std::vector<float> > battery_drop_matrix_;     // Square non-symetrical matrix, one row and column for each pylon and land station. The elements represent the battery drop (per unit, not %) to cover that edge if exist, if not the value is -1.

    grvc::PathPlanner path_planner_;

    bool construct_distance_cost_matrix_ = true;        // Parameter estimator constructs the distance_cost_matrix from the graph and export it to a default yaml file (if true), or swiftly read the last default yaml file of the graph (if false).
    std::string distance_cost_matrix_yaml_path_;

};  // end ParameterEstimator class

}   // end namespace aerialcore

#endif  // PARAMETER_ESTIMATOR_H