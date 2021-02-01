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
#include <aerialcore_msgs/FlightPlan.h>
#include <aerialcore_msgs/GraphNode.h>

namespace aerialcore {

/// ParameterEstimator class that works as interface
class ParameterEstimator {

public:
    ParameterEstimator();
    ~ParameterEstimator();

    // Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
    // updateMatrices(poses, batteries, plan, wind sensor?)

    // Getters that the Mission Controller will use for the Planner and Plan Monitor:
    // getCostMatrix()
    // getBatteryDropMatrix()

private:
    // cost_matrix_
    // battery_drop_matrix_

};  // end ParameterEstimator class

}   // end namespace aerialcore

#endif  // PARAMETER_ESTIMATOR_H