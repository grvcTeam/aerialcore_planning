/**
 * AERIALCORE Project:
 *
 * Plan Monitor.
 * 
 */

#ifndef PLAN_MONITOR_H
#define PLAN_MONITOR_H

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

/// PlanMonitor class that works as interface
class PlanMonitor {

public:
    PlanMonitor();
    ~PlanMonitor();

    // Method called periodically in an external thread, located in the Mission Controller, that will call the planner (in the same thread) if it returns true:
    bool enoughDeviationToReplan(/*plan, pose, battery, */const std::vector< std::vector<float> >& _distance_cost_matrix, const std::vector< std::vector<float> >& _battery_drop_matrix);

private:

    // Deviation metrics if needed.
    bool dummy_replan_only_once = true;

};  // end PlanMonitor class

}   // end namespace aerialcore

#endif  // PLAN_MONITOR_H