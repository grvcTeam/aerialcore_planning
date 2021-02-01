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

private:

};  // end PlanMonitor class

}   // end namespace aerialcore

#endif  // PLAN_MONITOR_H