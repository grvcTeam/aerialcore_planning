/**
 * AERIALCORE Project:
 *
 * Centralized planner.
 * 
 */

#include <centralized_planner.h>

namespace aerialcore {


// Brief Constructor
CentralizedPlanner::CentralizedPlanner() : path_planner_() {
}


// Brief Destructor
CentralizedPlanner::~CentralizedPlanner() {}


std::vector<aerialcore_msgs::FlightPlan> getPlan() {
} // end getPlan method


} // end namespace aerialcore
