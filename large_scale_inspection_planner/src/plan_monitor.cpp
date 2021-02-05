/**
 * AERIALCORE Project:
 *
 * Plan Monitor.
 * 
 */

#include <plan_monitor.h>

#include <math.h>

namespace aerialcore {


// Brief Constructor
PlanMonitor::PlanMonitor() {}


// Brief Destructor
PlanMonitor::~PlanMonitor() {}


// Method called periodically in an external thread, located in the Mission Controller, that will call the planner (in the same thread) if it returns true:
bool PlanMonitor::enoughDeviationToReplan(/*plan, pose, battery, */const std::vector< std::vector<float> >& _time_cost_matrix, const std::vector< std::vector<float> >& _battery_drop_matrix) {
    if (dummy_replan_only_once) {
        dummy_replan_only_once = false;
        return true;
    } else {
        return false;
    }
}


} // end namespace aerialcore
