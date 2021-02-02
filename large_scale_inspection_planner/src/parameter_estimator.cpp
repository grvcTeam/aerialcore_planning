/**
 * AERIALCORE Project:
 *
 * Parameter Estimator.
 * 
 */

#include <parameter_estimator.h>

#include <math.h>

namespace aerialcore {


// Brief Constructor
ParameterEstimator::ParameterEstimator() {}


// Brief Destructor
ParameterEstimator::~ParameterEstimator() {}

// Method called periodically in an external thread, located in the Mission Controller, that will update both the cost and battery drop matrices with the last prediction:
void ParameterEstimator::updateMatrices(/*poses, batteries, plan, wind sensor?*/) {
}


} // end namespace aerialcore
