#include <sphero_error_mapping/error_cell.h>
#include "ros/ros.h"

using namespace sphero_error_mapping;

/**
 * Inserts an error report into the cell for future error computations.
 * \param error The pointer to the error to insert.
 */
void ErrorCell::insert_report(ErrorInformation* error){
	history.push_back(*error);
	this->update_estimate(&(this->linearEstimate), error->linearError);
	this->update_estimate(&(this->angularEstimate), error->angularError);
}

/**
 * Updates a kalman estimation.
 * \param estimate The estimation to update.
 * \param measurement The new measurement to update the estimation with.
 * */
void ErrorCell::update_estimate(KalmanParams *estimate, double measurement) {
	// compute the kalman gain
	double k = estimate->P / ( estimate->P + estimate->R);
	// update the estimation
	estimate->X = estimate->X + (k * (measurement - estimate->X));
	// update the estimation uncertainty
	estimate->P = (1 - k) * estimate->P;
}

/**
 * Retrieves the error on a single cell of the map.
 * \return The error information of the cell.
 */
ErrorInformation* ErrorCell::get_error(){
	ErrorInformation* result = new ErrorInformation();
	result->linearError = this->linearEstimate.X;
	result->linearCovariance = this->linearEstimate.P;
	result->angularError = this->angularEstimate.X;
	result->angularCovariance = this->angularEstimate.P;
	return result;
}