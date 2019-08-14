#include <sphero_error_mapping/error_cell.h>

using namespace sphero_error_mapping;

/**
 * Inserts an error report into the cell for future error computations.
 * \param error The pointer to the error to insert.
 */
void ErrorCell::insert_report(ErrorInformation* error){
	history.push_back(*error);
}

/**
 * Aggregates the available error information for a single cell of the map.
 * \param historyLength The number of events in the history to take into account for the calculation.
 * \return The aggregated error information of the cell.
 */
ErrorInformation* ErrorCell::aggregateError(int historyLength){
	int limit = history.size();
	if(historyLength > 0 && historyLength < limit){
		limit = historyLength;
	}
	ErrorInformation* error = new ErrorInformation();
	// the more measurements are takent into account, the closer the resulting quality is to 1
	error->quality = 1 - (1/limit);
	// compute the mean, since we have exact values.
	// TODO: compute via Kalman Filter for un-precise measurements
	float linearError = 0;
	float angularError = 0;
	for (int i = 0; i < limit; ++i)
	{
		ErrorInformation item = history[i];
		linearError += item.linearError;
		angularError += item.angularError;
	}
	error->linearError = linearError / limit;
	error->angularError = angularError / limit;
	return error;
}

/**
 * Retrieves the error on a single cell of the map.
 * \return The error information of the cell.
 */
ErrorInformation* ErrorCell::get_error(){
	return this->aggregateError(-1);
}