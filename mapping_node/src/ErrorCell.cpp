#include "include/ErrorCell.h"


ErrorCell::ErrorCell(int x, int y){
	this.x = x;
	this.y = y;
}

void ErrorCell::insert_report(ErrorInformation& error){
	history.push_back(error);
}

ErrorInformation* ErrorCell::aggregateError(int historyLength){
	int limit = history.size();
	if(historyLength > 0 && historyLength < limit){
		limit = historyLength;
	}
	ErrorInformation error = new ErrorInformation();
	// quality is closer to 1, the more measurements are takent into account
	error.quality = 1 - (1/limit);
	// compute the mean, since we have exact values.
	// TODO: compute via Kalman Filter for un-precise measurements
	float linearError = 0;
	float angularError = 0;
	for (int i = 0; i < limit; ++i)
	{
		errorInsert item = history[i];
		linearError += item.linearError;
		angularError += item.angularError;
	}
	error.linearError = linearError / limit;
	error.angularError = angularError / limit;
	return &error;
}

ErrorInformation* ErrorCell::get_error(){
	return this.aggregateError(-1);
}