#include "include/ErrorCell.h"


ErrorCell::ErrorCell(int x, int y){
	this.x = x;
	this.y = y;
}

void ErrorCell::insert_report(errorInsert& error){
	history.push_back(error);
}

ErrorInformation ErrorCell::aggregateError(int historyLength){
	int limit = history.size();
	if(historyLength > 0 && historyLength < limit){
		limit = historyLength;
	}
	ErrorInformation error = new ErrorInformation();
	/*
	error.quality = 0;
	error.linearError = 0;
	error.angularError = 0;
	error.age = 0;
	*/
	for (int i = 0; i < limit; ++i)
	{
		errorInsert item = history[i];
		int itemLinearError = 
	}
}