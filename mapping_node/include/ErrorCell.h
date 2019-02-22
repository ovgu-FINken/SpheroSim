#ifndef ErrorCell_H
#define ErrorCell_H

#include "mapping_node/errorInsert.h"
#include <vector>

using namespace mapping_node;

///
/// Definition for a single cell of the error map.
///
class ErrorCell {
	private:
		/// The history of all reported error messages concerning this cell.
		std::vector<ErrorInformation> history;
		
		/// Computes the error from the history of error reports on this cell with respect to the maximum age of reports.
		ErrorInformation* aggregateError(int);
	public:
		ErrorCell();
		~ErrorCell();
		
		/// The position of the cell on the map in x dimension
		int x;
		
		/// The position of the cell on the map in y dimension
		int y;
		
		/// Inserts the reported error into the history and updates the aggrated error.
		void insert_report(ErrorInformation);
		
		/// retrieves the aggregated error information for this cell.
		ErrorInformation* get_error();
}

#endif