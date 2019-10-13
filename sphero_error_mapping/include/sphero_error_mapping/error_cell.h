#ifndef ErrorCell_H
#define ErrorCell_H

#include <sphero_error_mapping/error_information.h>
#include <sphero_error_mapping/kalman_params.h>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace sphero_error_mapping {

	///
	/// Definition for a single cell of the error map.
	///
	class ErrorCell {
		private:
			/// The history of all reported error messages concerning this cell.
			std::vector<ErrorInformation> history;
			
			/// Computes the error from the history of error reports on this cell with respect to the maximum age of reports.
			ErrorInformation* aggregateError(int);

			/// Updates a kalman estimation.
			void update_estimate(KalmanParams*, double);

			/// the estimation for linear error
			KalmanParams linearEstimate;

			/// the estimation for angular error
			KalmanParams angularEstimate;

		public:
			ErrorCell(int x, int y): x(x), y(y), linearEstimate(1, 2, 0.1e-5, 0.2), angularEstimate(1, 2, 0.1e-5, 0.2) {};
			
			/// The position of the cell on the map in x dimension
			int x;
			
			/// The position of the cell on the map in y dimension
			int y;
			
			/// Inserts the reported error into the history and updates the aggrated error.
			void insert_report(ErrorInformation*);
			
			/// retrieves the aggregated error information for this cell.
			ErrorInformation* get_error();
	};
}

#endif
