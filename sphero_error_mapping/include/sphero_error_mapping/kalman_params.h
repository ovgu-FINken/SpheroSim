#ifndef KalmanParams_H
#define KalmanParams_H

#include <eigen3/Eigen/Dense>

namespace sphero_error_mapping {

	///
	/// Definition for a single cell of the error map.
	///
	class KalmanParams {			
		public:

            /// Constructor for parameter initialization
			KalmanParams(double X, double P, double Q, double R): X(X), P(P), Q(Q), R(R) {};

			/// The state estimation
			double X;

			/// The estimation certainty/variance
			double P;

			/// Process noise, probably really small
			double Q;

            /// measurement uncertainty
            double R;
	};
}

#endif
