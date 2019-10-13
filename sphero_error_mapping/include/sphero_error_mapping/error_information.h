
#ifndef ErrorInformation_H
#define ErrorInformation_H

namespace sphero_error_mapping {

	///
	/// Contains the aggregated error information for a single cell of the map.
	///
	class ErrorInformation
	{
		public:
			/// The linear component of this error.
			double linearError;

			/// The covariance for the estimation of the linear Error
			double linearCovariance;
			
			/// The angular component of this error.
			double angularError;

			/// The covariance for the estimation of the angular Error
			double angularCovariance;
	};
}

#endif