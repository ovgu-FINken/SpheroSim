
#ifndef ErrorInformation_H
#define ErrorInformation_H

namespace sphero_error_mapping {

	///
	/// Contains the aggregated error information for a single cell of the map.
	///
	class ErrorInformation
	{
		public:
			/// Indicates the propability of this error to be correct.
			int quality;

			/// The linear component of this error.
			int linearError;
			
			/// The angular component of this error.
			int angularError;
			
			/// The time since the error was updated.
			int age;
	};
}

#endif