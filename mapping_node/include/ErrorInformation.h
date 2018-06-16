using namespace mapping_node;

///
/// Contains the aggregated error information for a single cell of the map.
///
struct ErrorInformation
{
	/// Indicates the propability of this error to be correct.
	int quality;

	/// The linear component of this error.
	int linearError;
	
	/// The angular component of this error.
	int angularError;
	
	/// The time since the error was updated.
	int age;
};