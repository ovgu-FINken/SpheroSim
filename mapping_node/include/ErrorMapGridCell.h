using namespace mapping_node;

///
/// A definition of the data that are included in any given grid cell of the map.
///
struct ErrorMapGridCell
{
	ErrorMapGridCell(int, int);
	~ErrorMapGridCell();
	
	/// The age of the data in respect to it's latest update.
	int age;

	/// An indicator for the (un-)vertainty included in the error data.
	int quality;
	
	/// An indication of how much the linar component of a movement on the given cell will vary.
	char linearError;

	/// An indication of how much the angular component of a movement on the given cell will vary.
	char angularError;

	/// The x coordinate of the cell on the map.
	int x;

	/// The y coordinate of the cell on the map.
	int y;
};