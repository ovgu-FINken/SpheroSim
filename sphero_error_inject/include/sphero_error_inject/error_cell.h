#ifndef SPHEROSIM_ERRORCELL
#define SPHEROSIM_ERRORCELL

namespace spheroSim {
	/**
	 * POCO for definition of the error a cell on the map has.
	 */
	class ErrorCell {
		public:
			ErrorCell(): linearError(0), angularError(0) {};
			ErrorCell(float lin, float ang): linearError(lin), angularError(ang) {};
			float linearError;
			float angularError;
	};
}

#endif