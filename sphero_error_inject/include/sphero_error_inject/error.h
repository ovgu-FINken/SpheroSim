#ifndef SPHEROSIM_ERROR
#define SPHEROSIM_ERROR

namespace spheroSim {
	/**
	 * POCO for definition of the error a cell on the map has.
	 */
	class Error {
		public:
			Error(float, float);
		private:
			float linearError;
			float angularError;
	};
}

#endif