namespace spheroSim {
	class ErrorMap {
		public:
			ErrorMap();
			ErrorMap(string csvPath, boolean header);
			ErrorInformation GetPositionError(geometryMsg::Pose2D pose);
		private:
			ErrorInformation* map;
	}
}