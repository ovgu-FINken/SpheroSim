#include <sphero_error_inject/error_map.h>
#include <boost/program_options.hpp>
#include <string>
#include <fstream>
#include <iostream>

using namespace boost::program_options;

bool getPositionError(sphero_error_inject::Error::Request &req, sphero_error_inject::Error::Response &res) {
	Error error = ErrorMap::getInstance().GetPositionError(req.pose);
	res.linearError = error.linearError;
	res.angularError = error.angularError;
	return true;
}

int main(int argc, char **argv) {
	try {
		options_description fileOptions{"File"};

	    fileOptions.add_options()
	    	("map", value<std::tuple<int, float, float>>(), "Map");

	    variables_map vm;
        std::ifstream ifs{"[[PATH TO CONFIG FILE]]"};
	    if (ifs) {
	    	store(parse_config_file(ifs, fileOptions), vm);
	    }
	    // TODO: load the map into the error map
    	//vm["map"]
	} catch(const error &ex) {
		ROS_ERROR(ex.what());
	}
	ros::init(argc, argv, "error_map_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("get_position_error", getPositionError)
	ros::spin();

	return 0;
}