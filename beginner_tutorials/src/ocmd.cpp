#include "ros/ros.h"
#include "beginner_tutorials/oint_control_srv.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "ocmd");
  
	if (argc != 8) {
		ROS_INFO("usage: OCMD x y z roll pitch yaw time");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::oint_control_srv>("oint_control_srv");

	beginner_tutorials::oint_control_srv srv;

	srv.request.x = atof(argv[1]);
	srv.request.y = atof(argv[2]);
	srv.request.z = atof(argv[3]);
	srv.request.roll = atof(argv[4]);
	srv.request.pitch = atof(argv[5]);
	srv.request.yaw = atof(argv[6]);
	srv.request.time = atof(argv[7]);
	
	if (client.call(srv)) {
		ROS_INFO_STREAM(srv.response.status);
	}
	else {
	ROS_ERROR("Failed to call service");
	return 1;
	}

	return 0;
}
