#include "ros/ros.h"
#include "beginner_tutorials/jint_control_srv.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "jcmd");
  
	if (argc != 5) {
		ROS_INFO("uzycie: JCMD teta1 teta2 teta3 czas");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::jint_control_srv>("jint_control_srv");

	beginner_tutorials::jint_control_srv srv;

	srv.request.t1 = atof(argv[1]);
	srv.request.t2 = atof(argv[2]);
	srv.request.t3 = atof(argv[3]);
	srv.request.time = atof(argv[4]);
	
	if (client.call(srv)) {
		ROS_INFO_STREAM(srv.response.status);
	}
	else {
	ROS_ERROR("Failed to call service");
	return 1;
	}

	return 0;
}
