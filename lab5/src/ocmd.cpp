#include "ros/ros.h"
#include "lab5/oint_point.h"
#include "lab5/oint_trajectory.h"
#include <cstdlib>

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "ocmd");

  
	if (!(argc == 2 || argc==5))
  	{
    		ROS_INFO("usage: ocmd x y z time \nor ocmd square");
    		return 1;
  	}	

	ros::NodeHandle n;
 	ros::ServiceClient p_client = n.serviceClient<lab5::oint_point>("oint_point");
 	ros::ServiceClient t_client = n.serviceClient<lab5::oint_trajectory>("oint_trajectory");

	lab5::oint_point p_srv;
	lab5::oint_trajectory t_srv;

	if (argc == 5 )
  	{
  		p_srv.request.x = atof(argv[1]);
  	    p_srv.request.y = atof(argv[2]);
	    p_srv.request.z = atof(argv[3]);
  	    p_srv.request.time = atof(argv[4]);
  	    
  	    if (p_client.call(p_srv))
      	{
        	ROS_INFO_STREAM(p_srv.response.status);
      	}
      	else
      	{
        	ROS_ERROR("Failed to call service");
        	return 1;
      	}
    		
  	}
  	else if (argc == 2 )
  	{
  		t_srv.request.type = argv[1];
  	    
  	    
  	    if (t_client.call(t_srv))
      	{
        	ROS_INFO_STREAM(t_srv.response.status);
      	}
      	else
      	{
        	ROS_ERROR("Failed to call service");
        	return 1;
      	}
    		
  	}




	return 0;
}
