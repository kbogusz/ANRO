#include "ros/ros.h"
#include "beginner_tutorials/oint_control_srv.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include <kdl/frames.hpp>

double pos[3], prev_pos[3];
double angle[3], prev_angle[3];
double czas;
ros::Publisher oint_pub;

void fillmsg(geometry_msgs::PoseStamped &msg, double pos[], double angle[]) {
	KDL::Rotation rot=KDL::Rotation::RPY(angle[0], angle[1], angle[2]);
	double x, y, z, w;
	rot.GetQuaternion(x, y, z, w);	

	msg.header.stamp=ros::Time::now();
	msg.header.frame_id="base_link";
	msg.pose.orientation.x=x;
	msg.pose.orientation.y=y;
	msg.pose.orientation.z=z;
	msg.pose.orientation.w=w;
	msg.pose.position.x=pos[0];
	msg.pose.position.y=pos[1];
	msg.pose.position.z=pos[2];
}

bool interpolate(beginner_tutorials::oint_control_srv::Request &req, beginner_tutorials::oint_control_srv::Response &res) {
	pos[0]=req.x;
	pos[1]=req.y;
	pos[2]=req.z;
	angle[0]=req.roll;
	angle[1]=req.pitch;
	angle[2]=req.yaw;
	czas=req.time;

	bool error=false;
	
	
	if(czas<=0) {
		ROS_ERROR_STREAM("czas poza zakresem");
		error=true;
	}
	if(error) {
		res.status="Zadanno niepoprawne parametry";
		return true;
	}
	
	double begin=ros::Time::now().toSec();
	double duration=ros::Time::now().toSec()-begin;

	ros::Rate loop_rate(100);

	double new_pos[3];
	double new_angle[3];
	

	while(duration<=czas) {
		for(int i=0; i<3; i++) {
			new_pos[i]=prev_pos[i]+((pos[i]-prev_pos[i])/czas)*duration;
			new_angle[i]=prev_angle[i]+((angle[i]-prev_angle[i])/czas)*duration;
		}
		geometry_msgs::PoseStamped msg;
		fillmsg(msg, new_pos, new_angle);
		oint_pub.publish(msg);

		loop_rate.sleep();
		duration=ros::Time::now().toSec()-begin;
	}

	for(int i=0; i<3; i++) {
		prev_pos[i]=new_pos[i];
		prev_angle[i]=new_angle[i];
	}
	res.status="Zakonczono interpolacje";
	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "oint");
	ros::NodeHandle n;
	ros::ServiceServer service=n.advertiseService("oint_control_srv", interpolate);
	oint_pub=n.advertise<geometry_msgs::PoseStamped>("/PoseStampedOINT", 1);
	
	ros::Rate loop_rate(100);
	
	for(int i=0; i<3; i++) {
		prev_pos[i]=0;
		prev_angle[i]=0;
	}
	
	ROS_INFO("Ready to interpolate");

	while(ros::ok()) {
		geometry_msgs::PoseStamped msg;
		fillmsg(msg, pos, angle);
		oint_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
