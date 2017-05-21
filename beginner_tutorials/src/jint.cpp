#include "ros/ros.h"
#include "beginner_tutorials/jint_control_srv.h"
#include <math.h>
#include "sensor_msgs/JointState.h"

double t[3], prev_t[3];
double czas;
double init_t[]={0,0,0};
ros::Publisher jint_pub;

void fillmsg(sensor_msgs::JointState &msg, double pos[]) {
	msg.header.stamp=ros::Time::now();
	msg.name.push_back("base_to_link_1");
	msg.name.push_back("link_1_to_link_2");
	msg.name.push_back("link_2_to_link_3");

	msg.position.push_back(pos[0]);
	msg.position.push_back(pos[1]);
	msg.position.push_back(pos[2]);
}

bool interpolate(beginner_tutorials::jint_control_srv::Request &req, beginner_tutorials::jint_control_srv::Response &res) {
	t[0]=req.t1;
	t[1]=req.t2;
	t[2]=req.t3;
	czas=req.time;

	bool error=false;
	
	if(t[1]<-1.57 || t[1]>0.5) {
		ROS_ERROR_STREAM("theta2 incorrect");
		error=true;
	}
	if(t[2]<-1.57 || t[2]>1.57) {
		ROS_ERROR_STREAM("theta3 incorrect");
		error=true;
	}
	if(czas<=0) {
		ROS_ERROR_STREAM("time incorrect");
		error=true;
	}
	if(error) {
		res.status="Zadano niepoprawne parametry";
		return true;
	}
	for(int i=0; i<3; i++) {
		t[i]+=init_t[i];
	}

	double begin=ros::Time::now().toSec();
	double duration=ros::Time::now().toSec()-begin;

	ros::Rate loop_rate(100);

	double intT[3];

	sensor_msgs::JointState msg;

	while(duration<=czas) {
		for(int i=0; i<3; i++) {
			intT[i]=prev_t[i]+((t[i]-prev_t[i])/czas)*duration;
		}
		sensor_msgs::JointState msg;
		fillmsg(msg, intT);
		jint_pub.publish(msg);

		loop_rate.sleep();
		duration=ros::Time::now().toSec()-begin;
	}

	for(int i=0; i<3; i++) {
		prev_t[i]=intT[i];
	}
	res.status="Zakonczono interpolacje";
	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "jint");
	ros::NodeHandle n;
	ros::ServiceServer service=n.advertiseService("jint_control_srv", interpolate);
	jint_pub=n.advertise<sensor_msgs::JointState>("joint_states", 1);
	
	ros::Rate loop_rate(100);
	
	for(int i=0; i<3; i++) {
		prev_t[i]=0;
	}
	
	ROS_INFO("Ready to interpolate");

	while(ros::ok()) {
		sensor_msgs::JointState msg;
		fillmsg(msg, prev_t);
		jint_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
